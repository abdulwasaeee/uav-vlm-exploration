#!/usr/bin/env python3
"""
VLM Unified Drone Control v2
Single LLM planning call handles everything.
No regex parsing - LLM decomposes all commands.
Memory system for context.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from cv_bridge import CvBridge
import cv2, base64, json, requests, os
import threading, math, time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum

OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
API_KEY   = OLLAMA_API_KEY or OPENAI_API_KEY
API_URL   = "https://ollama.com/v1/chat/completions" if OLLAMA_API_KEY else "https://api.openai.com/v1/chat/completions"
API_MODEL = "gemma3:4b" if OLLAMA_API_KEY else "gpt-4o"

MASTER_PROMPT = """You are a UAV drone controller AI.

Drone state:
- Position: x={x:.1f}, y={y:.1f}, z={z:.1f} metres
- Yaw: {yaw:.0f} degrees
- Altitude note: z increases going UP

Recent memory:
{memory}

User command: "{command}"

IMPORTANT EXAMPLES:
- "what do you see" → {{"steps":[{{"action":"describe"}}],"memory_update":"Described scene"}}
- "describe" → {{"steps":[{{"action":"describe"}}],"memory_update":"Described scene"}}
- "rotate left" → {{"steps":[{{"action":"rotate_left","value":90}}],"memory_update":"Rotated left"}}
- "move forward 2" → {{"steps":[{{"action":"move_forward","value":2}}],"memory_update":"Moved forward"}}
- "go down find red box" → {{"steps":[{{"action":"move_down","value":1.5}},{{"action":"search","target":"red box","direction":"all"}}],"memory_update":"Searched for red box"}}

Available atomic actions:
- rotate_left(degrees): turn left/CCW
- rotate_right(degrees): turn right/CW
- move_forward(metres): fly forward
- move_backward(metres): fly backward
- move_left(metres): strafe left
- move_right(metres): strafe right
- move_up(metres): increase altitude
- move_down(metres): decrease altitude
- search(target, direction): scan 360 looking for object. direction=left|right|forward|all
- describe(): describe what camera sees
- stop(): hover in place

Rules:
- NEVER combine rotation + movement in same step
- For compound commands decompose into ordered steps
- For "go down and find X" -> first move_down, then search
- For "turn left find X" -> first rotate_left, then search
- Extract ONLY the object name for search target (e.g. "red box" not "find a red box")
- search direction from user hint: "on the left"->left, "on the right"->right, else->all

Respond ONLY with valid JSON:
{{
  "steps": [
    {{"action": "move_down", "value": 1.5}},
    {{"action": "search", "target": "red box", "direction": "all"}}
  ],
  "memory_update": "<one line: what was done/found>"
}}"""

SCAN_PROMPT = """Drone camera image attached.
Looking for: "{target}"

Respond ONLY with JSON:
{{"found": true/false, "location": "left|center|right", "distance": "near|medium|far", "description": "<one line what you see>"}}"""

FRONTIER_PROMPT = """Drone exploration planner.
Camera view attached.
Frontier regions to explore:
{frontiers}
{goal_hint}
Pick the most promising frontier to explore next.
JSON only: {{"chosen_id": <id>, "reasoning": "<why>", "scene": "<what you see>"}}"""

class Mode(Enum):
    INTERACTIVE = "interactive"
    EXPLORE     = "explore"
    HYBRID      = "hybrid"

class VLMUnifiedNode(Node):
    def __init__(self):
        super().__init__("vlm_unified_node")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.drone_yaw = 0.0
        self.mode = Mode.INTERACTIVE
        self.is_busy = False
        self.stop_flag = False
        self.hybrid_goal = ""
        self.frontiers = []
        self.visited_frontiers = set()
        self.explore_active = False

        # Memory — last 8 events
        self.memory = []

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, "/drone/rgbd/image", self.image_cb, 10)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.px4_odom_cb, px4_qos)
        self.create_subscription(MarkerArray, "/uav/exploration_frontiers", self.frontier_cb, 10)
        self.create_subscription(Bool, "/uav/mission_complete", self.mission_cb, 10)

        self.teleop_pub = self.create_publisher(TrajectorySetpoint, "/teleop/setpoint", 10)
        self.waypoint_pub = self.create_publisher(PointStamped, "/uav/current_waypoint", 10)
        self.chat_pub = self.create_publisher(String, "/uav/drone_chat", 10)
        self.vlm_decision_pub = self.create_publisher(String, "/uav/vlm_decision", 10)

        self.create_timer(0.05, self.publish_hover)
        self.create_timer(5.0, self.explore_tick)

        self.print_banner()
        threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info(f"VLM Unified v2 ready — {API_MODEL}")

    def print_banner(self):
        print("\n" + "="*55)
        print(f" Drone Control v2 — {API_MODEL}")
        print("="*55)
        print(" Any natural language command works!")
        print(" Examples:")
        print("   go down and find a red box")
        print("   turn left then look for yellow cylinder")
        print("   what do you see")
        print("   explore")
        print("   explore find red box")
        print("   stop")
        print("="*55 + "\n")

    # ── Callbacks ─────────────────────────────────────────────────────

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def px4_odom_cb(self, msg):
        w,x,y,z = msg.q[0],msg.q[1],msg.q[2],msg.q[3]
        self.drone_yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

    def frontier_cb(self, msg):
        self.frontiers = [
            {"id":m.id, "x":round(m.pose.position.x,1),
             "y":round(m.pose.position.y,1),
             "z":round(m.pose.position.z,1),
             "size":round(m.scale.x,2)}
            for m in msg.markers
            if m.action==0 and m.id not in self.visited_frontiers
        ]

    def mission_cb(self, msg):
        if msg.data and self.mode in [Mode.EXPLORE, Mode.HYBRID]:
            self.is_busy = False

    # ── Memory ────────────────────────────────────────────────────────

    def add_memory(self, event):
        self.memory.append(f"[t={time.strftime('%H:%M:%S')}] {event}")
        if len(self.memory) > 8:
            self.memory = self.memory[-8:]

    def get_memory_str(self):
        if not self.memory:
            return "No recent events"
        return "\n".join(self.memory[-5:])

    # ── Publishers ────────────────────────────────────────────────────

    def publish_hover(self):
        if self.is_busy: return
        msg = TrajectorySetpoint()
        msg.position  = [float("nan")]*3
        msg.velocity  = [0.0,0.0,0.0]
        msg.yaw       = float(math.radians(self.drone_yaw))
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.teleop_pub.publish(msg)

    def hover_pub(self):
        msg = TrajectorySetpoint()
        msg.position  = [float("nan")]*3
        msg.velocity  = [0.0,0.0,0.0]
        msg.yaw       = float(math.radians(self.drone_yaw))
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.teleop_pub.publish(msg)

    # ── Movement ──────────────────────────────────────────────────────

    def wrap_pi(self, a):
        return (a+math.pi)%(2*math.pi)-math.pi

    def do_rotate(self, degrees):
        if self.stop_flag: return
        yaw_rate = -1.2 if degrees > 0 else 1.2  # faster rotation
        duration = abs(math.radians(degrees))/abs(yaw_rate)
        cur_yaw = math.radians(self.drone_yaw)
        start = time.time()
        while time.time()-start < duration:
            if self.stop_flag: break
            cur_yaw = self.wrap_pi(cur_yaw + yaw_rate*0.8*0.02)
            msg = TrajectorySetpoint()
            msg.position  = [float("nan")]*3
            msg.velocity  = [0.0,0.0,0.0]
            msg.yaw       = float(cur_yaw)
            msg.yawspeed  = float(yaw_rate*0.8)
            msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover_pub()
        time.sleep(0.3)

    def do_move(self, vx, vy, vz, dist):
        if self.stop_flag: return
        speed = 1.5
        dur = max(dist/speed, 0.5)
        yaw_rad = math.radians(self.drone_yaw)
        cy,sy = math.cos(yaw_rad),math.sin(yaw_rad)
        nx = cy*vx*speed - sy*vy*speed
        ny = sy*vx*speed + cy*vy*speed
        nz = vz*speed
        start = time.time()
        while time.time()-start < dur:
            if self.stop_flag: break
            msg = TrajectorySetpoint()
            msg.position  = [float("nan")]*3
            msg.velocity  = [float(nx),float(ny),float(nz)]
            msg.yaw       = float(math.radians(self.drone_yaw))
            msg.yawspeed  = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover_pub()
        time.sleep(0.3)

    # ── VLM API ───────────────────────────────────────────────────────

    def call_vlm(self, prompt, image=True, max_tokens=300):
        if image and self.latest_frame is None:
            return None
        try:
            if image:
                frame = cv2.resize(self.latest_frame,(640,480))
                _,buf = cv2.imencode(".jpg",frame,[cv2.IMWRITE_JPEG_QUALITY,85])
                img_b64 = base64.b64encode(buf).decode()
                content = [
                    {"type":"text","text":prompt},
                    {"type":"image_url","image_url":{
                        "url":f"data:image/jpeg;base64,{img_b64}",
                        "detail":"low"}}
                ]
            else:
                content = prompt

            r = requests.post(API_URL,
                headers={"Authorization":f"Bearer {API_KEY}",
                         "Content-Type":"application/json"},
                json={"model":API_MODEL,"max_tokens":max_tokens,
                      "stream":False,
                      "messages":[{"role":"user","content":content}]},
                timeout=60)

            if r.status_code != 200:
                return None
            res = r.json()
            if not res.get("choices"): return None
            return res["choices"][0]["message"].get("content","").strip()
        except Exception as e:
            print(f"API: {e}"); return None

    def clean_json(self, text):
        if not text: return None
        if "```" in text:
            text = text.split("```")[1]
            if text.startswith("json"): text=text[4:]
        try: return json.loads(text.strip())
        except: return None

    # ── Scan ─────────────────────────────────────────────────────────

    def do_scan(self, target):
        p = SCAN_PROMPT.format(target=target)
        r = self.call_vlm(p, max_tokens=100)
        d = self.clean_json(r)
        return d if d else {"found":False,"description":r or "no response"}

    # ── Search mission ────────────────────────────────────────────────

    def run_search(self, target, direction="all"):
        print(f"Searching for: {target} (direction: {direction})")
        self.add_memory(f"Started search for {target}")

        # Pre-rotate based on direction hint
        if direction == "left":
            print("  Rotating left to face search direction...")
            self.do_rotate(90)
        elif direction == "right":
            print("  Rotating right to face search direction...")
            self.do_rotate(-90)
        elif direction == "behind":
            print("  Rotating 180° to face behind...")
            self.do_rotate(180)

        found = False
        for attempt in range(2):
            if self.stop_flag: break
            if attempt == 1:
                print("  Moving to new position...")
                self.do_move(1,0,0,2.5)
                time.sleep(1)

            # Use fewer larger rotations — less drift, more reliable
            scans = 8
            rot_per_scan = 45  # degrees per step
            for i in range(scans):
                if self.stop_flag: break
                angle = i * rot_per_scan
                print(f"  Scan {angle}°...", end=" ", flush=True)
                self.do_rotate(rot_per_scan)
                time.sleep(2.0)  # wait longer for rotation to settle

                result = self.do_scan(target)
                desc = result.get("description","?")
                print(desc)

                if result.get("found", False):
                    loc  = result.get("location","center")
                    dist = result.get("distance","medium")
                    print(f"Found {target}! — {loc}, {dist}")
                    self.add_memory(f"Found {target} at yaw={self.drone_yaw:.0f}° — {loc}, {dist}")

                    # Align
                    if loc == "left":   self.do_rotate(25)
                    elif loc == "right": self.do_rotate(-25)

                    # Fly towards
                    md = {"near":1.0,"medium":2.5,"far":3.5}.get(dist,2.5)
                    print(f"Flying towards {target} ({md}m)...")
                    self.do_move(1,0,0,md)

                    # Verify
                    time.sleep(1)
                    v = self.do_scan(target)
                    print(f"Verify: {v.get('description','?')}")
                    if v.get("found",False):
                        print(f"Confirmed at destination!")
                        self.add_memory(f"Reached {target} at ({self.drone_x:.1f},{self.drone_y:.1f},{self.drone_z:.1f})")
                    else:
                        print(f"Close to {target}")
                    found = True
                    break
            if found: break

        if not found and not self.stop_flag:
            print(f"{target} not found after full search")
            self.add_memory(f"Could not find {target}")

    # ── Main command processor ────────────────────────────────────────

    def process_command(self, command):
        self.is_busy = True
        self.stop_flag = False

        prompt = MASTER_PROMPT.format(
            x=self.drone_x, y=self.drone_y, z=self.drone_z,
            yaw=self.drone_yaw,
            memory=self.get_memory_str(),
            command=command
        )

        print(f"  Planning...")
        content = self.call_vlm(prompt, max_tokens=400)
        data = self.clean_json(content)

        if not data or not data.get("steps"):
            print(f"  Could not parse: {content[:100] if content else 'no response'}")
            self.is_busy = False
            return

        steps = data.get("steps", [])
        mem_update = data.get("memory_update", "")

        print(f"  Plan: {len(steps)} steps:")
        for s in steps:
            print(f"    → {s}")

        for step in steps:
            if self.stop_flag: break
            action = step.get("action", "")
            value  = float(step.get("value", 1.0))
            target = step.get("target", "")
            direction = step.get("direction", "all")

            if action == "rotate_left":
                print(f"  Rotating left {value:.0f}°")
                self.do_rotate(abs(value))
            elif action == "rotate_right":
                print(f"  Rotating right {value:.0f}°")
                self.do_rotate(-abs(value))
            elif action == "move_forward":
                print(f"  Moving forward {value:.1f}m")
                self.do_move(1,0,0,value)
            elif action == "move_backward":
                print(f"  Moving backward {value:.1f}m")
                self.do_move(-1,0,0,value)
            elif action == "move_left":
                print(f"  Moving left {value:.1f}m")
                self.do_move(0,-1,0,value)
            elif action == "move_right":
                print(f"  Moving right {value:.1f}m")
                self.do_move(0,1,0,value)
            elif action == "move_up":
                print(f"  Moving up {value:.1f}m")
                self.do_move(0,0,-1,value)
            elif action == "move_down":
                print(f"  Moving down {value:.1f}m")
                self.do_move(0,0,1,value)
            elif action == "search":
                self.run_search(target, direction)
            elif action == "describe":
                result = self.call_vlm("Describe what the drone camera sees in 1-2 sentences.")
                print(f"\n{result}\n")
                self.add_memory(f"Described scene: {result[:60] if result else '?'}...")
            elif action == "stop":
                self.hover_pub()
                print("  Hovering")

        if mem_update:
            self.add_memory(mem_update)

        print("Done")
        self.is_busy = False

    # ── Exploration ───────────────────────────────────────────────────

    def explore_tick(self):
        if not self.explore_active or self.is_busy or not self.frontiers: return
        if self.mode == Mode.HYBRID and self.hybrid_goal:
            r = self.do_scan(self.hybrid_goal)
            if r.get("found", False):
                print(f"\nFound goal: {self.hybrid_goal}! Switching to interactive.")
                self.add_memory(f"Hybrid goal achieved: {self.hybrid_goal}")
                self.explore_active = False
                self.mode = Mode.INTERACTIVE
                self.hover_pub()
                return
        threading.Thread(target=self.pick_frontier, daemon=True).start()

    def pick_frontier(self):
        self.is_busy = True
        fd = "\n".join([
            f"  id={f['id']} pos=({f['x']},{f['y']},{f['z']}) size={f['size']}"
            for f in self.frontiers[:6]
        ])
        gh = f"Priority: frontiers towards {self.hybrid_goal}" if self.mode==Mode.HYBRID else ""
        p = FRONTIER_PROMPT.format(frontiers=fd, goal_hint=gh)
        content = self.call_vlm(p, max_tokens=150)
        data = self.clean_json(content)

        if not data:
            self.is_busy=False; return

        cid = data.get("chosen_id")
        reason = data.get("reasoning","?")
        scene = data.get("scene","?")
        print(f"\nExploring frontier {cid}: {reason}")
        print(f"  {scene}")
        self.add_memory(f"Exploring frontier {cid}: {reason}")

        msg = String()
        msg.data = json.dumps({
            "chosen_frontier":cid,"reasoning":reason,
            "scene":scene,"mode":self.mode.value
        })
        self.vlm_decision_pub.publish(msg)

        for f in self.frontiers:
            if f["id"]==cid:
                self.visited_frontiers.add(cid)
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = "map"
                pt.point.x = f["x"]
                pt.point.y = f["y"]
                pt.point.z = f["z"]
                self.waypoint_pub.publish(pt)
                break
        self.is_busy=False

    # ── Input loop ────────────────────────────────────────────────────

    def input_loop(self):
        while rclpy.ok():
            try:
                cmd = input("  ").strip()
                if not cmd: continue
                lower = cmd.lower()

                if lower == "quit": break

                if lower in ["stop","halt","stop exploring"]:
                    self.stop_flag = True
                    self.explore_active = False
                    self.mode = Mode.INTERACTIVE
                    self.hover_pub()
                    self.is_busy = False
                    print("Stopped")
                    continue

                if self.is_busy:
                    print("Busy — type stop to cancel")
                    continue

                self.stop_flag = False

                # Explore mode
                if lower.startswith("explore"):
                    rest = lower.replace("explore","").strip()
                    if rest:
                        self.hybrid_goal = rest
                        self.mode = Mode.HYBRID
                        print(f"Hybrid: exploring + finding: {rest}")
                        self.add_memory(f"Started hybrid exploration, goal: {rest}")
                    else:
                        self.mode = Mode.EXPLORE
                        self.hybrid_goal = ""
                        print("Autonomous exploration started")
                        self.add_memory("Started autonomous FBE+VLM exploration")
                    self.explore_active = True
                    self.visited_frontiers.clear()
                    continue

                # All other commands → LLM planning
                threading.Thread(
                    target=self.process_command,
                    args=(cmd,), daemon=True).start()

            except EOFError:
                break

def main():
    rclpy.init()
    node = VLMUnifiedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
