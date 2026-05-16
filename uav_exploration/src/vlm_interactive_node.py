#!/usr/bin/env python3
"""
VLM Interactive Control Node — Terminal B
SPF-style control using vision descriptions from Terminal A.
User types commands, drone executes them.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from cv_bridge import CvBridge
import cv2, base64, json, requests, os
import threading, math, time, re
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")

# Use OpenAI for planning (reliable JSON), Ollama for fast vision scans
PLAN_KEY   = OPENAI_API_KEY or OLLAMA_API_KEY
PLAN_URL   = "https://api.openai.com/v1/chat/completions" if OPENAI_API_KEY else "https://ollama.com/v1/chat/completions"
PLAN_MODEL = "gpt-4o" if OPENAI_API_KEY else "gemma3:4b"

# Use Ollama for fast vision scans
SCAN_KEY   = OLLAMA_API_KEY or OPENAI_API_KEY
SCAN_URL   = "https://ollama.com/v1/chat/completions" if OLLAMA_API_KEY else "https://api.openai.com/v1/chat/completions"
SCAN_MODEL = "gemma3:4b" if OLLAMA_API_KEY else "gpt-4o"

API_KEY = PLAN_KEY
API_URL = PLAN_URL
API_MODEL = PLAN_MODEL

MASTER_PROMPT = """You are a UAV drone controller.

Drone state:
- Position: x={x:.1f}, y={y:.1f}, z={z:.1f}
- Yaw: {yaw:.0f} degrees
- Current scene: {scene}

Recent memory:
{memory}

User command: "{command}"

Decompose into ordered atomic steps:
- rotate_left(deg) / rotate_right(deg)
- move_forward(m) / move_backward(m)
- move_left(m) / move_right(m)  
- move_up(m) / move_down(m)
- search(target, direction) — scan for object
- describe() — describe scene
- stop()

Rules:
- NEVER combine rotation + movement
- Extract ONLY object name for search (e.g. "red box" not "find a red box")
- Use scene description to inform decisions
- For compound commands decompose all steps

JSON only:
{{"steps": [{{"action": "move_down", "value": 1.5}}, {{"action": "search", "target": "red box", "direction": "all"}}], "memory_update": "<what was done>"}}"""

SCAN_PROMPT = """Drone camera image.
Looking for: "{target}"
JSON only: {{"found": true/false, "location": "left|center|right", "distance": "near|medium|far", "description": "<one line>"}}"""

class VLMInteractiveNode(Node):
    def __init__(self):
        super().__init__("vlm_interactive_node")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.drone_yaw = 0.0
        self.is_busy = False
        self.stop_flag = False
        self.memory = []
        self.latest_scene = "No description yet"

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, "/drone/rgbd/image", self.image_cb, 10)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.px4_odom_cb, px4_qos)
        self.create_subscription(String, "/uav/vlm_description", self.desc_cb, 10)

        self.teleop_pub = self.create_publisher(TrajectorySetpoint, "/teleop/setpoint", 10)

        self.create_timer(0.05, self.publish_hover)

        self.print_banner()
        threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info(f"VLM Interactive ready — {API_MODEL}")

    def print_banner(self):
        print("\n" + "="*60)
        print(f" 🎮 VLM Interactive Control — {API_MODEL}")
        print("="*60)
        print(" Any natural language command works!")
        print(" Vision descriptions from Terminal A are used automatically.")
        print(" Type 'stop' to halt, 'quit' to exit")
        print("="*60 + "\n")

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def px4_odom_cb(self, msg):
        w,x,y,z = msg.q[0],msg.q[1],msg.q[2],msg.q[3]
        self.drone_yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

    def desc_cb(self, msg):
        self.latest_scene = msg.data

    def add_memory(self, event):
        self.memory.append(f"{time.strftime('%H:%M:%S')} {event}")
        if len(self.memory) > 8:
            self.memory = self.memory[-8:]

    def get_memory_str(self):
        return "\n".join(self.memory[-5:]) if self.memory else "None"

    # ── Movement ──────────────────────────────────────────────────────

    def wrap_pi(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def publish_hover(self):
        if self.is_busy: return
        msg = TrajectorySetpoint()
        msg.position  = [float("nan")] * 3
        msg.velocity  = [0.0, 0.0, 0.0]
        msg.yaw       = float(math.radians(self.drone_yaw))
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.teleop_pub.publish(msg)

    def hover(self):
        msg = TrajectorySetpoint()
        msg.position  = [float("nan")] * 3
        msg.velocity  = [0.0, 0.0, 0.0]
        msg.yaw       = float(math.radians(self.drone_yaw))
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.teleop_pub.publish(msg)

    def do_rotate(self, degrees):
        if self.stop_flag: return
        yr = -1.0 if degrees > 0 else 1.0
        yaw_speed = 0.8
        duration = abs(math.radians(degrees)) / yaw_speed
        cur_yaw = math.radians(self.drone_yaw)
        start = time.time()
        while time.time() - start < duration:
            if self.stop_flag: break
            cur_yaw = self.wrap_pi(cur_yaw + yr * yaw_speed * 0.02)
            msg = TrajectorySetpoint()
            msg.position  = [float("nan")] * 3
            msg.velocity  = [0.0, 0.0, 0.0]
            msg.yaw       = float(cur_yaw)
            msg.yawspeed  = float(yr * yaw_speed)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover()
        time.sleep(0.5)

    def do_move(self, vx, vy, vz, dist):
        if self.stop_flag: return
        speed = 2.0
        dur = max(dist / speed, 0.3)
        yaw_rad = math.radians(self.drone_yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        ned_vx = cy * vx * speed - sy * vy * speed
        ned_vy = sy * vx * speed + cy * vy * speed
        ned_vz = vz * speed
        start = time.time()
        while time.time() - start < dur:
            if self.stop_flag: break
            msg = TrajectorySetpoint()
            msg.position  = [float("nan")] * 3
            msg.velocity  = [float(ned_vx), float(ned_vy), float(ned_vz)]
            msg.yaw       = float(math.radians(self.drone_yaw))
            msg.yawspeed  = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover()
        time.sleep(0.3)

    def exec_step(self, step):
        a = step.get("action", "")
        v = float(step.get("value", 1.0))
        if   a == "rotate_left":    self.do_rotate(abs(v))
        elif a == "rotate_right":   self.do_rotate(-abs(v))
        elif a == "move_forward":   self.do_move(1, 0, 0, v)
        elif a == "move_backward":  self.do_move(-1, 0, 0, v)
        elif a == "move_left":      self.do_move(0, -1, 0, v)
        elif a == "move_right":     self.do_move(0, 1, 0, v)
        elif a == "move_up":        self.do_move(0, 0, -1, v)
        elif a == "move_down":      self.do_move(0, 0, 1, v)
        elif a == "stop":           self.hover()
        elif a == "search":
            target = step.get("target", "object")
            direction = step.get("direction", "all")
            self.run_search(target, direction)
        elif a == "describe":
            print(f"\n👁  {self.latest_scene}\n")

    # ── VLM ───────────────────────────────────────────────────────────

    def call_vlm(self, prompt, image=True, max_tokens=300):
        if image and self.latest_frame is None: return None
        try:
            if image:
                frame = cv2.resize(self.latest_frame, (640, 480))
                _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                img_b64 = base64.b64encode(buf).decode()
                content = [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{img_b64}",
                        "detail": "low"}}
                ]
            else:
                content = prompt
            r = requests.post(API_URL,
                headers={"Authorization": f"Bearer {API_KEY}",
                         "Content-Type": "application/json"},
                json={"model": API_MODEL, "max_tokens": max_tokens,
                      "stream": False,
                      "messages": [{"role": "user", "content": content}]},
                timeout=60)
            if r.status_code != 200: return None
            res = r.json()
            if not res.get("choices"): return None
            return res["choices"][0]["message"].get("content", "").strip()
        except Exception as e:
            print(f"API: {e}"); return None

    def clean_json(self, text):
        if not text: return None
        if "```" in text:
            text = text.split("```")[1]
            if text.startswith("json"): text = text[4:]
        try: return json.loads(text.strip())
        except: return None

    def do_scan(self, target):
        p = SCAN_PROMPT.format(target=target)
        if self.latest_frame is None:
            return {"found": False, "description": "no camera"}
        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode()
        try:
            r = requests.post(SCAN_URL,
                headers={"Authorization": f"Bearer {SCAN_KEY}",
                         "Content-Type": "application/json"},
                json={"model": SCAN_MODEL, "max_tokens": 100,
                      "stream": False,
                      "messages": [{"role": "user", "content": [
                          {"type": "text", "text": p},
                          {"type": "image_url", "image_url": {
                              "url": f"data:image/jpeg;base64,{img_b64}",
                              "detail": "low"}}
                      ]}]},
                timeout=15)
            if r.status_code == 200:
                content = r.json()["choices"][0]["message"].get("content","").strip()
                d = self.clean_json(content)
                return d if d else {"found": False, "description": content}
        except Exception as e:
            pass
        return {"found": False, "description": "scan error"}

    def run_search(self, target, direction="all"):
        print(f"  Searching for: {target}")
        self.add_memory(f"Searching for {target}")
        if direction == "left":    self.do_rotate(90)
        elif direction == "right": self.do_rotate(-90)
        elif direction == "behind":self.do_rotate(180)

        found = False
        for attempt in range(2):
            if self.stop_flag: break
            if attempt == 1:
                print("  Moving to new position...")
                self.do_move(1, 0, 0, 2.5)
                time.sleep(1)
            for i in range(8):
                if self.stop_flag: break
                print(f"  Scan {i*45}°...", end=" ", flush=True)
                self.do_rotate(45)
                time.sleep(2.0)
                result = self.do_scan(target)
                print(result.get("description", "?"))
                if result.get("found", False):
                    loc  = result.get("location", "center")
                    dist = result.get("distance", "medium")
                    print(f"  ✅ Found {target}! — {loc}, {dist}")
                    if loc == "left":   self.do_rotate(25)
                    elif loc == "right": self.do_rotate(-25)
                    md = {"near": 1.0, "medium": 2.5, "far": 3.5}.get(dist, 2.5)
                    print(f"  🚁 Flying towards {target}...")
                    self.do_move(1, 0, 0, md)
                    time.sleep(1)
                    v = self.do_scan(target)
                    print(f"  Verify: {v.get('description', '?')}")
                    if v.get("found", False):
                        print(f"  ✅ Confirmed!")
                        self.add_memory(f"Found and reached {target}")
                    found = True; break
            if found: break
        if not found and not self.stop_flag:
            print(f"  ❌ {target} not found")
            self.add_memory(f"Could not find {target}")

    def process_command(self, command):
        self.is_busy = True
        self.stop_flag = False
        print(f"  Planning...")

        prompt = MASTER_PROMPT.format(
            x=self.drone_x, y=self.drone_y, z=self.drone_z,
            yaw=self.drone_yaw,
            scene=self.latest_scene,
            memory=self.get_memory_str(),
            command=command
        )

        content = self.call_vlm(prompt, max_tokens=400)
        data = self.clean_json(content)

        if not data or not data.get("steps"):
            print(f"  ❌ Raw response: {content[:200] if content else 'None'}")
            self.is_busy = False
            return

        steps = data.get("steps", [])
        mem = data.get("memory_update", "")
        print(f"  Plan: {len(steps)} steps")
        for s in steps:
            print(f"    → {s}")

        for step in steps:
            if self.stop_flag: break
            self.exec_step(step)

        if mem: self.add_memory(mem)
        print("  ✅ Done")
        self.is_busy = False

    def input_loop(self):
        while rclpy.ok():
            try:
                cmd = input("🎮 ").strip()
                if not cmd: continue
                lower = cmd.lower()

                if lower == "quit": break
                if lower in ["stop", "halt"]:
                    self.stop_flag = True
                    self.hover()
                    self.is_busy = False
                    print("  ⛔ Stopped")
                    continue
                if self.is_busy:
                    print("  ⏳ Busy — type stop to cancel")
                    continue

                self.stop_flag = False
                threading.Thread(
                    target=self.process_command,
                    args=(cmd,), daemon=True).start()

            except EOFError:
                break

def main():
    rclpy.init()
    node = VLMInteractiveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
