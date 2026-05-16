#!/usr/bin/env python3
"""
See Point Fly (SPF) v2
Uses existing planner stack properly:
- Yaw: via /uav/cmd_vel yawspeed
- XY: via /uav/navigate_to_goal action -> A* -> mp_node -> setpoint_publisher
- Z: via /uav/cmd_vel vz
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import VehicleOdometry
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import cv2, base64, json, requests, os
import threading, math, time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")
API_KEY   = OPENAI_API_KEY or OLLAMA_API_KEY
API_URL   = "https://api.openai.com/v1/chat/completions" if OPENAI_API_KEY else "https://ollama.com/v1/chat/completions"
API_MODEL = "gpt-4o" if OPENAI_API_KEY else "gemma4:31b"

# Camera params from SDF
IMG_W  = 1280
IMG_H  = 720
H_FOV  = 1.047
V_FOV  = 2 * math.atan(math.tan(H_FOV/2) * IMG_H/IMG_W)
ALPHA  = H_FOV / 2
BETA   = V_FOV / 2

# Adaptive depth params (paper)
S_SCALE  = 10.0
L_LEVELS = 10
D_MIN    = 0.3
P_NONLIN = 1.8

SPF_PROMPT = """You are controlling a UAV drone.
Instruction: "{instruction}"

Look at the camera image. Identify the target or navigation direction.
Output a 2D waypoint in pixel coordinates pointing WHERE the drone should fly.

Image size: 640x360. Center: (320, 180).

Rules:
- Point AT the target object if visible
- If not visible, point toward likely direction (e.g. edge of image)
- depth 1=very close(0.5m), 5=medium(3m), 10=far(8m+)

JSON only:
{{
  "point": [u, v],
  "depth": <1-10>,
  "label": "<what you see and target location>",
  "task_complete": false,
  "target_visible": true/false
}}"""

DESCRIBE_PROMPT = "Describe what the UAV drone camera sees in 1-2 sentences. Include objects, colors, positions."

class SPFNode(Node):
    def __init__(self):
        super().__init__("spf_node")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.drone_yaw = 0.0
        self.is_busy = False
        self.stop_flag = False
        self.current_instruction = ""
        self.task_active = False
        self.iteration = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.create_subscription(Image, "/drone/rgbd/image", self.image_cb, 10)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self.px4_odom_cb, px4_qos)

        # Publishers
        # cmd_vel → setpoint_publisher (existing pipeline)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/uav/cmd_vel", 10)
        self.status_pub   = self.create_publisher(String, "/uav/spf_status", 10)
        self.phase_pub    = self.create_publisher(String, "/uav/mission_phase", 10)

        # Waypoint publisher → mp_node directly (no A*)
        self.waypoint_pub = self.create_publisher(PointStamped, "/uav/current_waypoint", 10)
        self.mission_pub  = self.create_publisher(Bool, "/uav/mission_complete", 10)

        # SPF loop at 1Hz
        self.create_timer(1.0, self.spf_loop)

        self.print_banner()
        threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info(f"SPF v2 ready — {API_MODEL}")

    def print_banner(self):
        print("\n" + "="*60)
        print(f" ✈️  See Point Fly (SPF) v2 — {API_MODEL}")
        print("="*60)
        print(" Uses existing planner stack:")
        print("   Yaw  → /uav/cmd_vel yawspeed")
        print("   XY   → /uav/current_waypoint → mp_node (no A*)")
        print("   Z    → /uav/cmd_vel vz")
        print()
        print(" Commands:")
        print("   fly to the yellow cylinder")
        print("   find the red box")
        print("   what do you see")
        print("   stop / quit")
        print("="*60 + "\n")

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

    # ── SPF Math ──────────────────────────────────────────────────────

    def adaptive_depth(self, d_vlm):
        return max(D_MIN, S_SCALE * ((d_vlm / L_LEVELS) ** P_NONLIN))

    def pixel_to_3d(self, u, v, d_adj):
        u_norm = (u - 320) / 320.0
        v_norm = (v - 180) / 180.0
        Sx = u_norm * d_adj * math.tan(ALPHA)
        Sy = d_adj
        Sz = -v_norm * d_adj * math.tan(BETA)
        return Sx, Sy, Sz

    def displacement_to_controls(self, Sx, Sy, Sz):
        delta_yaw      = math.atan2(Sx, Sy)
        delta_pitch    = math.sqrt(Sx**2 + Sy**2)
        delta_throttle = Sz
        return delta_yaw, delta_pitch, delta_throttle

    # ── Control execution using existing stack ────────────────────────

    def execute_yaw(self, delta_yaw_rad):
        """Send yaw via /uav/cmd_vel → setpoint_publisher."""
        if abs(delta_yaw_rad) < 0.05: return
        duration = abs(delta_yaw_rad) / 0.8
        yr = -1.0 if delta_yaw_rad > 0 else 1.0
        print(f"  → Yaw {math.degrees(delta_yaw_rad):.1f}° ({duration:.1f}s)")
        start = time.time()
        while time.time() - start < duration:
            if self.stop_flag: break
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.twist.linear.x  = 0.0
            msg.twist.linear.y  = 0.0
            msg.twist.linear.z  = 0.0
            msg.twist.angular.z = yr * 0.8  # yaw rate rad/s
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.02)
        # Stop yaw
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.3)

    def execute_xy(self, Sx, Sy):
        """Send XY waypoint directly to mp_node — no A* planner."""
        if math.sqrt(Sx**2 + Sy**2) < 0.2: return

        # Limit step size to 2m max per iteration
        step_limit = 3.0  # must be > acceptance_radius(1.0m)
        magnitude = math.sqrt(Sx**2 + Sy**2)
        if magnitude > step_limit:
            scale = step_limit / magnitude
            Sx *= scale
            Sy *= scale

        # Convert body frame displacement to world frame
        yaw_rad = math.radians(self.drone_yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        world_x = self.drone_x + cy*Sy - sy*Sx
        world_y = self.drone_y + sy*Sy + cy*Sx
        world_z = self.drone_z

        # Clamp to room bounds
        world_x = max(-8.0, min(8.0, world_x))
        world_y = max(-8.0, min(8.0, world_y))
        world_z = max(1.0, min(3.5, world_z))

        print(f"  → XY waypoint: ({world_x:.1f}, {world_y:.1f}, {world_z:.1f})")

        # Publish directly to waypoint manager → mp_node → cmd_vel → setpoint_publisher
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "map"
        pt.point.x = world_x
        pt.point.y = world_y
        pt.point.z = world_z

        # Publish at 20Hz for duration to let mp_node drive toward waypoint
        dist = math.sqrt(Sx**2 + Sy**2)
        duration = min(dist / 1.5, 3.0)  # max 3s per iteration
        start = time.time()
        # Reset mission complete flag so mp_node accepts new waypoint
        reset = Bool()
        reset.data = False
        self.mission_pub.publish(reset)
        time.sleep(0.1)

        print(f"  Driving to waypoint for {duration:.1f}s...")
        while time.time() - start < duration:
            if self.stop_flag: break
            pt.header.stamp = self.get_clock().now().to_msg()
            self.waypoint_pub.publish(pt)
            reset.data = False
            self.mission_pub.publish(reset)
            time.sleep(0.05)

    def execute_z(self, delta_z):
        """Send altitude change via /uav/cmd_vel."""
        if abs(delta_z) < 0.1: return
        duration = abs(delta_z) / 1.5
        vz = 1.5 if delta_z > 0 else -1.5  # positive=up in ENU
        print(f"  → Z {delta_z:.1f}m ({duration:.1f}s)")
        start = time.time()
        while time.time() - start < duration:
            if self.stop_flag: break
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.twist.linear.z = vz
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.02)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.3)

    # ── VLM ───────────────────────────────────────────────────────────

    def call_spf_vlm(self, instruction):
        if self.latest_frame is None: return None
        frame = cv2.resize(self.latest_frame, (640, 360))
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode()
        prompt = SPF_PROMPT.format(instruction=instruction)
        try:
            r = requests.post(API_URL,
                headers={"Authorization": f"Bearer {API_KEY}",
                         "Content-Type": "application/json"},
                json={"model": API_MODEL, "max_tokens": 200,
                      "stream": False,
                      "messages": [{"role": "user", "content": [
                          {"type": "text", "text": prompt},
                          {"type": "image_url", "image_url": {
                              "url": f"data:image/jpeg;base64,{img_b64}",
                              "detail": "low"}}
                      ]}]},
                timeout=30)
            if r.status_code != 200: return None
            content = r.json()["choices"][0]["message"].get("content","").strip()
            if "```" in content:
                content = content.split("```")[1]
                if content.startswith("json"): content = content[4:]
            return json.loads(content.strip())
        except Exception as e:
            print(f"  VLM error: {e}"); return None

    def call_describe_vlm(self):
        if self.latest_frame is None: return "No camera feed"
        frame = cv2.resize(self.latest_frame, (640, 360))
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode()
        try:
            r = requests.post(API_URL,
                headers={"Authorization": f"Bearer {API_KEY}",
                         "Content-Type": "application/json"},
                json={"model": API_MODEL, "max_tokens": 150,
                      "stream": False,
                      "messages": [{"role": "user", "content": [
                          {"type": "text", "text": DESCRIBE_PROMPT},
                          {"type": "image_url", "image_url": {
                              "url": f"data:image/jpeg;base64,{img_b64}",
                              "detail": "low"}}
                      ]}]},
                timeout=20)
            if r.status_code == 200:
                return r.json()["choices"][0]["message"].get("content","").strip()
        except: pass
        return "Could not describe"

    # ── SPF Loop ──────────────────────────────────────────────────────

    def spf_loop(self):
        if not self.task_active or self.is_busy or self.stop_flag: return
        self.is_busy = True
        self.iteration += 1

        print(f"\n[SPF {self.iteration}] pos=({self.drone_x:.1f},{self.drone_y:.1f},{self.drone_z:.1f}) yaw={self.drone_yaw:.0f}°")

        data = self.call_spf_vlm(self.current_instruction)
        if not data:
            print("  No VLM response")
            self.is_busy = False
            return

        u        = data.get("point", [320,180])[0]
        v        = data.get("point", [320,180])[1]
        d_vlm    = float(data.get("depth", 5))
        label    = data.get("label", "?")
        complete = data.get("task_complete", False)
        visible  = data.get("target_visible", False)

        print(f"  VLM: point=({u},{v}) depth={d_vlm} visible={visible}")
        print(f"  {label}")

        # Publish status
        msg = String()
        msg.data = json.dumps({
            "iter": self.iteration,
            "point": [u,v], "depth": d_vlm,
            "label": label, "complete": complete
        })
        self.status_pub.publish(msg)

        if complete:
            print(f"  ✅ Task complete!")
            self.task_active = False
            self.is_busy = False
            return

        # SPF pipeline
        d_adj = self.adaptive_depth(d_vlm)
        Sx, Sy, Sz = self.pixel_to_3d(u, v, d_adj)
        delta_yaw, delta_pitch, delta_throttle = self.displacement_to_controls(Sx, Sy, Sz)

        print(f"  3D: Sx={Sx:.2f} Sy={Sy:.2f} Sz={Sz:.2f}")
        print(f"  Controls: yaw={math.degrees(delta_yaw):.1f}° xy={delta_pitch:.1f}m z={delta_throttle:.1f}m")

        if not self.stop_flag:
            # 1. Yaw via cmd_vel
            self.execute_yaw(delta_yaw)
            # 2. XY via navigate_to_goal → A* → mp_node
            self.execute_xy(Sx, Sy)
            # 3. Z via cmd_vel
            self.execute_z(delta_throttle)

        self.is_busy = False

    # ── Input ─────────────────────────────────────────────────────────

    def input_loop(self):
        while rclpy.ok():
            try:
                cmd = input("✈️  ").strip()
                if not cmd: continue
                lower = cmd.lower()
                if lower == "quit": break
                if lower in ["stop","halt"]:
                    self.stop_flag = True
                    self.task_active = False
                    self.is_busy = False
                    print("  ⛔ Stopped")
                    self.stop_flag = False
                    continue
                if any(k in lower for k in ["what do you see","describe","look"]):
                    threading.Thread(
                        target=lambda: print(f"\n  👁  {self.call_describe_vlm()}\n"),
                        daemon=True).start()
                    continue
                if self.is_busy:
                    print("  Busy — type stop to cancel")
                    continue
                self.current_instruction = cmd
                self.task_active = True
                self.stop_flag = False
                self.iteration = 0
                print(f"  🚀 SPF: {cmd}")
            except EOFError:
                break

def main():
    rclpy.init()
    node = SPFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
