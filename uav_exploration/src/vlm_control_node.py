#!/usr/bin/env python3
"""
VLM Drone Control — Smart Search + Sequential Control
- Simple commands handled locally (no API)
- Complex commands translated to action sequences by GPT
- Search missions: rotate, scan, find, fly to object
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from cv_bridge import CvBridge
import cv2, base64, json, requests, os
import threading, math, time, re
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")

# ── System prompt for command translation ─────────────────────────────
TRANSLATE_PROMPT = """You are a UAV drone command translator.
Convert natural language to a sequence of atomic drone actions.

Atomic actions available:
- rotate_left(degrees): rotate CCW
- rotate_right(degrees): rotate CW  
- move_forward(metres): move forward
- move_backward(metres): move backward
- move_left(metres): strafe left
- move_right(metres): strafe right
- move_up(metres): increase altitude
- move_down(metres): decrease altitude
- scan(): take photo and check for objects — use in search loops
- stop(): hover in place

Rules:
- NEVER combine movement + rotation in same step
- For search missions: use rotate + scan pattern
- For "find X": generate rotate_left(45) + scan() repeated 8 times (full 360)
- For "fly to X": first scan to find it, then move_forward

Respond ONLY with JSON:
{
  "steps": [
    {"action": "rotate_left", "value": 45},
    {"action": "scan", "target": "red box"},
    {"action": "rotate_left", "value": 45},
    {"action": "scan", "target": "red box"}
  ],
  "is_search": true,
  "target": "red box"
}"""

# ── System prompt for scene description ──────────────────────────────
DESCRIBE_PROMPT = """You are analyzing a drone camera feed.
Describe what you see clearly and concisely.
If asked to find a specific object, say if you can see it and where (left/center/right, near/far)."""

# ── System prompt for scan check ─────────────────────────────────────
SCAN_PROMPT = """Look at this drone camera image.
Target object: "{target}"
Can you see the {target} in this image?

Respond ONLY with JSON:
{{
  "found": true/false,
  "location": "left|center|right|not visible",
  "distance": "near(1-2m)|medium(3-4m)|far(5m+)|unknown",
  "description": "<one line of what you see>"
}}"""

class VLMDroneControl(Node):
    def __init__(self):
        super().__init__('vlm_drone_control')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.drone_yaw = 0.0
        self.is_busy = False
        self.stop_flag = False
        self.conversation_history = []

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(Image, '/drone/rgbd/image', self.image_cb, 10)
        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, 10)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.px4_odom_cb, px4_qos)

        self.teleop_pub = self.create_publisher(
            TrajectorySetpoint, '/teleop/setpoint', 10)

        print("\n" + "="*50)
        print(" 🚁 VLM Drone Control")
        print("="*50)
        print(" Try: 'find the red box'")
        print("      'what do you see?'")
        print("      'rotate left'")
        print("      'move forward 2'")
        print("      'stop'  'quit'")
        print("="*50 + "\n")

        # Continuous 20Hz hover publisher — keeps offboard controller happy
        self.create_timer(0.05, self.publish_hover)

        self.input_thread = threading.Thread(
            target=self.input_loop, daemon=True)
        self.input_thread.start()

    # ── Continuous hover publisher ───────────────────────────────────────

    def publish_hover(self):
        """Publish hover setpoint at 20Hz — keeps offboard controller active."""
        if self.is_busy:
            return  # movement commands handle publishing when busy
        msg = TrajectorySetpoint()
        msg.position  = [float('nan')] * 3
        msg.velocity  = [0.0, 0.0, 0.0]
        msg.yaw       = float(self.drone_yaw)
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.teleop_pub.publish(msg)

    # ── Sensor callbacks ──────────────────────────────────────────────────

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def px4_odom_cb(self, msg):
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        self.drone_yaw = math.atan2(siny, cosy)

    def wrap_pi(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ── Movement primitives ───────────────────────────────────────────

    def do_rotate(self, degrees):
        if self.stop_flag:
            return
        yaw_rate = -0.8 if degrees > 0 else 0.8  # confirmed: negative yawspeed = LEFT
        duration = abs(math.radians(degrees)) / abs(yaw_rate)
        current_yaw = self.drone_yaw
        start = time.time()
        while time.time() - start < duration:
            if self.stop_flag:
                break
            current_yaw = self.wrap_pi(current_yaw + yaw_rate * 0.8 * 0.02)
            msg = TrajectorySetpoint()
            msg.position  = [float('nan')] * 3
            msg.velocity  = [0.0, 0.0, 0.0]
            msg.yaw       = float(current_yaw)
            msg.yawspeed  = float(yaw_rate * 0.8)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover()
        time.sleep(0.3)

    def do_move(self, vx, vy, vz, distance):
        if self.stop_flag:
            return
        speed = 1.5
        duration = distance / speed
        cy = math.cos(self.drone_yaw)
        sy = math.sin(self.drone_yaw)
        ned_vx = cy * vx * speed - sy * vy * speed
        ned_vy = sy * vx * speed + cy * vy * speed
        ned_vz = vz * speed  # NED: positive=down, negative=up
        start = time.time()
        while time.time() - start < duration:
            if self.stop_flag:
                break
            msg = TrajectorySetpoint()
            msg.position  = [float('nan')] * 3
            msg.velocity  = [float(ned_vx), float(ned_vy), float(ned_vz)]
            msg.yaw       = float(self.drone_yaw)
            msg.yawspeed  = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.teleop_pub.publish(msg)
            time.sleep(0.02)
        self.hover()
        time.sleep(0.3)

    def hover(self):
        msg = TrajectorySetpoint()
        msg.position  = [float('nan')] * 3
        msg.velocity  = [0.0, 0.0, 0.0]
        msg.yaw       = float(self.drone_yaw)
        msg.yawspeed  = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.teleop_pub.publish(msg)

    # ── Scan — take photo and check for target ────────────────────────

    def do_scan(self, target):
        """Take snapshot and ask GPT if target is visible. Returns dict."""
        if self.latest_frame is None:
            return {"found": False, "description": "no camera feed"}

        time.sleep(0.5)  # let camera settle
        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode('utf-8')

        prompt = SCAN_PROMPT.format(target=target)
        try:
            response = requests.post(
                "https://ollama.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {OLLAMA_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "gemma4:31b",
                    "max_tokens": 150,
                    "messages": [{
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}",
                                "detail": "low"
                            }}
                        ]
                    }]
                },
                timeout=60
            )
            if response.status_code != 200:
                return {"found": False, "description": "API error"}

            result = response.json()
            if not result.get('choices'):
                return {"found": False, "description": "empty response"}
            content = result['choices'][0]['message'].get('content', '')
            if not content:
                return {"found": False, "description": "no content"}
            content = content.strip()
            if '```' in content:
                content = content.split('```')[1]
                if content.startswith('json'):
                    content = content[4:]
            return json.loads(content)
        except Exception as e:
            return {"found": False, "description": str(e)}

    # ── Execute one step ──────────────────────────────────────────────

    def execute_step(self, step, target=""):
        """Execute one atomic step. Returns scan result if scan action."""
        action = step.get('action', '')
        value  = float(step.get('value', 1.0))
        t      = step.get('target', target)

        if action == 'rotate_left':
            self.do_rotate(abs(value))   # positive = LEFT confirmed
        elif action == 'rotate_right':
            self.do_rotate(-abs(value))  # negative = RIGHT confirmed
        elif action == 'move_forward':
            self.do_move(1.0, 0.0, 0.0, value)
        elif action == 'move_backward':
            self.do_move(-1.0, 0.0, 0.0, value)
        elif action == 'move_left':
            self.do_move(0.0, -1.0, 0.0, value)
        elif action == 'move_right':
            self.do_move(0.0, 1.0, 0.0, value)
        elif action == 'move_up':
            self.do_move(0.0, 0.0, -1.0, value)  # NED up = -Z
        elif action == 'move_down':
            self.do_move(0.0, 0.0, 1.0, value)  # NED down = +Z
        elif action == 'scan':
            return self.do_scan(t)
        elif action == 'stop':
            self.hover()
        return None

    # ── Simple local command execution ───────────────────────────────

    def execute_local(self, steps):
        self.is_busy = True
        self.stop_flag = False
        for step in steps:
            if self.stop_flag:
                break
            self.execute_step(step)
        print("✅ Done")
        print("🎮 ", end='', flush=True)
        self.is_busy = False

    # ── VLM command translation ───────────────────────────────────────

    def translate_command(self, command):
        """Ask GPT to translate command to action sequence."""
        if self.latest_frame is None:
            print("⚠️  No camera feed")
            self.is_busy = False
            return

        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode('utf-8')

        try:
            response = requests.post(
                "https://ollama.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {OLLAMA_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "gemma4:31b",
                    "max_tokens": 400,
                    "messages": [
                        {"role": "system", "content": TRANSLATE_PROMPT},
                        {"role": "user", "content": [
                            {"type": "text",
                             "text": f'Command: "{command}"\nDrone at x={self.drone_x:.1f}, y={self.drone_y:.1f}, z={self.drone_z:.1f}, yaw={math.degrees(self.drone_yaw):.0f}°\nTranslate to action steps.'},
                            {"type": "image_url", "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}",
                                "detail": "low"
                            }}
                        ]}
                    ]
                },
                timeout=60
            )

            if response.status_code != 200:
                print(f"❌ API error: {response.status_code}")
                self.is_busy = False
                return

            result = response.json()
            if not result.get('choices'):
                print("❌ Empty API response")
                self.is_busy = False
                return
            
            content = result['choices'][0]['message'].get('content', '')
            if not content:
                print("❌ Empty content from API")
                self.is_busy = False
                return
                
            content = content.strip()
            if '```' in content:
                content = content.split('```')[1]
                if content.startswith('json'):
                    content = content[4:]

            data = json.loads(content)
            steps = data.get('steps', [])
            is_search = data.get('is_search', False)
            target = data.get('target', '')

            # Debug — show what GPT planned
            print(f"   📋 Plan: {len(steps)} steps, search={is_search}, target='{target}'")
            for s in steps:
                print(f"      → {s}")

            if is_search and target:
                self.run_search_mission(steps, target)
            elif not is_search and not steps:
                print("❌ No steps generated")
                self.is_busy = False
            else:
                self.is_busy = True
                self.stop_flag = False
                for step in steps:
                    if self.stop_flag:
                        break
                    self.execute_step(step, target)
                print("✅ Done")
                print("🎮 ", end='', flush=True)
                self.is_busy = False

        except Exception as e:
            print(f"❌ {e}")
            self.is_busy = False

    # ── Search mission ────────────────────────────────────────────────

    def run_search_mission(self, steps, target):
        """Full 360° search — rotate 45° x 8, scan at each angle."""
        self.is_busy = True
        self.stop_flag = False
        print(f"🔍 Searching for: {target} — full 360° scan")

        found = False
        # 16 x 45° = covers 360° accounting for ~50% drift
        for i in range(16):
            if self.stop_flag:
                print("⛔ Stopped")
                break

            angle = i * 22
            print(f"   🔄 Scanning at ~{angle}°...")

            # Rotate 45° (accounts for ~50% drift)
            self.do_rotate(45)
            time.sleep(1.0)  # let camera settle after rotation

            # Scan
            result = self.do_scan(target)
            desc = result.get('description', '')
            print(f"   👁  {desc}")

            if result.get('found', False):
                loc = result.get('location', 'center')
                dist = result.get('distance', 'medium')
                print(f"✅ Found {target}! Location: {loc}, Distance: {dist}")

                # Align direction
                if loc == 'left':
                    print("   ↺ Adjusting left...")
                    self.do_rotate(25)
                elif loc == 'right':
                    print("   ↻ Adjusting right...")
                    self.do_rotate(-25)

                # Move towards object
                if 'near' in dist:
                    move_dist = 1.0
                elif 'medium' in dist:
                    move_dist = 2.5
                else:
                    move_dist = 3.5

                print(f"🚁 Flying towards {target} ({move_dist:.0f}m)...")
                self.do_move(1.0, 0.0, 0.0, move_dist)

                # Final verification scan after flying
                print(f"🔍 Verifying {target} is in front...")
                time.sleep(1.0)
                verify = self.do_scan(target)
                print(f"   👁  {verify.get('description', '')}")

                if verify.get('found', False):
                    vloc = verify.get('location', 'center')
                    # Fine-tune alignment
                    if vloc == 'left':
                        print("   ↺ Fine-tuning left...")
                        self.do_rotate(15)
                        self.do_move(1.0, 0.0, 0.0, 0.5)
                    elif vloc == 'right':
                        print("   ↻ Fine-tuning right...")
                        self.do_rotate(-15)
                        self.do_move(1.0, 0.0, 0.0, 0.5)
                    print(f"✅ Confirmed — {target} is directly ahead!")
                else:
                    print(f"⚠️  {target} not in front after flying — scanning again...")
                    # Quick 360 rescan from new position
                    for j in range(16):
                        if self.stop_flag: break
                        self.do_rotate(45)
                        time.sleep(0.8)
                        r2 = self.do_scan(target)
                        print(f"   👁  {r2.get('description', '')}")
                        if r2.get('found', False):
                            loc2 = r2.get('location', 'center')
                            if loc2 == 'left': self.do_rotate(20)
                            elif loc2 == 'right': self.do_rotate(-20)
                            self.do_move(1.0, 0.0, 0.0, 1.5)
                            print(f"✅ Found and reached {target}!")
                            break
                    else:
                        print(f"⚠️  Could not reconfirm {target}")

                found = True
                break

        if not found and not self.stop_flag:
            # Try moving to a new position and searching again
            print(f"   🔄 Not found — moving to new position to search...")
            self.do_move(1.0, 0.0, 0.0, 2.0)
            time.sleep(1.0)
            
            for i in range(16):
                if self.stop_flag:
                    break
                print(f"   🔄 Second scan at ~{i*22}°...")
                self.do_rotate(45)
                time.sleep(1.0)
                result = self.do_scan(target)
                print(f"   👁  {result.get('description', '')}")
                if result.get('found', False):
                    loc = result.get('location', 'center')
                    dist = result.get('distance', 'medium')
                    print(f"✅ Found {target}! Location: {loc}")
                    if loc == 'left': self.do_rotate(25)
                    elif loc == 'right': self.do_rotate(-25)
                    move_dist = 2.5 if 'medium' in dist else 1.5
                    print(f"🚁 Flying towards {target}...")
                    self.do_move(1.0, 0.0, 0.0, move_dist)

                    # Final verification
                    print(f"🔍 Verifying {target} is in front...")
                    time.sleep(1.0)
                    verify = self.do_scan(target)
                    print(f"   👁  {verify.get('description', '')}")
                    if verify.get('found', False):
                        print(f"✅ Confirmed — {target} is right here!")
                    else:
                        print(f"⚠️  Could not reconfirm — close enough!")
                    found = True
                    break

            if not found and not self.stop_flag:
                print(f"❌ {target} not found")

        print("🎮 ", end='', flush=True)
        self.is_busy = False

    # ── Describe ──────────────────────────────────────────────────────

    def describe_scene(self, question=""):
        if self.latest_frame is None:
            print("⚠️  No camera feed")
            self.is_busy = False
            return

        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode('utf-8')

        prompt = question if question else "Describe what the drone camera sees."

        try:
            response = requests.post(
                "https://ollama.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {OLLAMA_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "gemma4:31b",
                    "max_tokens": 200,
                    "messages": [
                        {"role": "system", "content": DESCRIBE_PROMPT},
                        {"role": "user", "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}",
                                "detail": "low"
                            }}
                        ]}
                    ]
                },
                timeout=60
            )
            content = response.json()['choices'][0]['message']['content'].strip()
            print(f"\n{content}\n")
        except Exception as e:
            print(f"❌ {e}")

        print("🎮 ", end='', flush=True)
        self.is_busy = False

    # ── Input loop ────────────────────────────────────────────────────

    def ask_clarification(self, question, options=None):
        """Ask user a clarifying question and get answer."""
        print(f"\n❓ {question}")
        if options:
            for i, opt in enumerate(options, 1):
                print(f"   {i}. {opt}")
            print("   (type number or your own answer)")
        answer = input("   → ").strip().lower()
        if options and answer.isdigit():
            idx = int(answer) - 1
            if 0 <= idx < len(options):
                return options[idx].lower()
        return answer

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("🎮 ").strip()
                if not user_input:
                    continue
                lower = user_input.lower()

                if lower == 'quit':
                    break

                if lower == 'stop':
                    self.stop_flag = True
                    self.hover()
                    print("⛔ Stopped")
                    self.is_busy = False
                    continue

                if self.is_busy:
                    print("⏳ busy — type 'stop' to cancel")
                    continue

                self.stop_flag = False

                # ── Handle simple commands locally ──
                if lower in ['rotate left', 'turn left']:
                    t = threading.Thread(target=self.execute_local,
                        args=([{'action':'rotate_left','value':90}],), daemon=True)
                    t.start()
                elif lower in ['rotate right', 'turn right']:
                    t = threading.Thread(target=self.execute_local,
                        args=([{'action':'rotate_right','value':90}],), daemon=True)
                    t.start()
                elif lower in ['turn around', 'rotate 180']:
                    t = threading.Thread(target=self.execute_local,
                        args=([{'action':'rotate_left','value':180}],), daemon=True)
                    t.start()

                # Rotate N degrees
                elif re.match(r'rotate (left )?(\d+)', lower):
                    m = re.search(r'(\d+)', lower)
                    deg = float(m.group(1)) if m else 90
                    direc = 'rotate_right' if 'right' in lower else 'rotate_left'
                    t = threading.Thread(target=self.execute_local,
                        args=([{'action':direc,'value':deg}],), daemon=True)
                    t.start()

                # Move commands
                elif False:  # replaced above
                    dist = 1.5
                    if any(k in lower for k in ['forward','front']):
                        act = 'move_forward'
                    elif any(k in lower for k in ['back','backward']):
                        act = 'move_backward'
                    elif any(k in lower for k in ['strafe left','move left','go left']):
                        act = 'move_left'
                    elif any(k in lower for k in ['strafe right','move right','go right']):
                        act = 'move_right'
                    elif any(k in lower for k in ['up','ascend','rise','increase height']):
                        act = 'move_up'
                    else:
                        act = 'move_down'
                    t = threading.Thread(target=self.execute_local,
                        args=([{'action':act,'value':dist}],), daemon=True)
                    t.start()

                # Describe / what do you see
                elif any(k in lower for k in [
                        'what do you see', 'describe', 'what is',
                        'what can you', 'look around', 'tell me what']):
                    self.is_busy = True
                    t = threading.Thread(
                        target=self.describe_scene,
                        args=(user_input,), daemon=True)
                    t.start()

                # Search/find commands — go straight to search mission
                elif any(k in lower for k in [
                        'find', 'look for', 'search for',
                        'locate', 'where is', 'can you find']):
                    # Clean target extraction
                    import re as _re
                    target = lower
                    # Remove all filler phrases longest first
                    fillers = sorted([
                        'can you rotate on the left to find',
                        'can you rotate on the right to find',
                        'can you try to find', 'can you find',
                        'can you look for', 'can you locate',
                        'can you try to', 'rotate left to find',
                        'rotate right to find', 'rotating left find',
                        'find the', 'find a', 'find an',
                        'look for the', 'look for a', 'look for an',
                        'search for the', 'search for a', 'search for',
                        'locate the', 'locate a', 'locate',
                        'where is the', 'where is a', 'where is',
                        'for me', 'please', 'can you',
                        'find', 'look for'
                    ], key=len, reverse=True)
                    for f in fillers:
                        target = target.replace(f, ' ').strip()
                    # Remove directional hints from target
                    for d in ['on the left', 'on the right', 'to the left',
                              'to the right', 'in front', 'behind',
                              'left side', 'right side']:
                        target = target.replace(d, '').strip()
                    target = ' '.join(target.split()).strip('? .,')
                        # Ask clarifying questions if direction not clear
                    pre_rotate = 0
                    if any(k in lower for k in ['on the left', 'to the left', 'left side']):
                        pre_rotate = 90
                        print(f"   ↺ Will rotate left 90° first")
                    elif any(k in lower for k in ['on the right', 'to the right', 'right side']):
                        pre_rotate = -90
                        print(f"   ↻ Will rotate right 90° first")
                    elif 'behind' in lower or 'back' in lower:
                        pre_rotate = 180
                        print(f"   ↺ Will rotate 180° first")
                    else:
                        # Ask direction
                        dir_ans = self.ask_clarification(
                            f"Which direction should I search for {target}?",
                            ["Search full 360°", "Left", "Right", "Forward", "Behind"]
                        )
                        if 'left' in dir_ans:
                            pre_rotate = 90
                        elif 'right' in dir_ans:
                            pre_rotate = -90
                        elif 'behind' in dir_ans:
                            pre_rotate = 180
                        else:
                            pre_rotate = 0  # full 360

                    self.is_busy = True
                    def search_with_rotate(t, r):
                        if r != 0:
                            print(f"   ↺ Rotating {r}° towards search direction...")
                            self.do_rotate(r)
                        self.run_search_mission([], t)
                    t = threading.Thread(
                        target=search_with_rotate,
                        args=(target, pre_rotate), daemon=True)
                    t.start()

                # Everything else → translate via GPT
                else:
                    self.is_busy = True
                    t = threading.Thread(
                        target=self.translate_command,
                        args=(user_input,), daemon=True)
                    t.start()

            except EOFError:
                break

def main():
    rclpy.init()
    node = VLMDroneControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
