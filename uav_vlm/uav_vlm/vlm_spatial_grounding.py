#!/usr/bin/env python3
"""
SPF-B: VLM Spatial Grounding Node
Subscribes to /user/instruction + camera + depth
Outputs /spf/target_pose (geometry_msgs/PoseStamped)
Uses depth from /drone/rgbd/depth for accurate 3D projection
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2, base64, json, requests, os
import numpy as np
import math, threading

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")
API_KEY   = OPENAI_API_KEY or OLLAMA_API_KEY
API_URL   = "https://api.openai.com/v1/chat/completions" if OPENAI_API_KEY else "https://ollama.com/v1/chat/completions"
API_MODEL = "gpt-4o" if OPENAI_API_KEY else "gemma4:31b"

# Camera intrinsics from SDF
# fx = fy = (IMG_W/2) / tan(HFOV/2)
IMG_W  = 1280
IMG_H  = 720
H_FOV  = 1.047  # rad
FX     = (IMG_W / 2.0) / math.tan(H_FOV / 2.0)
FY     = FX
CX     = IMG_W / 2.0
CY     = IMG_H / 2.0

GROUNDING_PROMPT = """You are a UAV drone vision system.
User instruction: "{instruction}"

Look at the camera image and find the target described.
Point to WHERE the drone should fly — click on the target object.

Image size: {w}x{h} pixels. Center: ({cx}, {cy}).

Rules:
- Point exactly AT the target if visible
- If not visible, point toward most likely direction
- depth_estimate: 1=very close(0.5m), 5=medium(3m), 10=far(8+m)
- yaw_change_deg: rotation needed BEFORE flying (positive=left, negative=right)

JSON only:
{{
  "target_uv": [u, v],
  "depth_estimate": <1-10>,
  "yaw_change_deg": <float>,
  "target_visible": true/false,
  "label": "<what you see at that point>",
  "confidence": <0.0-1.0>
}}"""

class VLMSpatialGrounding(Node):
    def __init__(self):
        super().__init__("vlm_spatial_grounding")
        self.bridge = CvBridge()
        self.latest_rgb   = None
        self.latest_depth = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.drone_yaw = 0.0
        self.is_processing = False

        # Subscribers — topic remaps from SDF names
        self.create_subscription(
            Image, "/drone/rgbd/image", self.rgb_cb, 10)
        self.create_subscription(
            Image, "/drone/rgbd/depth", self.depth_cb, 10)
        self.create_subscription(
            Odometry, "/drone/odom", self.odom_cb, 10)
        self.create_subscription(
            String, "/user/instruction", self.instruction_cb, 10)

        # Publishers
        self.pose_pub   = self.create_publisher(
            PoseStamped, "/spf/target_pose", 10)
        self.status_pub = self.create_publisher(
            String, "/uav/vlm_grounding_status", 10)

        self.get_logger().info(
            f"VLM Spatial Grounding ready — {API_MODEL}")
        self.get_logger().info(
            f"Camera: {IMG_W}x{IMG_H} fx={FX:.1f} fy={FY:.1f}")

    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        # Depth image — float32 in metres
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
        except Exception:
            self.latest_depth = None

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.drone_yaw = math.degrees(
            math.atan2(2*(q.w*q.z + q.x*q.y),
                       1 - 2*(q.y*q.y + q.z*q.z)))

    def instruction_cb(self, msg):
        if self.is_processing:
            self.get_logger().warn("Still processing previous instruction")
            return
        instruction = msg.data
        self.get_logger().info(f"Instruction: {instruction}")
        threading.Thread(
            target=self.process_instruction,
            args=(instruction,), daemon=True).start()

    def get_depth_at_pixel(self, u, v):
        """Get depth from depth image at pixel (u,v)."""
        if self.latest_depth is None:
            return None
        u_img = int(u * IMG_W / 640)
        v_img = int(v * IMG_H / 360)
        u_img = max(0, min(IMG_W-1, u_img))
        v_img = max(0, min(IMG_H-1, v_img))
        # Sample 5x5 patch and take median for robustness
        patch = self.latest_depth[
            max(0,v_img-2):v_img+3,
            max(0,u_img-2):u_img+3]
        valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    def project_to_3d(self, u, v, depth_m):
        """Pinhole camera model — pixel + depth → 3D world point."""
        # Scale from 640x360 to full resolution
        u_full = u * IMG_W / 640.0
        v_full = v * IMG_H / 360.0

        # Camera frame (x=right, y=down, z=forward)
        x_cam = (u_full - CX) * depth_m / FX
        y_cam = (v_full - CY) * depth_m / FY
        z_cam = depth_m

        # Camera → drone body frame
        # Camera faces forward: z_cam=forward, x_cam=right, y_cam=down
        x_body =  z_cam   # forward
        y_body = -x_cam   # left
        z_body = -y_cam   # up

        # Body → world frame (rotate by drone yaw)
        yaw_rad = math.radians(self.drone_yaw)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        world_x = self.drone_x + cos_y*x_body - sin_y*y_body
        world_y = self.drone_y + sin_y*x_body + cos_y*y_body
        world_z = self.drone_z + z_body

        return world_x, world_y, world_z

    def yaw_from_pixel(self, u, depth_m):
        """Compute required yaw change from pixel offset."""
        u_full = u * IMG_W / 640.0
        x_cam  = (u_full - CX) * depth_m / FX
        return math.degrees(math.atan2(x_cam, depth_m))

    def process_instruction(self, instruction):
        self.is_processing = True
        status = String()

        try:
            if self.latest_rgb is None:
                self.get_logger().error("No RGB image available")
                return

            # Encode image
            frame = cv2.resize(self.latest_rgb, (640, 360))
            _, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_b64 = base64.b64encode(buf).decode()

            prompt = GROUNDING_PROMPT.format(
                instruction=instruction,
                w=640, h=360, cx=320, cy=180)

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

            if r.status_code != 200:
                self.get_logger().error(f"API error: {r.status_code}")
                return

            content = r.json()["choices"][0]["message"].get(
                "content", "").strip()
            if "```" in content:
                content = content.split("```")[1]
                if content.startswith("json"):
                    content = content[4:]

            data = json.loads(content.strip())

            u = data["target_uv"][0]
            v = data["target_uv"][1]
            d_est = float(data.get("depth_estimate", 5))
            yaw_vlm = float(data.get("yaw_change_deg", 0))
            visible = data.get("target_visible", False)
            label   = data.get("label", "?")
            conf    = data.get("confidence", 0.5)

            pass  # suppressed
            self.get_logger().info(f"Label: {label} conf={conf:.2f}")

            # Get depth — from sensor first, fallback to VLM estimate
            depth_sensor = self.get_depth_at_pixel(u, v)
            if depth_sensor is not None and 0.3 < depth_sensor < 15.0:
                depth_m = depth_sensor
                pass  # suppressed
            else:
                # Adaptive scaling from SPF paper
                depth_m = max(0.3, 10.0 * ((d_est/10.0)**1.8))
                pass  # suppressed

            # Project to 3D world coordinates
            wx, wy, wz = self.project_to_3d(u, v, depth_m)

            # Clamp to room bounds
            wx = max(-8.0, min(8.0, wx))
            wy = max(-8.0, min(8.0, wy))
            wz = max(1.0,  min(3.5, wz))

            pass  # suppressed

            # Compute target yaw
            yaw_sensor = self.yaw_from_pixel(u, depth_m)
            target_yaw_deg = self.drone_yaw + yaw_sensor
            target_yaw_rad = math.radians(target_yaw_deg)

            # Build quaternion from yaw
            half_yaw = target_yaw_rad / 2.0

            # Publish target pose
            pose = PoseStamped()
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = wz
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            self.pose_pub.publish(pose)

            # Publish status
            status.data = json.dumps({
                "instruction": instruction,
                "target_uv": [u, v],
                "depth_m": depth_m,
                "target_world": [wx, wy, wz],
                "target_yaw_deg": target_yaw_deg,
                "visible": visible,
                "label": label,
                "confidence": conf
            })
            self.status_pub.publish(status)
            pass  # suppressed

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
        finally:
            self.is_processing = False

def main():
    rclpy.init()
    rclpy.spin(VLMSpatialGrounding())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
