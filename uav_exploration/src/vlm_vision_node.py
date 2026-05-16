#!/usr/bin/env python3
"""
VLM Vision Node — Terminal A
Continuously captures camera and describes what drone sees.
Publishes descriptions for interactive node to use.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2, base64, requests, os, time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

OLLAMA_API_KEY = os.environ.get("OLLAMA_API_KEY", "")
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
API_KEY   = OLLAMA_API_KEY or OPENAI_API_KEY
API_URL   = "https://ollama.com/v1/chat/completions" if OLLAMA_API_KEY else "https://api.openai.com/v1/chat/completions"
API_MODEL = "gemma3:4b" if OLLAMA_API_KEY else "gpt-4o"

VISION_PROMPT = """You are the eyes of a UAV drone.
Describe what you see in the camera in 1-2 sentences.
Include: objects visible, their colors, approximate positions (left/center/right), distances.
Be specific and concise."""

class VLMVisionNode(Node):
    def __init__(self):
        super().__init__("vlm_vision_node")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.scan_interval = 3.0  # seconds between scans
        self.last_scan = 0.0
        self.scan_count = 0

        self.create_subscription(Image, "/drone/rgbd/image", self.image_cb, 10)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, 10)

        self.desc_pub = self.create_publisher(String, "/uav/vlm_description", 10)
        self.create_timer(self.scan_interval, self.scan_and_describe)

        print("\n" + "="*60)
        print(f" 👁  VLM Vision Node — {API_MODEL}")
        print("="*60)
        print(" Continuously describing drone camera feed...")
        print(" Publishing to: /uav/vlm_description")
        print("="*60 + "\n")

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z

    def scan_and_describe(self):
        if self.latest_frame is None:
            return
        self.scan_count += 1
        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode()

        try:
            r = requests.post(API_URL,
                headers={"Authorization": f"Bearer {API_KEY}",
                         "Content-Type": "application/json"},
                json={"model": API_MODEL, "max_tokens": 150,
                      "stream": False,
                      "messages": [{"role": "user", "content": [
                          {"type": "text", "text": VISION_PROMPT},
                          {"type": "image_url", "image_url": {
                              "url": f"data:image/jpeg;base64,{img_b64}",
                              "detail": "low"}}
                      ]}]},
                timeout=10)

            if r.status_code == 200:
                content = r.json()["choices"][0]["message"].get("content", "").strip()
                if content:
                    print(f"[{self.scan_count:04d}] pos=({self.drone_x:.1f},{self.drone_y:.1f},{self.drone_z:.1f})")
                    print(f"       {content}")
                    print()
                    msg = String()
                    msg.data = content
                    self.desc_pub.publish(msg)
        except Exception as e:
            print(f"[{self.scan_count:04d}] API error: {e}")

def main():
    rclpy.init()
    node = VLMVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
