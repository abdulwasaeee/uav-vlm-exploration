#!/usr/bin/env python3
"""
VLM Frontier Selector
At each decision point, captures camera image and asks GPT-4o
which frontier to explore next and why.
Publishes decision + reasoning to /uav/vlm_decision
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import cv2, base64, json, requests, os

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")

class VLMFrontierSelector(Node):
    def __init__(self):
        super().__init__('vlm_frontier_selector')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frontiers = []
        self.last_decision_time = 0

        # Subscribers
        self.create_subscription(Image, '/drone/rgbd/image', self.image_cb, 10)
        self.create_subscription(MarkerArray, '/uav/exploration_frontiers', self.frontier_cb, 10)
        self.create_subscription(String, '/uav/exploration_status', self.status_cb, 10)

        # Publishers
        self.decision_pub = self.create_publisher(String, '/uav/vlm_decision', 10)
        self.status_pub = self.create_publisher(String, '/uav/vlm_status', 10)

        self.get_logger().info('VLM Frontier Selector ready!')

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def frontier_cb(self, msg):
        # Extract frontier positions from markers
        frontiers = []
        for marker in msg.markers:
            if marker.action == 0:  # ADD
                frontiers.append({
                    'id': marker.id,
                    'x': round(marker.pose.position.x, 2),
                    'y': round(marker.pose.position.y, 2),
                    'z': round(marker.pose.position.z, 2),
                    'size': round(marker.scale.x, 2)
                })
        self.latest_frontiers = frontiers

        # Call VLM if we have multiple frontiers and a camera frame
        now = self.get_clock().now().nanoseconds / 1e9
        if len(frontiers) > 1 and self.latest_frame is not None:
            if now - self.last_decision_time > 10.0:  # max once per 10s
                self.last_decision_time = now
                self.ask_vlm(frontiers)

    def status_cb(self, msg):
        self.get_logger().info(f'Exploration status: {msg.data}')

    def ask_vlm(self, frontiers):
        self.get_logger().info(f'Asking VLM to choose from {len(frontiers)} frontiers...')

        # Encode image
        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode('utf-8')

        # Build frontier description
        frontier_desc = "\n".join([
            f"Frontier {f['id']}: position ({f['x']}, {f['y']}, {f['z']}), size={f['size']}"
            for f in frontiers[:5]  # max 5
        ])

        prompt = f"""You are controlling a UAV drone exploring an indoor environment.
The drone's current camera view is attached.

Available frontiers (unexplored areas):
{frontier_desc}

Based on what you can see in the camera and the frontier positions:
1. Which frontier should the drone explore next?
2. Why did you choose it?
3. What do you see in the current camera view?

Respond ONLY with valid JSON:
{{"chosen_frontier": <id>, "reasoning": "<why>", "scene_description": "<what you see>"}}"""

        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {OPENAI_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "gpt-4o",
                    "max_tokens": 300,
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
                timeout=30
            )

            if response.status_code != 200:
                self.get_logger().error(f'API error: {response.status_code}')
                return

            content = response.json()['choices'][0]['message']['content'].strip()
            if '```' in content:
                content = content.split('```')[1]
                if content.startswith('json'):
                    content = content[4:]

            data = json.loads(content)
            self.get_logger().info(f"VLM sees: {data.get('scene_description', '?')}")
            self.get_logger().info(f"VLM chose frontier {data.get('chosen_frontier')} — {data.get('reasoning')}")

            # Publish decision
            msg = String()
            msg.data = json.dumps(data)
            self.decision_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'VLM error: {e}')

def main():
    rclpy.init()
    rclpy.spin(VLMFrontierSelector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
