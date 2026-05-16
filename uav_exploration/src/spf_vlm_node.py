#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from uav_planner_interface.action import NavigateToGoal
import cv2, base64, json, requests, os
import numpy as np

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
CAMERA_HFOV = 1.047
STEP_SIZE = 1.5

class SPFVLMNode(Node):
    def __init__(self):
        super().__init__("spf_vlm_node")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.5
        self.create_subscription(Image, "/drone/rgbd/image", self.image_cb, 10)
        self.create_subscription(String, "/uav/vlm_instruction", self.instruction_cb, 10)
        self.nav_client = ActionClient(self, NavigateToGoal, "/uav/navigate_to_goal")
        self.status_pub = self.create_publisher(String, "/uav/vlm_status", 10)
        self.get_logger().info("SPF VLM Node ready!")
        self.get_logger().info("Send instructions to /uav/vlm_instruction")

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def instruction_cb(self, msg):
        instruction = msg.data
        self.get_logger().info(f"Instruction: {instruction}")
        if self.latest_frame is None:
            self.get_logger().warn("No camera frame yet!")
            return
        self.process(instruction)

    def process(self, instruction):
        frame = cv2.resize(self.latest_frame, (640, 480))
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_b64 = base64.b64encode(buf).decode("utf-8")
        prompt = f"""You are controlling a drone. Instruction: "{instruction}"
Respond with ONLY valid JSON no markdown:
{{"u": <pixel 0-640>, "v": <pixel 0-480>, "distance": <1-3>, "description": "<what you see>"}}"""
        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers={"Authorization": f"Bearer {OPENAI_API_KEY}", "Content-Type": "application/json"},
                json={"model": "gpt-4o", "max_tokens": 200, "messages": [{"role": "user", "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}", "detail": "low"}}
                ]}]},
                timeout=30
            )
            self.get_logger().info(f"API status: {response.status_code}")
            if response.status_code != 200:
                self.get_logger().error(f"API error: {response.text}")
                return
            result = response.json()
            content = result["choices"][0]["message"]["content"].strip()
            self.get_logger().info(f"VLM response: {content}")
            if "```" in content:
                content = content.split("```")[1]
                if content.startswith("json"):
                    content = content[4:]
            data = json.loads(content)
            self.get_logger().info(f"VLM sees: {data.get('description','?')}")
            self.pixel_to_goal(data["u"], data["v"], data.get("distance", 2))
        except Exception as e:
            self.get_logger().error(f"VLM error: {e}")

    def pixel_to_goal(self, u, v, dist_label):
        nx = (u - 320) / 320
        ny = (v - 240) / 240
        step = STEP_SIZE * dist_label
        dx = step * np.cos(nx * CAMERA_HFOV / 2)
        dy = -step * np.sin(nx * CAMERA_HFOV / 2)
        dz = -ny * 0.5
        goal_x = self.drone_x + dx
        goal_y = self.drone_y + dy
        goal_z = max(1.0, self.drone_z + dz)
        self.get_logger().info(f"Sending goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})")
        self.send_goal(goal_x, goal_y, goal_z)

    def send_goal(self, x, y, z):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Action server not available")
            return
        goal = NavigateToGoal.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        goal.planning_timeout_sec = 15.0
        self.nav_client.send_goal_async(goal)
        msg = String()
        msg.data = f"FLYING_TO ({x:.1f},{y:.1f},{z:.1f})"
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(SPFVLMNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
