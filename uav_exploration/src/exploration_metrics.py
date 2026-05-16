#!/usr/bin/env python3
"""
Improved Exploration Metrics Logger
Tracks:
- Distance traveled
- Unique frontiers visited vs revisits
- VLM decisions with reasoning
- Frontiers per minute efficiency
- Saves decision log with images
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math, csv, os, time, json, cv2

LOG_DIR = "/scratch/e1547056/irobot/logs"
CSV_PATH = f"{LOG_DIR}/metrics.csv"
DECISION_LOG = f"{LOG_DIR}/vlm_decisions.txt"
IMAGE_DIR = f"{LOG_DIR}/vlm_images"
os.makedirs(IMAGE_DIR, exist_ok=True)

class ExplorationMetrics(Node):
    def __init__(self):
        super().__init__('exploration_metrics')

        self.declare_parameter('run_name', 'baseline')
        self.run_name = self.get_parameter('run_name').value

        self.bridge = CvBridge()
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_pos = None
        self.status = "WAITING"
        self.latest_frame = None

        # Frontier tracking
        self.unique_frontiers = set()
        self.revisited_frontiers = 0
        self.current_frontier = None
        self.frontiers_reached = 0

        # VLM tracking
        self.vlm_calls = 0
        self.vlm_decisions = []

        # Subscribers
        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, 10)
        self.create_subscription(String, '/uav/exploration_status', self.status_cb, 10)
        self.create_subscription(String, '/uav/vlm_decision', self.vlm_cb, 10)
        self.create_subscription(MarkerArray, '/uav/exploration_frontiers', self.frontier_cb, 10)
        self.create_subscription(Image, '/drone/rgbd/image', self.image_cb, 10)

        # Timers
        self.create_timer(5.0, self.log_metrics)
        self.create_timer(60.0, self.print_summary)

        # CSV header
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'run_name', 'elapsed_s', 'distance_m',
                    'unique_frontiers', 'revisits', 'frontiers_reached',
                    'vlm_calls', 'frontiers_per_min',
                    'efficiency_m_per_frontier', 'status'
                ])

        self.get_logger().info(f'Metrics logger started — run: {self.run_name}')

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        if self.last_pos is not None:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dz = pos.z - self.last_pos[2]
            self.total_distance += math.sqrt(dx**2 + dy**2 + dz**2)
        self.last_pos = (pos.x, pos.y, pos.z)

    def status_cb(self, msg):
        self.status = msg.data
        if 'reached' in msg.data.lower() or 'EXPLORING' in msg.data:
            self.frontiers_reached += 1

    def frontier_cb(self, msg):
        for marker in msg.markers:
            if marker.action == 0:
                # Round to 1 decimal to identify unique locations
                key = (
                    round(marker.pose.position.x, 1),
                    round(marker.pose.position.y, 1),
                    round(marker.pose.position.z, 1)
                )
                if key in self.unique_frontiers:
                    self.revisited_frontiers += 1
                else:
                    self.unique_frontiers.add(key)

    def vlm_cb(self, msg):
        self.vlm_calls += 1
        elapsed = time.time() - self.start_time

        try:
            data = json.loads(msg.data)
            frontier = data.get('chosen_frontier', '?')
            reasoning = data.get('reasoning', '?')
            scene = data.get('scene_description', '?')

            self.get_logger().info(
                f'\n=== VLM Decision #{self.vlm_calls} ===\n'
                f'Scene: {scene}\n'
                f'Chose frontier: {frontier}\n'
                f'Reasoning: {reasoning}\n'
            )

            # Save decision to log file
            with open(DECISION_LOG, 'a') as f:
                f.write(f"\n{'='*50}\n")
                f.write(f"Decision #{self.vlm_calls} at t={elapsed:.0f}s\n")
                f.write(f"Run: {self.run_name}\n")
                f.write(f"Scene: {scene}\n")
                f.write(f"Chosen frontier: {frontier}\n")
                f.write(f"Reasoning: {reasoning}\n")
                f.write(f"Distance so far: {self.total_distance:.1f}m\n")

            # Save camera image at decision point
            if self.latest_frame is not None:
                img_path = f"{IMAGE_DIR}/{self.run_name}_decision_{self.vlm_calls}_t{elapsed:.0f}.jpg"
                cv2.imwrite(img_path, self.latest_frame)
                self.get_logger().info(f'Saved decision image: {img_path}')

            self.vlm_decisions.append({
                'call': self.vlm_calls,
                'time': elapsed,
                'frontier': frontier,
                'reasoning': reasoning,
                'scene': scene,
                'distance': self.total_distance
            })

        except Exception as e:
            self.get_logger().error(f'VLM decision parse error: {e}')

    def log_metrics(self):
        elapsed = time.time() - self.start_time
        minutes = elapsed / 60.0
        frontiers_per_min = len(self.unique_frontiers) / minutes if minutes > 0 else 0
        efficiency = (self.total_distance / len(self.unique_frontiers)
                     if len(self.unique_frontiers) > 0 else 0)

        self.get_logger().info(
            f'[{self.run_name}] '
            f't={elapsed:.0f}s | '
            f'dist={self.total_distance:.1f}m | '
            f'unique_frontiers={len(self.unique_frontiers)} | '
            f'revisits={self.revisited_frontiers} | '
            f'vlm_calls={self.vlm_calls} | '
            f'frontiers/min={frontiers_per_min:.1f} | '
            f'efficiency={efficiency:.1f}m/frontier'
        )

        with open(CSV_PATH, 'a', newline='') as f:
            csv.writer(f).writerow([
                self.run_name,
                round(elapsed, 1),
                round(self.total_distance, 2),
                len(self.unique_frontiers),
                self.revisited_frontiers,
                self.frontiers_reached,
                self.vlm_calls,
                round(frontiers_per_min, 2),
                round(efficiency, 2),
                self.status
            ])

    def print_summary(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'RUN SUMMARY — {self.run_name}\n'
            f'{"="*50}\n'
            f'Time elapsed:        {elapsed:.0f}s\n'
            f'Distance traveled:   {self.total_distance:.1f}m\n'
            f'Unique frontiers:    {len(self.unique_frontiers)}\n'
            f'Frontier revisits:   {self.revisited_frontiers}\n'
            f'Frontiers reached:   {self.frontiers_reached}\n'
            f'VLM calls:           {self.vlm_calls}\n'
            f'{"="*50}\n'
        )

def main():
    rclpy.init()
    rclpy.spin(ExplorationMetrics())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
