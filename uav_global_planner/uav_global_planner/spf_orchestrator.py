#!/usr/bin/env python3
"""
SPF-E: SPF Orchestrator (Global Planner)
Subscribes to /spf/target_pose from VLM spatial grounding
Calls /uav/navigate_to_goal action (A* handles obstacle-aware path)
Publishes status on /uav/global_planner_status

State machine: IDLE → EXECUTING → COMPLETE/ABORTED
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_planner_interface.action import NavigateToGoal
import threading, json
from enum import Enum

class State(Enum):
    IDLE      = "IDLE"
    EXECUTING = "EXECUTING"
    COMPLETE  = "COMPLETE"
    ABORTED   = "ABORTED"

class SPFOrchestrator(Node):
    def __init__(self):
        super().__init__("spf_orchestrator")

        self.state         = State.IDLE
        self.current_goal  = None
        self.goal_handle   = None
        self._lock         = threading.Lock()

        # Subscribers
        self.create_subscription(
            PoseStamped, "/spf/target_pose",
            self.target_pose_cb, 10)
        self.create_subscription(
            String, "/uav/emergency_trigger",
            self.emergency_cb, 10)

        # Publishers
        self.status_pub = self.create_publisher(
            String, "/uav/global_planner_status", 10)

        # Action client → planner_server → waypoint_manager → mp_node
        self.nav_client = ActionClient(
            self, NavigateToGoal, "/uav/navigate_to_goal")

        # Status timer
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info("SPF Orchestrator ready")
        self.get_logger().info(
            "Listening on /spf/target_pose → /uav/navigate_to_goal")

    def target_pose_cb(self, msg):
        with self._lock:
            if self.state == State.EXECUTING:
                self.get_logger().warn(
                    "Already executing — cancelling previous goal")
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()

            self.current_goal = msg
            self.state = State.EXECUTING

        self.get_logger().info(
            f"New target: ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, "
            f"{msg.pose.position.z:.2f})")

        threading.Thread(
            target=self.execute_goal,
            args=(msg,), daemon=True).start()

    def emergency_cb(self, msg):
        self.get_logger().error("Emergency trigger — aborting!")
        with self._lock:
            self.state = State.ABORTED
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()

    def execute_goal(self, pose_stamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available!")
            with self._lock:
                self.state = State.ABORTED
            return

        goal = NavigateToGoal.Goal()
        goal.target_pose            = pose_stamped
        goal.planning_timeout_sec   = 15.0

        self.get_logger().info("Sending goal to planner server...")
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback):
        pct = feedback.feedback.percent_complete
        self.get_logger().info(
            f"Planning: {pct:.0f}%", throttle_duration_sec=2.0)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected by planner!")
            with self._lock:
                self.state = State.ABORTED
            return

        self.get_logger().info("Goal accepted — executing...")
        with self._lock:
            self.goal_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        code   = result.result_code

        with self._lock:
            if code == "SUCCESS":
                self.state = State.COMPLETE
                self.get_logger().info("✅ Goal reached!")
            else:
                self.state = State.ABORTED
                self.get_logger().warn(f"Goal failed: {code}")
            self.goal_handle = None

    def publish_status(self):
        msg = String()
        with self._lock:
            status = {
                "state": self.state.value,
                "has_goal": self.current_goal is not None
            }
            if self.current_goal:
                status["target"] = [
                    round(self.current_goal.pose.position.x, 2),
                    round(self.current_goal.pose.position.y, 2),
                    round(self.current_goal.pose.position.z, 2),
                ]
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(SPFOrchestrator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
