#!/usr/bin/env python3
"""
send_and_record.py — publish a path to the waypoint manager, record a ROS 2 bag
until the drone stops.

Publishes a nav_msgs/Path to /uav/global_path so the waypoint_manager handles
waypoint sequencing and mission_complete signalling.  This is the proper
integration path — no more raw PointStamped bypass.

Stops recording when ANY condition holds for --settle-sec (default 3 s):
  - /uav/mission_complete is True (waypoint manager declares done)
  - /uav/vfh_status is IDLE, STALLED, or ORBITING
  - /uav/cmd_vel magnitude < 0.05 m/s  (zero-velocity after the drone moved)
  - Ctrl-C

Usage:
    python3 scripts/send_and_record.py --x 5.0 --y 0.0 --z 1.5
    python3 scripts/send_and_record.py --x 5.0 --y 0.0 --z 1.5 --bag-prefix obs_test --settle-sec 5

Add --with-cloud to also capture /drone/tof_merged/points (large).
"""

import argparse
import datetime
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


TERMINAL_STATUSES = frozenset({"IDLE", "STALLED", "ORBITING"})


class Monitor(Node):
    def __init__(self, x: float, y: float, z: float, settle_sec: float):
        super().__init__("send_and_record")
        self._goal = (x, y, z)
        self._settle = settle_sec

        self._path_pub = self.create_publisher(Path, "/uav/global_path", 10)
        self.create_subscription(String,       "/uav/vfh_status",      self._on_status, 10)
        self.create_subscription(TwistStamped, "/uav/cmd_vel",         self._on_cmd,    10)
        self.create_subscription(Bool,         "/uav/mission_complete", self._on_mission, 10)
        self.create_subscription(Odometry,     "/drone/odom",          self._on_odom,   10)

        self._status: str = ""
        self._status_start: float = 0.0
        self._zero_vel_start: float | None = None
        self._ever_moved: bool = False
        self._mission_done: bool = False
        self._start_pos: tuple[float, float, float] | None = None

        self.stop_reason: str = ""
        self.done: bool = False

        self._path_sends = 0
        self.create_timer(0.5, self._send_path)

    # ── odometry callback — capture start position for the path ────────────
    def _on_odom(self, msg: Odometry):
        if self._start_pos is None:
            self._start_pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )

    # ── path publisher (fires 3 times then stops) ──────────────────────────
    def _send_path(self):
        if self._path_sends >= 3:
            return
        if self._start_pos is None:
            return  # don't publish until we have a start position

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # Start pose (current drone position)
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x, start.pose.position.y, start.pose.position.z = self._start_pos
        start.pose.orientation.w = 1.0
        path.poses.append(start)

        # Goal pose
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self._goal
        goal.pose.orientation.w = 1.0
        path.poses.append(goal)

        self._path_pub.publish(path)
        self._path_sends += 1
        if self._path_sends == 1:
            self.get_logger().info(
                f"Path sent: ({self._start_pos[0]:.2f},{self._start_pos[1]:.2f},{self._start_pos[2]:.2f})"
                f" → ({self._goal[0]:.2f},{self._goal[1]:.2f},{self._goal[2]:.2f})"
            )

    # ── mission_complete callback ────────────────────────────────────────
    def _on_mission(self, msg: Bool):
        if msg.data and not self.done:
            self._mission_done = True
            self.stop_reason = "mission_complete=True from waypoint manager"
            self.done = True

    # ── status callback ───────────────────────────────────────────────────
    def _on_status(self, msg: String):
        if msg.data != self._status:
            self._status = msg.data
            self._status_start = time.monotonic()

        if self._status in TERMINAL_STATUSES and self._path_sends >= 1:
            held = time.monotonic() - self._status_start
            if held >= self._settle and not self.done:
                self.stop_reason = f"status={self._status!r} held for {held:.1f} s"
                self.done = True

    # ── cmd_vel callback ──────────────────────────────────────────────────
    def _on_cmd(self, msg: TwistStamped):
        v = msg.twist.linear
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        now = time.monotonic()

        if speed > 0.05:
            self._ever_moved = True
            self._zero_vel_start = None
        else:
            if self._zero_vel_start is None:
                self._zero_vel_start = now
            elif self._ever_moved:
                held = now - self._zero_vel_start
                if held >= self._settle and not self.done:
                    self.stop_reason = f"cmd_vel ≈ 0 for {held:.1f} s after movement"
                    self.done = True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send a path and record a ROS 2 bag until the drone stops."
    )
    parser.add_argument("--x",          type=float, required=True,  help="Goal X (map frame, m)")
    parser.add_argument("--y",          type=float, required=True,  help="Goal Y (map frame, m)")
    parser.add_argument("--z",          type=float, default=1.5,    help="Goal Z / altitude (default 1.5 m)")
    parser.add_argument("--bag-prefix", default="mp_run",           help="Bag file prefix (default mp_run)")
    parser.add_argument("--settle-sec", type=float, default=3.0,
                        help="Seconds a stop condition must hold before stopping (default 3)")
    parser.add_argument("--with-cloud", action="store_true",
                        help="Also record /drone/tof_merged/points (warning: large)")
    args = parser.parse_args()

    # ── Bag name ──────────────────────────────────────────────────────────
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"{args.bag_prefix}_{ts}"

    # Place bags in the workspace-level bags/ directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = os.path.normpath(os.path.join(script_dir, "..", ".."))
    bags_dir = os.path.join(ws_root, "bags")
    bag_path = os.path.join(bags_dir, bag_name)

    topics = [
        "/drone/odom",
        "/uav/cmd_vel",
        "/uav/vfh_status",
        "/uav/mp_diag",
        "/uav/current_waypoint",
        "/uav/global_path",
        "/uav/mission_complete",
    ]
    if args.with_cloud:
        topics.append("/drone/tof_merged/points")

    # ── Start bag recording ───────────────────────────────────────────────
    os.makedirs(bags_dir, exist_ok=True)
    print(f"\nBag: {bag_path}")
    print(f"Topics: {', '.join(topics)}\n")

    bag_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", bag_path, *topics],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.8)  # let ros2 bag initialise before publishing

    # ── ROS 2 node ────────────────────────────────────────────────────────
    rclpy.init()
    node = Monitor(args.x, args.y, args.z, args.settle_sec)

    print(f"Waiting for stop condition (settle={args.settle_sec}s) — Ctrl-C to abort early ...\n")
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.stop_reason = "user interrupt (Ctrl-C)"
        print()

    finally:
        node.destroy_node()
        rclpy.shutdown()

    # ── Stop recording ────────────────────────────────────────────────────
    bag_proc.send_signal(signal.SIGINT)
    try:
        bag_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        bag_proc.kill()

    abs_bag = os.path.abspath(bag_path)
    print(f"Stop reason : {node.stop_reason}")
    print(f"Bag saved   : {abs_bag}/")
    print()
    print("Replay:")
    print(f"  ros2 bag play {bag_path}")
    print("Plot (PlotJuggler):")
    print(f"  ros2 run plotjuggler plotjuggler")
    print("Quick check — print diag topic:")
    print(f"  ros2 bag play {bag_path} --topics /uav/mp_diag | ros2 topic echo /uav/mp_diag")


if __name__ == "__main__":
    main()
