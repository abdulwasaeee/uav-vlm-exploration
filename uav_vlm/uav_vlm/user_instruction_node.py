#!/usr/bin/env python3
"""User Instruction Interface — Clean HRI"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading, json, time

class UserInstructionNode(Node):
    def __init__(self):
        super().__init__("user_instruction_node")
        self.busy = False

        self.instr_pub = self.create_publisher(String, "/user/instruction", 10)

        self.create_subscription(String, "/uav/vlm_grounding_status", self.status_cb, 10)
        self.create_subscription(PoseStamped, "/spf/target_pose", self.pose_cb, 10)
        self.create_subscription(String, "/uav/global_planner_status", self.planner_cb, 10)

        threading.Thread(target=self.run, daemon=True).start()

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            label = data.get("label", "")
            conf  = data.get("confidence", 0)
            if label:
                if conf > 0.6:
                    print("\n🤖 I can see: " + label)
                else:
                    print("\n🤖 I think I can see: " + label + " (not very confident)")
        except Exception:
            pass

    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        print("\n🤖 Flying to ({:.1f}, {:.1f}, {:.1f})...".format(x, y, z))

    def planner_cb(self, msg):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            if state == "COMPLETE":
                print("\n🤖 Done! I have reached the target.")
                self.busy = False
                self.prompt()
            elif state == "ABORTED":
                print("\n🤖 I could not reach the target. Please try again.")
                self.busy = False
                self.prompt()
        except Exception:
            pass

    def prompt(self):
        print("\nYou: ", end="", flush=True)

    def run(self):
        time.sleep(3.0)
        sep = "=" * 55
        print("")
        print(sep)
        print(" Hi! I am your inspection drone.")
        print(" How can I help you today?")
        print(sep)
        print(" You can ask me to:")
        print("   - Fly to an object  e.g. fly to the red box")
        print("   - Find something    e.g. find the yellow cylinder")
        print("   - Describe the area e.g. what do you see")
        print("   - Move around       e.g. go forward, move left")
        print(" Type quit to exit.")
        print(sep)

        while rclpy.ok():
            try:
                self.prompt()
                user_input = input().strip()
                if not user_input:
                    continue
                if user_input.lower() == "quit":
                    print("\n🤖 Goodbye! Hovering in place.")
                    break
                msg = String()
                msg.data = user_input
                self.instr_pub.publish(msg)
                self.busy = True
                print("\n🤖 Got it! Working on it...")
            except EOFError:
                break

def main():
    rclpy.init()
    rclpy.spin(UserInstructionNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
