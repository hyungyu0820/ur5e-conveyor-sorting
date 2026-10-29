#!/usr/bin/env python3
"""Open/close the simulated gripper.

Usage:
  ros2 run ur_pick_place gripper.py open
  ros2 run ur_pick_place gripper.py close
  ros2 run ur_pick_place gripper.py 0.02     # finger travel in metres (0.0 = open, 0.04 = closed)
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

OPEN = 0.0
CLOSED = 0.04


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    arg = sys.argv[1]
    if arg == "open":
        target = OPEN
    elif arg == "close":
        target = CLOSED
    else:
        target = min(max(float(arg), OPEN), CLOSED)

    rclpy.init()
    node = Node("gripper_command")
    pub = node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

    # wait until the controller has subscribed, otherwise the message is lost
    while pub.get_subscription_count() == 0:
        node.get_logger().info("waiting for /gripper_controller/commands subscriber...", once=True)
        rclpy.spin_once(node, timeout_sec=0.2)

    pub.publish(Float64MultiArray(data=[target, target]))
    node.get_logger().info(f"gripper -> {target:.3f} m per finger")
    rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
