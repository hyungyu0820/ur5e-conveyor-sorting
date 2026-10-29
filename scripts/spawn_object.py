#!/usr/bin/env python3
"""Spawn a 4 cm cube on the table (uses models/cube.sdf).

Usage:
  ros2 run ur_pick_place spawn_object.py NAME X Y [--color Blue]
  ros2 run ur_pick_place spawn_object.py box1 0.40 -0.15 --color Blue
  ros2 run ur_pick_place spawn_object.py box1 --delete

Colors: Red, Green, Blue, Yellow, Orange, Purple, White, Black ... (Gazebo/<Color> materials)
Keep cubes at least 6 cm apart, otherwise the gripper may grab the neighbour.
"""
import argparse
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose

TABLE_TOP = 0.20
CUBE_SIZE = 0.04


def call(node, client, request):
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{client.srv_name} not available, is the simulation running?")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("x", type=float, nargs="?")
    parser.add_argument("y", type=float, nargs="?")
    parser.add_argument("--color", default="Red")
    parser.add_argument("--delete", action="store_true", help="remove the object instead")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not args.delete and (args.x is None or args.y is None):
        parser.error("X and Y are required when spawning")

    rclpy.init()
    node = rclpy.create_node("spawn_object")

    if args.delete:
        res = call(node, node.create_client(DeleteEntity, "/delete_entity"), DeleteEntity.Request(name=args.name))
    else:
        sdf_path = os.path.join(get_package_share_directory("ur_pick_place"), "models", "cube.sdf")
        with open(sdf_path) as f:
            sdf = f.read().replace("Gazebo/Red", f"Gazebo/{args.color}")
        pose = Pose()
        pose.position.x, pose.position.y = args.x, args.y
        pose.position.z = TABLE_TOP + CUBE_SIZE / 2 + 0.002  # drop from 2 mm
        pose.orientation.w = 1.0
        res = call(node, node.create_client(SpawnEntity, "/spawn_entity"),
                   SpawnEntity.Request(name=args.name, xml=sdf, initial_pose=pose))

    ok = res is not None and res.success
    if res is not None:
        node.get_logger().info(res.status_message)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
