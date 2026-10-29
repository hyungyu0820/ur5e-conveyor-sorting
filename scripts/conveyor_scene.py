#!/usr/bin/env python3
"""Sorting cell: spawns the colour bins once, then feeds random items onto the conveyor forever.

A new item is dropped every ITEM_SPACING metres of belt travel, so items queue up with even
gaps while the belt is stopped at the light curtain. To keep Gazebo fast, the oldest items
(already sorted into the bins) are deleted once more than --max-items exist.

Item names contain their true colour (item_0007_red) so results can be checked afterwards;
the sorter does not use the names, it recognises the colour with the wrist camera.

Started by: ros2 launch ur_pick_place ur_gripper_sim.launch.py scene:=conveyor
"""
import argparse
import os
import random
import sys
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conveyor_layout as layout  # noqa: E402

CUBE_SIZE = 0.04


def read_model(name):
    with open(os.path.join(get_package_share_directory("ur_pick_place"), "models", f"{name}.sdf")) as f:
        return f.read()


class ConveyorScene(Node):
    def __init__(self, max_items, seed):
        super().__init__("conveyor_scene")
        self.max_items = max_items
        self.rng = random.Random(seed)
        self.cube_sdf = read_model("cube")
        self.bin_sdf = read_model("bin")
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        self.running = None  # None until the conveyor plugin publishes, i.e. the belt exists
        self.travel = layout.ITEM_SPACING  # spawn the first item right away
        self.count = 0
        self.items = deque()
        self.pending = None  # in-flight service call
        self.bins_done = False

        self.create_subscription(Bool, "/conveyor/running", self._on_running, 10)
        self.period = 0.1
        self.create_timer(self.period, self._tick)

    def _on_running(self, msg):
        self.running = msg.data

    def _spawn(self, name, sdf, x, y, z):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        return self.spawn_cli.call_async(SpawnEntity.Request(name=name, xml=sdf, initial_pose=pose))

    def _tick(self):
        if not self.spawn_cli.service_is_ready() or self.running is None:
            return
        if self.pending is not None and not self.pending.done():
            return
        self.pending = None

        if not self.bins_done:
            for label, (x, y) in layout.BINS.items():
                sdf = self.bin_sdf.replace("Gazebo/Red", f"Gazebo/{layout.GAZEBO_COLORS[label]}")
                self._spawn(f"bin_{label}", sdf, x, y, 0.0)
            self.bins_done = True
            self.get_logger().info(f"bins spawned: {', '.join(layout.BINS)}")
            return

        if self.running:
            self.travel += layout.BELT_SPEED * self.period
        if self.travel < layout.ITEM_SPACING:
            return

        if len(self.items) >= self.max_items:
            # one call per tick: delete first, spawn on a later tick
            old = self.items.popleft()
            self.pending = self.delete_cli.call_async(DeleteEntity.Request(name=old))
            return

        label = self.rng.choice(list(layout.GAZEBO_COLORS))
        name = f"item_{self.count:04d}_{label}"
        sdf = self.cube_sdf.replace("Gazebo/Red", f"Gazebo/{layout.GAZEBO_COLORS[label]}")
        self.pending = self._spawn(name, sdf, layout.BELT_X, layout.SPAWN_Y, layout.BELT_TOP + CUBE_SIZE / 2 + 0.003)
        self.items.append(name)
        self.count += 1
        self.travel = 0.0
        self.get_logger().info(f"new item {name}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-items", type=int, default=30)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = ConveyorScene(args.max_items, args.seed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
