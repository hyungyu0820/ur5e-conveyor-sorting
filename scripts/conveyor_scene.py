#!/usr/bin/env python3
"""Sorting cell: spawns the colour bins once, then feeds random items onto the conveyor forever.

A new item is dropped every item_spacing metres of belt travel (read from the belt encoder,
so the gap does not depend on how fast the simulation runs). To keep Gazebo fast, the oldest items
(already sorted into the bins) are deleted once more than --max-items exist.

Item names contain their kind and true colour (item_0007_red, long_0008_blue, pair_0009a_red,
unk_0010_purple) so results can be checked afterwards; the sorter does not use the names.

Fault mode (--faults true, launch faults:=true) mixes in exceptions (layout.FAULT_MIX):
rotated long boxes, overlapping pairs (side by side or stacked) and unknown purple items,
and sets the gripper's slip and drop rates (layout.SLIP_RATE, layout.DROP_RATE).

Started by: ros2 launch ur_pick_place ur_gripper_sim.launch.py scene:=conveyor
"""
import argparse
import math
import os
import random
import sys
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conveyor_layout as layout  # noqa: E402

CUBE_SIZE = 0.04


def read_model(name):
    with open(os.path.join(get_package_share_directory("ur_pick_place"), "models", f"{name}.sdf")) as f:
        return f.read()


class ConveyorScene(Node):
    def __init__(self, max_items, seed, faults, continuous, belt_speed, item_spacing):
        super().__init__("conveyor_scene")
        self.max_items = max_items
        self.rng = random.Random(seed)
        self.faults = faults
        self.continuous = continuous
        self.belt_speed = belt_speed
        self.item_spacing = item_spacing
        self.belt_params = self.create_client(SetParameters, "/conveyor_belt/set_parameters")
        self.belt_set = False
        self.cube_sdf = read_model("cube")
        self.long_sdf = read_model("long_box")
        self.bin_sdf = read_model("bin")
        self.gripper_params = self.create_client(SetParameters, "/grasp_attacher/set_parameters")
        self.faults_set = not faults
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        self.encoder = None  # None until the conveyor plugin publishes, i.e. the belt exists
        self.last_item_at = None  # encoder value at the last spawn
        self.count = 0
        self.items = deque()
        self.pending = None  # in-flight service call
        self.bins_done = False

        self.create_subscription(JointState, "/conveyor/encoder", self._on_encoder, 10)
        self.create_timer(0.02, self._tick)  # wall clock: keeps the spacing exact when the simulation runs fast

    def _on_encoder(self, msg):
        self.encoder = msg.position[0]

    def _spawn(self, name, sdf, x, y, z, yaw=0.0):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self.spawn_cli.call_async(SpawnEntity.Request(name=name, xml=sdf, initial_pose=pose))

    def _set_gripper_faults(self):
        def dbl(name, v):
            return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v))

        self.gripper_params.call_async(
            SetParameters.Request(parameters=[dbl("slip_rate", layout.SLIP_RATE), dbl("drop_rate", layout.DROP_RATE)])
        )
        self.faults_set = True
        self.get_logger().info(f"gripper faults: slip {layout.SLIP_RATE:.0%}, drop {layout.DROP_RATE:.0%}")

    def _set_belt(self):
        self.belt_params.call_async(
            SetParameters.Request(parameters=[
                Parameter(name="speed", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.belt_speed)),
                Parameter(name="stop_at_sensor", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=not self.continuous)),
            ])
        )
        self.belt_set = True
        self.get_logger().info(
            f"belt: {self.belt_speed:.3f} m/s, item every {self.item_spacing:.2f} m, "
            f"{'never stops' if self.continuous else 'stops at the light curtain'}"
        )

    def _cube(self, color):
        return self.cube_sdf.replace("Gazebo/Red", f"Gazebo/{color}")

    def _feed(self):
        """Drop the next item (or pair) on the belt; returns the names spawned."""
        n = self.count
        z = layout.BELT_TOP + CUBE_SIZE / 2 + 0.003
        x, y = layout.BELT_X, layout.SPAWN_Y
        label = self.rng.choice(list(layout.GAZEBO_COLORS))
        color = layout.GAZEBO_COLORS[label]
        kind = self.rng.choices(list(layout.FAULT_MIX), weights=list(layout.FAULT_MIX.values()))[0] if self.faults else "normal"

        if kind == "long":
            yaw = self.rng.uniform(0, math.pi)
            name = f"long_{n:04d}_{label}"
            self.pending = self._spawn(name, self.long_sdf.replace("Gazebo/Red", f"Gazebo/{color}"), x, y, z, yaw)
            self.get_logger().info(f"new item {name} (rotated {math.degrees(yaw):.0f} deg)")
            return [name]
        if kind == "pair":
            label2 = self.rng.choice(list(layout.GAZEBO_COLORS))
            a, b = f"pair_{n:04d}a_{label}", f"pair_{n:04d}b_{label2}"
            if self.rng.random() < 0.5:  # side by side across the belt, touching
                self._spawn(a, self._cube(color), x - 0.021, y, z)
                self.pending = self._spawn(b, self._cube(layout.GAZEBO_COLORS[label2]), x + 0.021, y, z)
                how = "side by side"
            else:  # stacked
                self._spawn(a, self._cube(color), x, y, z)
                self.pending = self._spawn(b, self._cube(layout.GAZEBO_COLORS[label2]), x, y, z + CUBE_SIZE + 0.002)
                how = "stacked"
            self.get_logger().info(f"new items {a} + {b} ({how})")
            return [a, b]
        if kind == "unknown":
            name = f"unk_{n:04d}_purple"
            self.pending = self._spawn(name, self._cube(layout.UNKNOWN_COLOR), x, y, z)
            self.get_logger().info(f"new item {name} (unknown)")
            return [name]
        name = f"item_{n:04d}_{label}"
        self.pending = self._spawn(name, self._cube(color), x, y, z)
        self.get_logger().info(f"new item {name}")
        return [name]

    def _tick(self):
        if not self.spawn_cli.service_is_ready() or self.encoder is None:
            return
        if self.pending is not None and not self.pending.done():
            return
        self.pending = None

        if not self.bins_done:
            for label, (x, y) in layout.BINS.items():
                sdf = self.bin_sdf.replace("Gazebo/Red", f"Gazebo/{layout.GAZEBO_COLORS[label]}")
                self._spawn(f"bin_{label}", sdf, x, y, 0.0)
            self._spawn("bin_hold", self.bin_sdf.replace("Gazebo/Red", "Gazebo/Grey"), *layout.REJECT, 0.0)
            self.bins_done = True
            self.get_logger().info(f"bins spawned: {', '.join(layout.BINS)}, hold")
            return
        if not self.faults_set and self.gripper_params.service_is_ready():
            self._set_gripper_faults()

        if not self.belt_set:
            if self.belt_params.service_is_ready():
                self._set_belt()
            return

        if self.last_item_at is not None and self.encoder - self.last_item_at < self.item_spacing:
            return

        if len(self.items) >= self.max_items:
            # one call per tick: delete first, spawn on a later tick
            old = self.items.popleft()
            self.pending = self.delete_cli.call_async(DeleteEntity.Request(name=old))
            return

        self.items.extend(self._feed())
        self.count += 1
        self.last_item_at = self.encoder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-items", type=int, default=30)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--faults", default="false", help="true: mix in exceptions and gripper faults")
    parser.add_argument("--continuous", default="false", help="true: the belt never stops at the light curtain")
    parser.add_argument("--belt-speed", type=float, default=layout.BELT_SPEED)
    parser.add_argument("--item-spacing", type=float, default=layout.ITEM_SPACING)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = ConveyorScene(
        args.max_items, args.seed, args.faults.lower() == "true",
        args.continuous.lower() == "true", args.belt_speed, args.item_spacing,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
