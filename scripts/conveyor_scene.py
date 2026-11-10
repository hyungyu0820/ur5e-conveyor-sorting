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

For experiments (launch feed_limit:=N wait_start:=true): --feed-limit stops after N feeds (a pair
counts as one), --wait-start holds the first item back until a sorter says it is ready on
/feeder/start, so no run loses its first item to start-up timing. /feeder/status (JSON) lists what
was fed; it is for grading and for knowing when a run is over.

Started by: ros2 launch ur_pick_place ur_gripper_sim.launch.py scene:=conveyor
"""
import argparse
import json
import math
import os
import random
import sys
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conveyor_layout as layout  # noqa: E402

CUBE_SIZE = 0.04


def read_model(name):
    with open(os.path.join(get_package_share_directory("ur_pick_place"), "models", f"{name}.sdf")) as f:
        return f.read()


class ConveyorScene(Node):
    def __init__(self, max_items, seed, faults, continuous, belt_speed, item_spacing, feed_limit=0, wait_start=False):
        super().__init__("conveyor_scene")
        self.max_items = max(max_items, 2 * feed_limit + 4)  # an experiment keeps every item for grading
        self.seed = seed
        self.rng = random.Random(seed)
        self.feed_limit = feed_limit
        self.started = not wait_start
        self.manifest = []  # what was fed: {slot, kind, names, encoder, ...}
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
        self.requests = {}   # parameter requests waiting for their answer: key -> (future, time sent)
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        self.encoder = None  # None until the conveyor plugin publishes, i.e. the belt exists
        self.last_item_at = None  # encoder value at the last spawn
        self.count = 0
        self.items = deque()
        self.pending = None  # in-flight service call
        self.steps = []      # calls still to make for the pair being put down, one per tick
        self.pause_cli = self.create_client(EmptySrv, "/pause_physics")
        self.unpause_cli = self.create_client(EmptySrv, "/unpause_physics")
        self.bins_done = False

        self.create_subscription(JointState, "/conveyor/encoder", self._on_encoder, 10)
        self.create_subscription(Empty, "/feeder/start", self._on_start, 10)
        self.status_pub = self.create_publisher(String, "/feeder/status", 10)
        self.create_timer(0.25, self._publish_status)
        self.create_timer(0.02, self._tick)  # wall clock: keeps the spacing exact when the simulation runs fast

    def _on_encoder(self, msg):
        self.encoder = msg.position[0]

    def _on_start(self, _msg):
        if not self.started:
            self.get_logger().info("sorter is ready, feeding starts")
        self.started = True

    def _publish_status(self):
        done = self.feed_limit > 0 and self.count >= self.feed_limit
        self.status_pub.publish(String(data=json.dumps({
            "fed": self.count, "limit": self.feed_limit, "done": done, "waiting_for_start": not self.started,
            "faults": self.faults, "faults_confirmed": self.faults and self.faults_set,
            "last_item_at": self.last_item_at, "encoder": self.encoder, "seed": self.seed, "items": self.manifest,
        })))

    def _spawn(self, name, sdf, x, y, z, yaw=0.0):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self.spawn_cli.call_async(SpawnEntity.Request(name=name, xml=sdf, initial_pose=pose))

    def _set_gripper_faults(self):
        def dbl(name, v):
            return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v))

        seed = Parameter(name="seed", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=self.seed or 0))
        return self.gripper_params.call_async(
            SetParameters.Request(parameters=[dbl("slip_rate", layout.SLIP_RATE), dbl("drop_rate", layout.DROP_RATE), seed])
        )

    def _confirm(self, key, send, what):
        """Send a parameter request and only believe it once the answer says every value was taken.

        A request sent the moment a service appears can get lost; without this check a run would
        silently go on with the old values (no gripper faults, or the default belt speed).
        Returns True when confirmed; call again on later ticks until then.
        """
        future, sent_at = self.requests.get(key, (None, 0.0))
        now = time.monotonic()
        if future is not None and future.done():
            result = future.result()
            if result is not None and all(r.successful for r in result.results):
                self.get_logger().info(f"{what} (confirmed)")
                return True
            future = None
        if future is None or now - sent_at > 2.0:
            self.requests[key] = (send(), now)
        return False

    def _set_belt(self):
        return self.belt_params.call_async(
            SetParameters.Request(parameters=[
                Parameter(name="speed", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.belt_speed)),
                Parameter(name="stop_at_sensor", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=not self.continuous)),
            ])
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
            self.note = {"kind": "long", "yaw_deg": round(math.degrees(yaw), 1)}
            return [name]
        if kind == "pair":
            label2 = self.rng.choice(list(layout.GAZEBO_COLORS))
            a, b = f"pair_{n:04d}a_{label}", f"pair_{n:04d}b_{label2}"
            # The belt never waits, and a spawned model appears about 0.2 s after the call: at 10 cm/s
            # the two cubes came out 2 cm apart along the belt and stacks fell over. So the simulation
            # clock is paused while the pair is put down (no simulated time passes, the belt, the robot
            # and every timestamp are untouched), one call after the other.
            cube_a, cube_b = self._cube(color), self._cube(layout.GAZEBO_COLORS[label2])
            if self.rng.random() < 0.5:  # side by side across the belt, touching
                pose_a, pose_b = (x - 0.021, y, z), (x + 0.021, y, z)
                how = "side by side"
            else:  # stacked
                pose_a, pose_b = (x, y, z), (x, y, z + CUBE_SIZE + 0.002)
                how = "stacked"
            if self.pause_cli.service_is_ready() and self.unpause_cli.service_is_ready():
                self.pending = self.pause_cli.call_async(EmptySrv.Request())
                self.steps += [lambda: self._spawn(a, cube_a, *pose_a), lambda: self._spawn(b, cube_b, *pose_b),
                               lambda: self.unpause_cli.call_async(EmptySrv.Request())]
            else:
                self.get_logger().warning("no /pause_physics service: the pair will come out staggered")
                self._spawn(a, cube_a, *pose_a)
                self.pending = self._spawn(b, cube_b, *pose_b)
            self.get_logger().info(f"new items {a} + {b} ({how})")
            self.note = {"kind": "pair", "how": how}
            return [a, b]
        if kind == "unknown":
            name = f"unk_{n:04d}_purple"
            self.pending = self._spawn(name, self._cube(layout.UNKNOWN_COLOR), x, y, z)
            self.get_logger().info(f"new item {name} (unknown)")
            self.note = {"kind": "unknown"}
            return [name]
        name = f"item_{n:04d}_{label}"
        self.pending = self._spawn(name, self._cube(color), x, y, z)
        self.get_logger().info(f"new item {name}")
        self.note = {"kind": "normal"}
        return [name]

    def _tick(self):
        if not self.spawn_cli.service_is_ready() or self.encoder is None:
            return
        if self.pending is not None and not self.pending.done():
            return
        self.pending = None
        if self.steps:  # rest of a pair: second cube, then let the clock run again
            self.pending = self.steps.pop(0)()
            return

        if not self.bins_done:
            for label, (x, y) in layout.BINS.items():
                sdf = self.bin_sdf.replace("Gazebo/Red", f"Gazebo/{layout.GAZEBO_COLORS[label]}")
                self._spawn(f"bin_{label}", sdf, x, y, 0.0)
            self._spawn("bin_hold", self.bin_sdf.replace("Gazebo/Red", "Gazebo/Grey"), *layout.REJECT, 0.0)
            self.bins_done = True
            self.get_logger().info(f"bins spawned: {', '.join(layout.BINS)}, hold")
            return
        if not self.faults_set:  # no item is fed before the gripper's fault rates are in force
            if self.gripper_params.service_is_ready():
                self.faults_set = self._confirm(
                    "gripper", self._set_gripper_faults,
                    f"gripper faults: slip {layout.SLIP_RATE:.0%}, drop {layout.DROP_RATE:.0%}, seed {self.seed}")
            return

        if not self.belt_set:
            if self.belt_params.service_is_ready():
                self.belt_set = self._confirm(
                    "belt", self._set_belt,
                    f"belt: {self.belt_speed:.3f} m/s, item every {self.item_spacing:.2f} m, "
                    f"{'never stops' if self.continuous else 'stops at the light curtain'}")
            return

        if not self.started or (self.feed_limit > 0 and self.count >= self.feed_limit):
            return
        if self.last_item_at is not None and self.encoder - self.last_item_at < self.item_spacing:
            return

        if len(self.items) >= self.max_items:
            # one call per tick: delete first, spawn on a later tick
            old = self.items.popleft()
            self.pending = self.delete_cli.call_async(DeleteEntity.Request(name=old))
            return

        names = self._feed()
        self.items.extend(names)
        self.manifest.append({"slot": self.count, "names": names, "encoder": self.encoder, **self.note})
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
    parser.add_argument("--feed-limit", type=int, default=0, help="stop feeding after N feeds (0 = endless)")
    parser.add_argument("--wait-start", default="false", help="true: feed only after a message on /feeder/start")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = ConveyorScene(
        args.max_items, args.seed, args.faults.lower() == "true",
        args.continuous.lower() == "true", args.belt_speed, args.item_spacing,
        args.feed_limit, args.wait_start.lower() == "true",
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
