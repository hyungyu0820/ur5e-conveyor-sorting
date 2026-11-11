#!/usr/bin/env python3
"""Record the simulator's truth during an exception-handling run; the sorter never sees any of it.

Start it before the sorter and stop it after (SIGTERM or Ctrl+C):
  ros2 run ur_pick_place exception_metrics.py --out ~/ros2_ws/log/sim/exception/run/truth.json

Written to the file:
  items         where every fed object ended up (Gazebo position and the place that is: a bin, the
                hold bin, the floor, the belt)
  grasp_events  every grasp attempt, attach, drop and release with the true reason (from the gripper plugin)
  feeder        what was fed and in which order (kind, names, rotation of long boxes, ...)
tools/exception_report.py turns this plus the sorter's own log into the per-item outcome table.
"""
import argparse
import json
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from score_bins import where  # noqa: E402

ZONE_START = -0.15  # tracking_sorter.Y_MIN


class Recorder(Node):
    def __init__(self):
        super().__init__("exception_metrics")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.last = {}        # name -> [x, y, z]
        self.first_seen = {}  # name -> sim time
        self.last_seen = {}
        self.at_spawn = {}    # name -> first position seen
        self.at_zone = {}     # name -> position when it reached the start of the pick zone
        self.grasp_events = []
        self.feeder = None
        self.create_subscription(String, "/sim_truth/items", self._on_items, 10)
        self.create_subscription(String, "/sim_truth/grasp_events", self._on_grasp, 200)
        self.create_subscription(String, "/feeder/status", self._on_feeder, 10)

    def _on_items(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for name, pos in json.loads(msg.data).items():
            self.last[name] = pos
            self.first_seen.setdefault(name, now)
            self.last_seen[name] = now
            self.at_spawn.setdefault(name, pos)
            if name not in self.at_zone and pos[1] >= ZONE_START:
                self.at_zone[name] = pos

    def _on_grasp(self, msg):
        self.grasp_events.append(json.loads(msg.data))

    def _on_feeder(self, msg):
        self.feeder = json.loads(msg.data)

    def write(self, path):
        items = {
            name: {"pos": [round(v, 4) for v in pos], "place": where(*pos),
                   "first_seen": self.first_seen[name], "last_seen": self.last_seen[name],
                   "at_spawn": [round(v, 4) for v in self.at_spawn[name]],
                   "at_zone": [round(v, 4) for v in self.at_zone[name]] if name in self.at_zone else None}
            for name, pos in sorted(self.last.items()) if not name.startswith("bin_")
        }
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"items": items, "grasp_events": self.grasp_events, "feeder": self.feeder}, f, indent=1)
        print(f"wrote {path}: {len(items)} objects, {len(self.grasp_events)} grasp events", flush=True)


def stop(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="~/ros2_ws/log/sim/exception/truth.json")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # SIGTERM can arrive while a message is being taken: rclpy then reports a RuntimeError instead
        # of the KeyboardInterrupt, and the truth of the whole run would be lost
        pass
    node.write(args.out)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
