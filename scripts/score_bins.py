#!/usr/bin/env python3
"""Grade a sorting run from Gazebo's true object positions; the sorter never sees these.

Start it before the sorter and stop it after (Ctrl+C or SIGTERM):
  ros2 run ur_pick_place score_bins.py

Item names contain the true colour (item_0007_red), so an item counts as correct when it was last
seen inside the bin of that colour. Items the feeder deleted to keep Gazebo fast keep their last
position, so long runs can be graded completely.
"""
import json
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conveyor_layout as layout  # noqa: E402

BIN_HALF = 0.11
BIN_TOP = 0.10


def where(x, y, z):
    if z < BIN_TOP:
        for name, (bx, by) in list(layout.BINS.items()) + [("hold", layout.REJECT)]:
            if abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF:
                return name
        return "floor"
    return "belt" if abs(x - layout.BELT_X) < 0.1 else "in the air"


class Scorer(Node):
    def __init__(self):
        super().__init__("score_bins")
        self.last = {}
        self.create_subscription(String, "/sim_truth/items", self._on_items, 10)

    def _on_items(self, msg):
        self.last.update(json.loads(msg.data))

    def summary(self):
        correct = wrong = 0
        other = {}
        for name, pos in sorted(self.last.items()):
            if not name.startswith("item_"):
                continue
            color = name.rsplit("_", 1)[1]
            place = where(*pos)
            print(f"{name}: {place} ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            if place == color:
                correct += 1
            elif place in layout.BINS:
                wrong += 1
            else:
                other[place] = other.get(place, 0) + 1
        print(f"correct bin {correct}, wrong bin {wrong}, elsewhere {other}", flush=True)


def stop(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = Scorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.summary()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
