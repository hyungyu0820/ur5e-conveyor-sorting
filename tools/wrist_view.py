#!/usr/bin/env python3
"""Photograph what the wrist camera sees on the way down to an item stopped at the light curtain.

Needs: run_sim.sh ur5e scene:=conveyor (belt stops at the curtain) with the wrist camera on.
  python3 tools/wrist_view.py <name>   -> ~/ros2_ws/log/sim/wrist_<name>.png plus a table
"""
import json
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import color_detector  # noqa: E402
import conveyor_layout as layout  # noqa: E402
from conveyor_sorter import seed_towards  # noqa: E402
from pick_place import GRASP_Z, GRIPPER_CLOSED, GRIPPER_OPEN, PickPlace, lock_arm  # noqa: E402

STOPS = [("watch", 0.45, GRIPPER_OPEN), ("above", GRASP_Z + 0.10, GRIPPER_OPEN), ("mid", GRASP_Z + 0.05, GRIPPER_OPEN),
         ("grasp", GRASP_Z, GRIPPER_OPEN), ("closed", GRASP_Z, GRIPPER_CLOSED)]


class Viewer(PickPlace):
    def __init__(self):
        super().__init__()
        self.rgb = None
        self.items = {}
        self.create_subscription(Image, "/wrist_camera/image_raw", self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(String, "/sim_truth/items", lambda m: self.items.update(json.loads(m.data)), 10)

    def _on_rgb(self, msg):
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        self.rgb = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3).copy()

    def settle(self, seconds):
        for _ in range(int(seconds / 0.05)):
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    name = sys.argv[1]
    lock = lock_arm()  # noqa: F841
    rclpy.init()
    node = Viewer()
    node.wait_ready()
    node.gripper(GRIPPER_OPEN)
    def item_at_curtain():
        at = [p for n, p in node.items.items() if n.startswith("item_") and abs(p[1] - layout.SENSOR_Y) < 0.08]
        return at[0] if at else None

    while True:  # the belt stops an item when its centre reaches 2 cm before the light curtain
        node.settle(0.5)
        now = item_at_curtain()
        if now and abs(now[1] - (layout.SENSOR_Y - 0.02)) < 0.004 and node.rgb is not None:
            break
    x, y = now[0], now[1]
    node.move_to(x, y, 0.45, duration=3.0, seed=seed_towards(x, y))
    tiles = []
    print(f"{name}: item at ({x:.3f}, {y:.3f})")
    for label, z, grip in STOPS:
        node.move_to(x, y, z, duration=1.5, steps=5)
        node.gripper(grip, wait=0.6)
        node.settle(1.0)
        rgb = node.rgb.copy()
        found = color_detector.detect(rgb)
        desc = ", ".join(f"{d.label} centre ({d.u:.0f},{d.v:.0f}) area {d.area}" for d in found) or "item not found"
        print(f"  {label:7s} z={z:.2f}: {desc}")
        tile = rgb[:, :, ::-1].copy()
        cv2.imwrite(os.path.expanduser(f"~/ros2_ws/log/sim/wrist_{name}_{label}.png"), tile)
        cv2.putText(tile, f"{name} {label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        tiles.append(cv2.resize(tile, (320, 240)))
    cv2.imwrite(os.path.expanduser(f"~/ros2_ws/log/sim/wrist_{name}.png"), np.hstack(tiles))
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
