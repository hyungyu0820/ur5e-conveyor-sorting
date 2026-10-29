#!/usr/bin/env python3
"""Sort items arriving on the conveyor into colour bins, forever (Ctrl+C to stop).

Start the cell first:  bash run_sim.sh ur5e scene:=conveyor
Then:                  ros2 run ur_pick_place conveyor_sorter.py [--cycles N]

Cycle:
  1. wait above the light curtain until the belt stops with an item at the sensor
  2. wrist camera -> color_detector.detect() -> label + pixel -> depth -> world (x, y)
  3. pick, lift, restart the belt, drop the item into the bin for its label
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_detector  # noqa: E402
import conveyor_layout as layout  # noqa: E402
import ur_kinematics as kin  # noqa: E402
from pick_place import GRASP_Z, GRIPPER_CLOSED, GRIPPER_OPEN, HOME, PickPlace  # noqa: E402

# the camera sits 5.5 cm to the side of the TCP (tool -y in the world at YAW = -pi/2),
# so this TCP pose centres the light curtain in the image, ~30 cm above the belt
CAMERA_OFFSET_Y = 0.055
WATCH = (layout.BELT_X, layout.SENSOR_Y + CAMERA_OFFSET_Y, 0.45)
TRAVEL_Z = 0.40       # height for moves between the belt and the bins
DROP_Z = 0.30         # TCP height when releasing above a bin
MAX_PICK_ERROR = 0.08  # ignore detections this far from the sensor (next item in the queue)
# drop spots inside a bin (3 x 3 grid), used in turn so items do not pile up into a tower
DROP_GRID = [(dx, dy) for dx in (-0.06, 0.0, 0.06) for dy in (-0.06, 0.0, 0.06)]


def quat_to_matrix(t):
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def seed_towards(x, y):
    """HOME rotated about the base so the TCP faces (x, y): a good IK start for far moves."""
    home_tcp = kin.fk(HOME)[:3, 3]
    q = np.array(HOME)
    q[0] += np.arctan2(y, x) - np.arctan2(home_tcp[1], home_tcp[0])
    q[5] += q[0] - HOME[0]  # keep the tool yaw fixed in the world (tool z points down)
    return q


class ConveyorSorter(PickPlace):
    def __init__(self, detect):
        super().__init__()
        self.detect = detect
        self.rgb = self.depth = self.K = None
        self.rgb_time = 0.0
        self.belt_running = None
        self.item_at_sensor = False
        self.power_cli = self.create_client(SetBool, "/conveyor/power")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/wrist_camera/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/wrist_camera/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(Bool, "/conveyor/running", self._on_running, 10)
        self.create_subscription(Bool, "/conveyor/object_at_sensor", self._on_sensor, 10)
        self.stats = {}

    # ---------- callbacks ----------
    def _on_rgb(self, msg):
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        self.rgb = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            self.rgb = self.rgb[:, :, ::-1]
        self.rgb_time = time.time()

    def _on_depth(self, msg):
        data = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)
        self.depth = data[:, : msg.width]

    def _on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def _on_running(self, msg):
        self.belt_running = msg.data

    def _on_sensor(self, msg):
        self.item_at_sensor = msg.data

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------- conveyor ----------
    def belt(self, on):
        self.power_cli.wait_for_service()
        future = self.power_cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future)

    def wait_for_item(self):
        self.get_logger().info("waiting for an item at the sensor ...")
        while not (self.belt_running is False and self.item_at_sensor):
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---------- vision ----------
    def locate_item(self):
        """Return (label, x, y) of the item at the sensor in the world frame, or None."""
        # use an image taken after the arm and the item have settled
        t0 = time.time()
        while self.rgb_time < t0 + 0.3 or self.depth is None or self.K is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        rgb, depth, K = self.rgb.copy(), self.depth.copy(), self.K

        tf = self.tf_buffer.lookup_transform("world", "camera_optical_frame", rclpy.time.Time())
        T = quat_to_matrix(tf.transform)

        best = None
        for d in self.detect(rgb):
            u, v = int(round(d.u)), int(round(d.v))
            patch = depth[max(v - 2, 0): v + 3, max(u - 2, 0): u + 3]
            z = float(np.nanmedian(patch))
            if not np.isfinite(z):
                continue
            p_cam = np.array([(d.u - K[0, 2]) * z / K[0, 0], (d.v - K[1, 2]) * z / K[1, 1], z, 1.0])
            x, y, _ = (T @ p_cam)[:3]
            err = np.hypot(x - layout.BELT_X, y - layout.SENSOR_Y)
            if err < MAX_PICK_ERROR and (best is None or err < best[3]):
                best = (d.label, x, y, err)
        return best[:3] if best else None

    # ---------- task ----------
    def to_watch(self):
        self.move_to(*WATCH, duration=3.0, seed=seed_towards(WATCH[0], WATCH[1]))

    def sort_one(self):
        self.wait_for_item()
        found = None
        for _ in range(3):
            found = self.locate_item()
            if found:
                break
        if found is None:
            # nothing recognisable: take it from the sensor position into the reject area
            label, x, y = "unknown", layout.BELT_X, layout.SENSOR_Y - 0.01
        else:
            label, x, y = found
        self.get_logger().info(f"item: {label} at ({x:.3f}, {y:.3f})")

        # pick
        self.move_to(x, y, GRASP_Z + 0.10, duration=1.5)
        self.move_to(x, y, GRASP_Z, duration=1.5, steps=8)
        self.gripper(GRIPPER_CLOSED, wait=0.6)
        if not self.call(self.attach_cli):
            self.gripper(GRIPPER_OPEN, wait=0.3)
            self.move_to(x, y, GRASP_Z + 0.10, duration=1.0, steps=5)
            return
        self.move_to(x, y, TRAVEL_Z, duration=1.5, steps=8)
        self.belt(True)  # item is clear of the light curtain, let the next one come

        # place
        bx, by = layout.BINS.get(label, layout.REJECT)
        dx, dy = DROP_GRID[self.stats.get(label, 0) % len(DROP_GRID)]
        bx, by = bx + dx, by + dy
        self.move_to(bx, by, TRAVEL_Z, duration=3.0, seed=seed_towards(bx, by))
        self.move_to(bx, by, DROP_Z, duration=1.0, steps=5)
        self.call(self.detach_cli)
        self.gripper(GRIPPER_OPEN, wait=0.4)
        self.move_to(bx, by, TRAVEL_Z, duration=1.0, steps=5)
        self.stats[label] = self.stats.get(label, 0) + 1
        self.get_logger().info(f"sorted so far: {self.stats}")

        self.move_to(WATCH[0], WATCH[1], TRAVEL_Z, duration=3.0, seed=seed_towards(WATCH[0], WATCH[1]))
        self.to_watch()

    def run_forever(self, cycles):
        self.wait_ready()
        self.send_joints([HOME], 3.0)
        self.gripper(GRIPPER_OPEN)
        self.to_watch()
        self.belt(True)
        n = 0
        while rclpy.ok() and (cycles == 0 or n < cycles):
            self.sort_one()
            n += 1
        self.get_logger().info(f"done, sorted: {self.stats}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cycles", type=int, default=0, help="stop after N items (0 = forever)")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = ConveyorSorter(color_detector.detect)
    try:
        node.run_forever(args.cycles)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(str(e))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
