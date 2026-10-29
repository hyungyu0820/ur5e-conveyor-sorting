#!/usr/bin/env python3
"""Pick the cube from the table and place it at another spot.

Run after the simulation is up (bash run_sim.sh):
  ros2 run ur_pick_place pick_place.py
  ros2 run ur_pick_place pick_place.py --pick 0.45 -0.25 --place 0.45 0.25
  ros2 run ur_pick_place pick_place.py --object box1 --place 0.55 0.2   # position read from Gazebo

Sequence: home -> open -> above cube -> down -> close + attach -> up
          -> above place -> down -> detach + open -> up -> home
Positions are TCP (point between the fingertips) in the world frame, see launch/ur_gripper_sim.launch.py.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_kinematics as kin  # noqa: E402

HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # TCP above the table, pointing down
TABLE_TOP = 0.20
CUBE_SIZE = 0.04
GRASP_Z = TABLE_TOP + CUBE_SIZE / 2  # TCP at cube centre
APPROACH = 0.12                      # height above the grasp/place point
YAW = -np.pi / 2                     # tool yaw at HOME, keeps wrist_3 near 0

GRIPPER_OPEN = 0.0
# finger travel for a 40 mm gap is 0.025 (gap = 0.09 - 2 * travel); squeeze 1 mm more
GRIPPER_CLOSED = (0.09 - CUBE_SIZE) / 2 + 0.001


def object_position(name):
    """World position of a Gazebo model, via the `gz` command line tool."""
    out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True, text=True, timeout=10).stdout
    values = out.split()
    if len(values) < 3:
        raise RuntimeError(f"object '{name}' not found in Gazebo")
    return [float(v) for v in values[:3]]


def to_duration(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.arm = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.attach_cli = self.create_client(Trigger, "/grasp/attach")
        self.detach_cli = self.create_client(Trigger, "/grasp/detach")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in kin.JOINT_NAMES):
            self.q = np.array([pos[n] for n in kin.JOINT_NAMES])

    def wait_ready(self):
        self.get_logger().info("waiting for controllers and /joint_states ...")
        self.arm.wait_for_server()
        self.attach_cli.wait_for_service()
        self.detach_cli.wait_for_service()
        while self.q is None or self.gripper_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---------- arm ----------
    def send_joints(self, waypoints, duration):
        """Follow a list of joint configurations, evenly spread over `duration` seconds."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = kin.JOINT_NAMES
        n = len(waypoints)
        for i, q in enumerate(waypoints):
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=list(map(float, q)), time_from_start=to_duration(duration * (i + 1) / n))
            )
        handle_future = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle_future)
        handle = handle_future.result()
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != 0:
            raise RuntimeError(f"trajectory failed: {result.error_string}")
        # let /joint_states catch up so the next IK starts from where the arm really is
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

    def target(self, x, y, z):
        T = np.eye(4)
        T[:3, :3] = kin.rot_z_down(YAW)
        T[:3, 3] = [x, y, z]
        return T

    def move_to(self, x, y, z, duration=3.0, steps=1, seed=None):
        """Move the TCP to (x, y, z). steps > 1 gives a straight-line (Cartesian) path.

        seed: joint angles to start the IK from (for far moves, e.g. turning to the other side).
        """
        start = kin.fk(self.q)[:3, 3]
        end = np.array([x, y, z])
        q_seed = self.q if seed is None else seed
        waypoints = []
        for s in np.linspace(0.0, 1.0, steps + 1)[1:]:
            q = kin.ik(self.target(*(start + s * (end - start))), q_seed)
            if q is None:
                raise RuntimeError(f"no IK solution for {np.round(end, 3)}")
            waypoints.append(q)
            q_seed = q
        self.get_logger().info(f"move TCP -> ({x:.3f}, {y:.3f}, {z:.3f})")
        self.send_joints(waypoints, duration)

    # ---------- gripper ----------
    def gripper(self, travel, wait=1.0):
        self.gripper_pub.publish(Float64MultiArray(data=[travel, travel]))
        end = time.time() + wait
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"{client.srv_name}: {res.message}")
        return res.success

    # ---------- task ----------
    def run(self, pick, place):
        """pick: (x, y) on the table or (x, y, z) of the cube centre; place: (x, y) on the table."""
        pick_z = pick[2] if len(pick) > 2 else GRASP_Z
        self.wait_ready()
        self.get_logger().info("home")
        self.send_joints([HOME], 3.0)
        self.gripper(GRIPPER_OPEN)

        # pick
        self.move_to(pick[0], pick[1], pick_z + APPROACH)
        self.move_to(pick[0], pick[1], pick_z, duration=2.0, steps=10)
        self.gripper(GRIPPER_CLOSED)
        if not self.call(self.attach_cli):
            self.get_logger().error("grasp failed, aborting")
            self.gripper(GRIPPER_OPEN)
            self.move_to(pick[0], pick[1], pick_z + APPROACH, duration=2.0, steps=10)
            return False
        self.move_to(pick[0], pick[1], pick_z + APPROACH, duration=2.0, steps=10)

        # place (2 mm above the table so the cube does not get pushed into it)
        self.move_to(place[0], place[1], GRASP_Z + APPROACH, duration=3.0, steps=10)
        self.move_to(place[0], place[1], GRASP_Z + 0.002, duration=2.0, steps=10)
        self.call(self.detach_cli)
        self.gripper(GRIPPER_OPEN)
        self.move_to(place[0], place[1], GRASP_Z + APPROACH, duration=2.0, steps=10)

        self.send_joints([HOME], 3.0)
        self.get_logger().info("pick and place done")
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pick", nargs=2, type=float, default=[0.45, -0.25], metavar=("X", "Y"))
    parser.add_argument("--place", nargs=2, type=float, default=[0.45, 0.25], metavar=("X", "Y"))
    parser.add_argument("--object", help="Gazebo model name to pick (overrides --pick)")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = PickPlace()
    try:
        pick = args.pick
        if args.object:
            pick = object_position(args.object)
            node.get_logger().info(f"{args.object} is at ({pick[0]:.3f}, {pick[1]:.3f}, {pick[2]:.3f})")
        ok = node.run(pick, args.place)
    except RuntimeError as e:
        node.get_logger().error(str(e))
        ok = False
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
