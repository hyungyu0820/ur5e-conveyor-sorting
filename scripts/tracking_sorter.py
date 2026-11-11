#!/usr/bin/env python3
"""Sort items from a belt that never stops (conveyor tracking).

Start the cell first:  bash run_sim.sh ur5e scene:=conveyor continuous:=true
Then:                  ros2 run ur_pick_place tracking_sorter.py [--cycles N]

How it works:
  1. The fixed belt camera finds items upstream. Each item gets one number, its belt coordinate
     s = y - encoder, which stays constant while the belt carries it. Then y(t) = s + encoder(t).
  2. The arm goes to where the item will be when it arrives, descends while following the belt,
     closes the gripper and grasps without the belt slowing down.
  3. It drops the item into the bin for its label and goes straight to the next item.

Moves respect joint speed and acceleration limits (--joint-speed, --joint-accel), so the measured
cycle time means something even though the simulated joints could follow any command.
Results are written to ~/ros2_ws/log/sim/tracking_result.json.
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_detector  # noqa: E402
import conveyor_layout as layout  # noqa: E402
import ur_kinematics as kin  # noqa: E402
from conveyor_sorter import DROP_GRID, seed_towards  # noqa: E402
from pick_place import GRASP_Z, GRIPPER_CLOSED, GRIPPER_OPEN, YAW, PickPlace, lock_arm, to_duration  # noqa: E402

ITEM_TOP_Z = layout.BELT_TOP + 0.04  # the camera looks at the top face of a cube
TRAVEL_Z = GRASP_Z + 0.10   # height for moves, also the height above an item before descending
MIN_HITS = 3                # camera sightings before an item is trusted
SAME_ITEM = 0.03            # [m] sightings closer than this on the belt are the same item
ENTRY_Y = -0.50             # items start here; anything new further down is the arm or what it carries
EDGE_MARGIN = 35            # [px] an item cut off by the image border gives a wrong centre
Y_MIN, Y_MAX = -0.15, 0.25  # stretch of the belt where the arm picks
READY = (layout.BELT_X, Y_MIN, TRAVEL_Z)
DT = 0.05                   # [s] spacing of trajectory points
PLAN_MARGIN = 0.25          # [s] planning and goal sending happen before the trajectory starts
DESCEND_TIME = 0.5
CLOSE_TIME = 0.35           # gripper closing, while following the item
HOLD_TIME = 0.15            # keep following after the grasp is triggered
LIFT_TIME = 0.4
RELEASE_TIME = 0.2


@dataclass
class Track:
    id: int
    label: str
    s: float        # belt coordinate, y - encoder
    x: float
    hits: int = 1
    attempts: int = 0       # grasp attempts on this item
    grasp_yaw: float = YAW  # tool yaw of the grasp that holds it
    first_seen: float = 0.0
    samples: list = field(default_factory=list)  # (time, y) from the camera

    def camera_speed(self):
        """Belt speed from this item's motion in the images, independent of the encoder."""
        if len(self.samples) < 5:
            return None
        t, y = zip(*self.samples)
        return float(np.polyfit(t, y, 1)[0])


def smooth(u):
    """0 -> 1 with zero speed at both ends."""
    return u * u * (3 - 2 * u)


def trapezoid(distance, v, a):
    """(duration, position function) of a rest-to-rest move under speed and acceleration limits."""
    d = abs(distance)
    if d < 1e-9:
        return 0.0, lambda t: 0.0
    if d < v * v / a:  # never reaches top speed
        t_acc = math.sqrt(d / a)
        total, v_top = 2 * t_acc, a * t_acc
    else:
        t_acc = v / a
        total, v_top = d / v + t_acc, v

    def s(t):
        t = min(max(t, 0.0), total)
        if t < t_acc:
            return 0.5 * a * t * t
        if t <= total - t_acc:
            return 0.5 * a * t_acc * t_acc + v_top * (t - t_acc)
        return d - 0.5 * a * (total - t) ** 2

    return total, s


class TrackingSorter(PickPlace):
    def __init__(self, v_max, a_max, plan_margin, out_path="~/ros2_ws/log/sim/tracking_result.json"):
        super().__init__()
        self.plan_margin = plan_margin
        self.out_path = out_path
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.v_max, self.a_max = v_max, a_max
        self.encoder = deque(maxlen=200)  # (time, position)
        self.belt_speed = 0.0
        self.tracks = []
        self.next_id = 0
        self.placed = {}
        self.results = []
        self.last_grasp = ""
        self.missed = 0
        self.events = []     # what the robot did and noticed, with sim time (for the exception experiments)
        self.feeder = None   # last /feeder/status, None when the feeder does not report
        self.start_pub = self.create_publisher(Empty, "/feeder/start", 1)
        self.create_subscription(String, "/feeder/status", self._on_feeder, 10)
        w, _ = layout.BELT_CAMERA_SIZE
        self.focal = (w / 2) / math.tan(layout.BELT_CAMERA_HFOV / 2)
        self.create_subscription(JointState, "/conveyor/encoder", self._on_encoder, 50)
        self.create_subscription(Image, "/belt_camera/image_raw", self._on_image, qos_profile_sensor_data)

    # ---------- time ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def sleep(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def log_event(self, event, track=None, **more):
        entry = {"t": round(self.now(), 3), "event": event, **more}
        if track is not None:
            entry.update(track=track.id, label=track.label)
        self.events.append(entry)
        return entry

    # ---------- belt ----------
    def _on_feeder(self, msg):
        self.feeder = json.loads(msg.data)

    def all_fed_and_gone(self):
        """The feeder has finished and its last item has left the pick zone."""
        f = self.feeder
        if not f or not f["done"] or f["last_item_at"] is None or not self.encoder:
            return False
        return self.encoder[-1][1] > f["last_item_at"] + (Y_MAX - layout.SPAWN_Y) + 0.10

    def _on_encoder(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.encoder.append((t, msg.position[0]))
        self.belt_speed = msg.velocity[0]

    def encoder_at(self, t):
        """Belt travel at time t: interpolated from the history, or predicted for the future."""
        times = [e[0] for e in self.encoder]
        if t >= times[-1]:
            return self.encoder[-1][1] + self.belt_speed * (t - times[-1])
        return float(np.interp(t, times, [e[1] for e in self.encoder]))

    def item_y(self, track, t):
        return track.s + self.encoder_at(t)

    # ---------- vision ----------
    def _on_image(self, msg):
        if not self.encoder:
            return
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        rgb = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        enc = self.encoder_at(t)
        cx, cy, cz = layout.BELT_CAMERA
        scale = (cz - ITEM_TOP_Z) / self.focal
        for d in self.detect(rgb):
            if not EDGE_MARGIN < d.u < msg.width - EDGE_MARGIN:
                continue
            y = cy - (d.u - msg.width / 2) * scale
            x = cx - (d.v - msg.height / 2) * scale
            if abs(x - layout.BELT_X) < 0.12:
                self._update_tracks(d.label, x, y, y - enc, t)

    def detect(self, rgb):
        return color_detector.detect(rgb)

    def _update_tracks(self, label, x, y, s, t):
        for tr in self.tracks:
            if tr.label == label and abs(tr.s - s) < SAME_ITEM:
                tr.s += (s - tr.s) / (tr.hits + 1)  # running mean
                tr.x += (x - tr.x) / (tr.hits + 1)
                tr.hits += 1
                tr.samples.append((t, y))
                return
        if y > ENTRY_Y:
            return
        self.tracks.append(Track(self.next_id, label, s, x, first_seen=t, samples=[(t, y)]))
        self.get_logger().info(f"new item #{self.next_id}: {label} at y={y:.3f}")
        self.next_id += 1

    def next_track(self):
        ready = [tr for tr in self.tracks if tr.hits >= MIN_HITS]
        return max(ready, key=lambda tr: tr.s) if ready else None  # furthest along the belt

    # ---------- motion ----------
    def ik_at(self, x, y, z, seed, yaw=YAW):
        target = self.target(x, y, z)
        if yaw != YAW:
            target[:3, :3] = kin.rot_z_down(yaw)
        q = kin.ik(target, seed)
        if q is None:
            raise RuntimeError(f"no IK solution for ({x:.3f}, {y:.3f}, {z:.3f})")
        return q

    def move_time(self, q0, q1):
        return max(trapezoid(d, self.v_max, self.a_max)[0] for d in np.asarray(q1) - q0)

    def joint_move(self, q0, q1, t0=0.0):
        """Time-stamped points of a synchronised joint-space move under the limits."""
        q0, q1 = np.asarray(q0), np.asarray(q1)
        total = self.move_time(q0, q1)
        if total < 1e-6:
            return [], t0
        d = q1 - q0
        lead = int(np.argmax([trapezoid(x, self.v_max, self.a_max)[0] for x in d]))
        _, s = trapezoid(d[lead], self.v_max, self.a_max)
        n = max(int(total / DT), 1)
        return [(t0 + total * i / n, q0 + d * s(total * i / n) / abs(d[lead])) for i in range(1, n + 1)], t0 + total

    def send(self, points, start, events=()):
        """Run a time-stamped trajectory starting at sim time `start`, firing (time, fn) events on the way."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = kin.JOINT_NAMES
        goal.trajectory.header.stamp = Time(sec=int(start), nanosec=int((start % 1) * 1e9))
        for t, q in points:
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=list(map(float, q)), time_from_start=to_duration(t))
            )
        handle_future = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle_future)
        handle = handle_future.result()
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        result_future = handle.get_result_async()
        pending = sorted(events, key=lambda e: e[0])
        while not result_future.done() or pending:
            rclpy.spin_once(self, timeout_sec=0.005)
            while pending and self.now() >= start + pending[0][0]:
                pending.pop(0)[1]()
        result = result_future.result().result
        if result.error_code != 0:
            raise RuntimeError(f"trajectory failed: {result.error_string}")
        for _ in range(5):  # let /joint_states catch up
            rclpy.spin_once(self, timeout_sec=0.02)

    def trigger(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ---------- task ----------
    def seed_for(self, x, y, yaw):
        """IK start for a tool at (x, y) turned to `yaw`: wrist_3 turns against the world yaw (tool z points down)."""
        q = seed_towards(x, y)
        q[5] -= yaw - YAW
        return q

    def intercept(self, track, above_z=TRAVEL_Z, yaw=YAW):
        """Where and when the arm can meet the item: (start, y, q above the item, move time)."""
        start = self.now() + self.plan_margin
        q0 = self.q.copy()
        duration = 1.0
        y, q = 0.0, q0
        for _ in range(3):
            y = self.item_y(track, start + duration)
            # seed from the belt side: from some bin spots the numeric IK would flip the arm over
            q = self.ik_at(track.x, y, above_z, self.seed_for(track.x, y, yaw), yaw)
            duration = max(self.move_time(q0, q), 0.3)
        return start, y, q, duration

    def on_pick_planned(self, track, start, t_above):
        """Hook: the pick trajectory starts at sim time `start` and is above the item `t_above` later."""

    def pick(self, track, yaw=YAW, above=TRAVEL_Z - GRASP_Z, descend=DESCEND_TIME, open_first=False, grasp_z=GRASP_Z):
        """Return 'ok', 'wait' (too early), 'missed' (too late) or 'fail:<reason>'.

        yaw, above, descend: a grasp turned about the vertical or started from lower than the travel
        height (re-grasps). open_first: the gripper is still closed from a failed grasp.
        grasp_z: tool height of the grasp, higher than usual for an item lying on another one.
        """
        above_z = grasp_z + above
        start, y, q_above, move = self.intercept(track, above_z, yaw)
        if y < Y_MIN:
            return "wait"
        follow = descend + CLOSE_TIME + HOLD_TIME
        if y + self.belt_speed * follow > Y_MAX:
            return "missed"

        points, t = self.joint_move(self.q, q_above)
        if not points:
            points, t = [(0.1, q_above)], 0.1
        t_above = t
        seed = q_above
        while t < t_above + follow:  # descend and keep following the item
            t += DT
            z = above_z + (grasp_z - above_z) * smooth(min((t - t_above) / descend, 1.0))
            seed = self.ik_at(track.x, self.item_y(track, start + t), z, seed, yaw)
            points.append((t, seed))

        grasp = {}

        def close():
            self.gripper_pub.publish(self._gripper_msg(GRIPPER_CLOSED))

        def attach():
            grasp["res"] = self.trigger(self.attach_cli)
            self.get_logger().info(f"grasp: {grasp['res'].message}")

        track.attempts += 1
        self.on_pick_planned(track, start, t_above)
        self.log_event("pick_start", track, attempt=track.attempts, t_start=round(start, 3),
                       y_above=round(y, 3), yaw=round(yaw, 3))
        events = [(t_above + descend, close), (t_above + descend + CLOSE_TIME, attach)]
        if open_first:
            events.insert(0, (0.0, lambda: self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))))
        self.send(points, start, events)
        res = grasp["res"]
        self.last_grasp = res.message
        track.grasp_yaw = yaw
        # holding = what the gripper's object sensor says; truth = the simulator's reason, logged for
        # grading only (a sorter must not decide on it)
        self.log_event("grasp", track, attempt=track.attempts, holding=bool(res.success), truth=res.message)
        return "ok" if res.success else res.message

    def _gripper_msg(self, travel):
        return Float64MultiArray(data=[travel, travel])

    def lift_points(self, x, y, yaw=YAW):
        """Straight up to the travel height."""
        pts, seed = [], self.q.copy()
        n = int(LIFT_TIME / DT)
        for i in range(1, n + 1):
            u = i / n
            seed = self.ik_at(x, y, GRASP_Z + (TRAVEL_Z - GRASP_Z) * smooth(u), seed, yaw)
            pts.append((LIFT_TIME * u, seed))
        return pts, seed

    def drop_spot(self, track):
        """Where to let go of the item: the next free cell of its bin."""
        bx, by = layout.BINS.get(track.label, layout.REJECT)
        dx, dy = DROP_GRID[self.placed.get(track.label, 0) % len(DROP_GRID)]
        return bx + dx, by + dy

    def place(self, track):
        tool = kin.fk(self.q)[:3, 3]
        lift, q_top = self.lift_points(tool[0], tool[1], track.grasp_yaw)
        x, y = self.drop_spot(track)
        q_bin = self.ik_at(x, y, TRAVEL_Z, seed_towards(x, y))
        transport, _ = self.joint_move(q_top, q_bin, LIFT_TIME)
        self.send(lift + transport, self.now() + self.plan_margin)
        self.release(track)

    def release(self, track):
        self.trigger(self.detach_cli)
        self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))
        self.sleep(RELEASE_TIME)
        self.placed[track.label] = self.placed.get(track.label, 0) + 1
        self.log_event("released", track, bin=track.label if track.label in layout.BINS else "hold")

    def recover_empty(self, yaw=YAW):
        """The grasp failed: open and rise, the arm is free for the next item."""
        self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))
        tool = kin.fk(self.q)[:3, 3]
        lift, _ = self.lift_points(tool[0], tool[1], yaw)
        self.send(lift, self.now() + self.plan_margin)

    def go_ready(self):
        q = self.ik_at(*READY, seed_towards(READY[0], READY[1]))
        points, _ = self.joint_move(self.q, q)
        if points:
            self.send(points, self.now() + self.plan_margin)

    def report(self, sim_start):
        elapsed = self.now() - sim_start
        ok = [r for r in self.results if r["result"] == "ok"]
        cycles = [b["t_done"] - a["t_done"] for a, b in zip(ok, ok[1:])]
        speeds = [r["camera_speed"] for r in self.results if r["camera_speed"]]
        out = {
            "belt_speed_encoder": self.belt_speed,
            "belt_speed_camera_mean": float(np.mean(speeds)) if speeds else None,
            "picked": len(ok),
            "failed": [r for r in self.results if r["result"] != "ok"],
            "missed": self.missed,
            "placed": self.placed,
            "label_correct": sum(r["label_correct"] for r in self.results),
            "mean_cycle_s": float(np.mean(cycles)) if cycles else None,
            "max_cycle_s": float(np.max(cycles)) if cycles else None,
            "sim_elapsed_s": elapsed,
            "sim_start": sim_start,
            "items": self.results,
            "events": self.events,
        }
        path = os.path.expanduser(self.out_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(out, f, indent=1)
        summary = {k: v for k, v in out.items() if k not in ("items", "failed", "events")}
        self.get_logger().info(f"done: {summary}, failed {len(out['failed'])}, wrote {path}")

    def after_grasp(self, track, result):
        """After a grasp attempt on `track`: sort it, or give the item up (the behaviour without recovery)."""
        self.tracks.remove(track)
        if result == "ok":
            self.place(track)
        else:
            self.recover_empty()
            self.log_event("gave_up", track, why="grasp failed, no recovery")
        self.record(track, result)

    def record(self, track, result):
        # the plugin names what it really grasped (item_0007_red): check the camera's colour against it
        entry = {
            "id": track.id, "label": track.label, "result": result, "t_done": self.now(),
            "grasped": self.last_grasp,
            "label_correct": result == "ok" and f"_{track.label}::" in self.last_grasp,
            "camera_speed": track.camera_speed(), "camera_hits": track.hits, "attempts": track.attempts,
        }
        self.results.append(entry)
        self.get_logger().info(f"item #{track.id} {track.label}: {result}, sorted so far {self.placed}")

    def run(self, cycles, until_fed=False):
        self.wait_ready()
        self.get_logger().info("waiting for the belt encoder and the sim clock ...")
        end = time.time() + 30.0
        while len(self.encoder) < 2 or self.now() == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > end:
                raise RuntimeError("벨트 엔코더나 시뮬레이션 시계가 없습니다. scene:=conveyor로 실행했는지 확인하세요.")
        self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))
        self.go_ready()
        sim_start = self.now()
        self.start_pub.publish(Empty())  # a feeder started with wait_start:=true feeds from now on
        while rclpy.ok() and (cycles == 0 or len(self.results) < cycles):
            if self.feeder and self.feeder["waiting_for_start"]:
                self.start_pub.publish(Empty())
            track = self.next_track()
            if track is None:
                if until_fed and self.all_fed_and_gone():
                    break
                rclpy.spin_once(self, timeout_sec=0.02)
                continue
            result = self.pick(track)
            if result == "wait":
                rclpy.spin_once(self, timeout_sec=0.02)
                continue
            if result == "missed":
                self.tracks.remove(track)
                self.missed += 1
                self.log_event("missed", track, why="past the pick zone before the arm was free")
                self.get_logger().warning(f"item #{track.id} went past the pick zone")
                self.go_ready()
                continue
            self.after_grasp(track, result)
            if self.next_track() is None:
                self.go_ready()
        self.report(sim_start)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cycles", type=int, default=0, help="stop after N items (0 = forever)")
    parser.add_argument("--joint-speed", type=float, default=math.pi, help="joint speed limit [rad/s]")
    parser.add_argument("--joint-accel", type=float, default=3.0, help="joint acceleration limit [rad/s^2]")
    parser.add_argument("--plan-margin", type=float, default=PLAN_MARGIN,
                        help="sim seconds kept for planning before a trajectory starts; raise it in faster-than-real-time runs")
    parser.add_argument("--until-fed", action="store_true",
                        help="stop when a feeder started with feed_limit:=N has fed everything and the last item is gone")
    parser.add_argument("--out", default="~/ros2_ws/log/sim/tracking_result.json", help="result file")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    arm_lock = lock_arm()  # noqa: F841
    rclpy.init()
    node = TrackingSorter(args.joint_speed, args.joint_accel, args.plan_margin, args.out)
    try:
        node.run(args.cycles, args.until_fed)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(str(e))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
