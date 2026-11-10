#!/usr/bin/env python3
"""Sort from the belt that never stops, and try to save an item when a grasp goes wrong.

Start the cell first (the wrist camera must be on, mount raised_wide):
  bash run_sim.sh ur5e scene:=conveyor continuous:=true belt_speed:=0.1 faults:=true seed:=1
Then:
  ros2 run ur_pick_place recovery_sorter.py [--cycles N]

Same tracking and motions as tracking_sorter.py. What is added (the robot only uses its own sensors):
  gripper object sensor  after closing: is there something between the fingers? on arrival at the
                         bin: is it still there? (/grasp/holding and the answers of attach/detach)
  wrist camera           one RGB-D frame from above the item at the start of every descent. It is
                         only looked at when the grasp then comes up empty (wrist_check.py).
  belt camera            also reports saturated blobs that are not red/green/blue as "unknown".

Rules after an empty grasp, tried in this order while the item is still inside the pick zone:
  one normal item seen     -> it slipped: grasp again the same way
  long box seen            -> grasp again with the gripper turned across its short side
  two items seen (side by side, or one on the other) -> leave them on the belt, skip their partner track
  nothing seen / unclear / still failing -> ask the diagnosis hook (gemini_diagnosis.py), leave the item
Unknown items go to the hold bin. An item lost on the way to the bin is reported, not counted as sorted.
The belt never waits: whatever is not saved rides on to the end of the line.
"""
import argparse
import json
import math
import os
import sys
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_detector  # noqa: E402
import conveyor_layout as layout  # noqa: E402
import ur_kinematics as kin  # noqa: E402
import wrist_check  # noqa: E402
from gemini_diagnosis import DiagnosisHook  # noqa: E402
from pick_place import GRIPPER_OPEN, YAW, lock_arm  # noqa: E402
from tracking_sorter import PLAN_MARGIN, TrackingSorter  # noqa: E402

MAX_ATTEMPTS = 3        # grasp attempts on one item, the first one included
REGRASP_ABOVE = 0.05    # [m] a re-grasp starts this far above the grasp height (the first grasp: 0.10)
REGRASP_DESCEND = 0.30  # [s]
SAME_YAW = math.radians(15)   # a turn smaller than this is not worth it: grasp again as before
# [s] wrist frames from this long before to this long after "above the item" are kept. The camera only
# delivers a few frames per second here; later than this the tool is too low to see a long box whole.
LOOK_BEFORE, LOOK_AFTER = 0.20, 0.28
PARTNER = 0.07          # [m] tracks this close on the belt belong to the same pile
UNKNOWN_MIN_AREA = 400  # [px] belt camera, a cube top is about 3000


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def unknown_blobs(rgb):
    """Coloured things that are not red, green or blue (belt camera)."""
    img = rgb.astype(np.float32)
    top, low = img.max(axis=2), img.min(axis=2)
    mask = (top > color_detector.MIN_LEVEL) & ((top - low) > 0.5 * top)
    for c in color_detector.CHANNEL.values():
        others = np.delete(img, c, axis=2).max(axis=2)
        mask &= ~((img[:, :, c] > color_detector.MIN_LEVEL) & (img[:, :, c] > color_detector.DOMINANCE * others))
    # blended pixels on the outline of a red/green/blue item also pass the test: drop thin rings
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    return [
        color_detector.Detection("unknown", float(centroids[i][0]), float(centroids[i][1]), int(stats[i, cv2.CC_STAT_AREA]))
        for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= UNKNOWN_MIN_AREA
    ]


class RecoverySorter(TrackingSorter):
    def __init__(self, v_max, a_max, plan_margin, out_path, run_dir, save_frames="failed"):
        super().__init__(v_max, a_max, plan_margin, out_path)
        self.run_dir = os.path.expanduser(run_dir)
        self.save_frames = save_frames
        self.holding = None
        self.K = None
        self.T_tool_cam = wrist_check.camera_in_tool(os.environ.get("UR_WRIST_CAMERA_MOUNT", "raised_wide"))
        self.joint_log = deque(maxlen=800)
        self.window = None          # (from, to, aim) sim times of the wrist frames to keep
        self.frames = {"rgb": {}, "depth": {}}
        self.looked = {}            # track id -> (features, rgb)
        self.long_placed = {}
        self.wrist_stamps = {"rgb": [], "depth": []}  # every frame received, for the frame-rate statistic
        self.look_stats = {"looks": 0, "no_frame": 0, "frames_in_window": []}
        self.hook = DiagnosisHook(os.path.join(self.run_dir, "gemini"), log=self.get_logger().info)
        self.create_subscription(Bool, "/grasp/holding", self._on_holding, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", lambda m: self._on_wrist("rgb", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/wrist_camera/depth/image_raw", lambda m: self._on_wrist("depth", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/wrist_camera/camera_info", self._on_info, qos_profile_sensor_data)

    # ---------- sensors ----------
    def _on_joints(self, msg):
        super()._on_joints(msg)
        if self.q is not None and hasattr(self, "joint_log"):
            self.joint_log.append((stamp_of(msg), self.q))

    def _on_holding(self, msg):
        self.holding = msg.data

    def _on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def _on_wrist(self, kind, msg):
        self.wrist_stamps[kind].append(stamp_of(msg))
        if self.window and self.window[0] <= stamp_of(msg) <= self.window[1]:
            self.frames[kind][round(stamp_of(msg), 4)] = msg

    def detect(self, rgb):
        return color_detector.detect(rgb) + unknown_blobs(rgb)

    # ---------- wrist camera ----------
    def on_pick_planned(self, track, start, t_above):
        if track.attempts == 1:  # the look from the travel height, before the first grasp
            self.window = (start + t_above - LOOK_BEFORE, start + t_above + LOOK_AFTER, start + t_above + 0.03)
            self.frames = {"rgb": {}, "depth": {}}

    def look(self, track):
        """Measure what was under the gripper from the frame taken above the item. (features, rgb) or (None, None)."""
        if track.id in self.looked:
            return self.looked[track.id]
        end = self.now() + 0.15  # frames can lag behind the motion
        while self.now() < end and not (set(self.frames["rgb"]) & set(self.frames["depth"])):
            rclpy.spin_once(self, timeout_sec=0.01)
        # best first: frames from "above the item" on (the whole item is in view), nearest to that moment;
        # earlier ones, taken while the tool was still arriving, can cut a long box short
        aim = self.window[2]
        stamps = sorted(set(self.frames["rgb"]) & set(self.frames["depth"]), key=lambda s: (s < aim - 0.08, abs(s - aim)))
        self.look_stats["looks"] += 1
        self.look_stats["frames_in_window"].append(len(stamps))
        if not stamps or self.K is None or not self.joint_log:
            self.look_stats["no_frame"] += 1
            self.looked[track.id] = (None, None)
            return None, None
        seen = rgb = None
        for i, t in enumerate(stamps):  # the frame nearest to "above the item" first; the others only if it shows nothing
            m = self.frames["rgb"][t]
            image = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
            if m.encoding == "bgr8":
                image = image[:, :, ::-1]
            image = np.ascontiguousarray(image)
            d = self.frames["depth"][t]
            depth = np.frombuffer(d.data, np.float32).reshape(d.height, d.step // 4)[:, : d.width]
            tq, q = min(self.joint_log, key=lambda e: abs(e[0] - t))
            expect = (track.x, self.item_y(track, t))
            if self.save_frames != "none":
                os.makedirs(os.path.join(self.run_dir, "frames"), exist_ok=True)
                np.savez(os.path.join(self.run_dir, "frames", f"track{track.id:03d}_{i}.npz"), rgb=image, depth=depth, K=self.K,
                         q=q, expect=np.array(expect), stamp=t, label=track.label, after_above=t - self.window[2] + 0.03)
            if seen is None or not seen.get("seen"):
                seen = wrist_check.measure(image, depth, self.K, kin.fk(q, tcp=False) @ self.T_tool_cam, expect, self.T_tool_cam)
                seen.update(stamp=t, frames=len(stamps), joint_dt=round(tq - t, 4), kind=wrist_check.classify(seen),
                            after_above=round(t - self.window[2] + 0.03, 3))
                rgb = image
            if seen.get("seen") and self.save_frames != "all":
                break
        self.looked[track.id] = (seen, rgb)
        return seen, rgb

    # ---------- rules ----------
    def decide(self, track):
        """What to do after an empty grasp, from what the wrist camera saw. Returns a dict with 'action'."""
        seen, rgb = self.look(track)
        kind = seen["kind"] if seen else "no_frame"
        brief = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in (seen or {}).items()
                 if k in ("kind", "height", "length", "width", "angle", "colours", "frames")}
        if kind == "single":
            if track.attempts < MAX_ATTEMPTS:
                return {"action": "retry", "kind": kind, "seen": brief, "why": "one normal item under the gripper, it slipped"}
        elif kind == "long":
            if track.attempts < MAX_ATTEMPTS:
                yaw = wrist_check.grasp_yaw_for(seen["angle"], YAW)
                if abs(yaw - track.grasp_yaw) < SAME_YAW:
                    return {"action": "retry", "kind": kind, "seen": brief, "yaw": track.grasp_yaw,
                            "why": "long box already gripped across its short side, it slipped"}
                return {"action": "regrasp_turned", "kind": kind, "seen": brief, "yaw": yaw,
                        "why": "long box lying across the fingers, turn the gripper"}
        elif kind in ("pair", "stacked"):
            return {"action": "skip", "kind": kind, "seen": brief, "why": "two items together, the gripper cannot separate them"}
        # nothing seen, no frame, a shape the rules do not know, or a normal-looking item that keeps failing
        situation = {
            "belt_camera_label": track.label, "grasp_attempts": track.attempts, "gripper_holds_item": False,
            "wrist_camera": brief or "no usable frame",
            "item_position_on_belt_m": round(self.item_y(track, self.now()), 3), "pick_zone_ends_at_m": 0.25,
        }
        why = "grasp keeps failing" if kind in ("single", "long") else f"wrist camera: {kind}"
        return self.hook.ask(rgb, situation, {"action": "skip", "kind": kind, "seen": brief, "why": why})

    def after_grasp(self, track, result):
        self.tracks.remove(track)
        if result != "ok":
            t_detect = self.now()
            self.log_event("exception", track, kind="grasp_empty", attempt=track.attempts)
            result = self.recover(track, t_detect)
        if result == "ok":
            self.place(track)
            if track.label == "unknown":  # second opinion on what it was; it is already in the hold bin
                seen, rgb = self.look(track)
                self.hook.ask(rgb, {"belt_camera_label": "unknown", "gripper_holds_item": True,
                                    "wrist_camera": (seen or {}).get("kind", "no usable frame")},
                              {"action": "hold", "kind": "unknown item", "why": "colour not in the sorting rule"})
            result = "ok" if not getattr(track, "dropped", False) else "dropped"
        elif self.save_frames == "all":
            self.look(track)
        if self.save_frames == "all" and result == "ok":
            self.look(track)
        self.record(track, result)

    def recover(self, track, t_detect):
        """Rule-based recovery after an empty grasp. Returns 'ok' when the item is in the gripper after all."""
        while True:
            plan = self.decide(track)
            self.log_event("decision", track, **{k: v for k, v in plan.items() if k != "seen"}, seen=plan.get("seen"))
            self.get_logger().info(f"item #{track.id} empty grasp -> {plan['action']} ({plan.get('kind')}: {plan.get('why')})")
            if plan["action"] not in ("retry", "regrasp_turned"):
                break
            yaw = plan.get("yaw", track.grasp_yaw)
            try:
                result = self.pick(track, yaw=yaw, above=REGRASP_ABOVE, descend=REGRASP_DESCEND, open_first=True)
            except RuntimeError as e:  # no IK for the turned grasp
                plan = {"action": "skip", "kind": plan.get("kind"), "why": f"re-grasp not reachable: {e}"}
                break
            if result == "ok":
                self.log_event("recovered", track, how=plan["action"], seconds=round(self.now() - t_detect, 3),
                               attempts=track.attempts)
                return "ok"
            if result in ("wait", "missed"):
                plan = {"action": "skip", "kind": plan.get("kind"), "why": "the item left the pick zone before a re-grasp fits"}
                self.log_event("decision", track, **plan)
                break
            self.log_event("exception", track, kind="grasp_empty", attempt=track.attempts)
        if plan.get("kind") in ("pair", "stacked"):
            for other in [tr for tr in self.tracks if abs(tr.s - track.s) < PARTNER]:
                self.tracks.remove(other)
                self.log_event("skipped_partner", other, of=track.id)
        self.recover_empty(track.grasp_yaw)
        self.log_event("gave_up", track, why=plan.get("why", ""), kind=plan.get("kind"),
                       seconds=round(self.now() - t_detect, 3), attempts=track.attempts)
        return f"gave_up:{plan.get('kind')}"

    def drop_spot(self, track):
        seen = self.looked.get(track.id, (None, None))[0]
        if seen and seen.get("kind") == "long":
            # the box arrives lying along y (the tool x axis): it fits the middle row of the 3 x 3 grid only
            bx, by = layout.BINS.get(track.label, layout.REJECT)
            self.long_placed[track.label] = self.long_placed.get(track.label, 0) + 1
            return bx + (-0.06, 0.0, 0.06)[self.long_placed[track.label] % 3], by
        return super().drop_spot(track)

    def release(self, track):
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.01)
        res = self.trigger(self.detach_cli)
        if self.holding is False or not res.success:  # the object sensor says the gripper arrived empty
            self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))
            track.dropped = True
            self.log_event("exception", track, kind="lost_in_transport")
            self.hook.ask(None, {"belt_camera_label": track.label, "gripper_holds_item": False,
                                 "event": "item was in the gripper after the grasp and gone on arrival at the bin"},
                          {"action": "call_person", "kind": "dropped item", "why": "the robot cannot see where it fell"})
            self.log_event("gave_up", track, why="lost on the way to the bin, a person has to pick it up",
                           kind="dropped", seconds=0.0, attempts=track.attempts)
            return
        self.gripper_pub.publish(self._gripper_msg(GRIPPER_OPEN))
        self.sleep(0.2)
        self.placed[track.label] = self.placed.get(track.label, 0) + 1
        self.log_event("released", track, bin=track.label if track.label in layout.BINS else "hold")

    def report(self, sim_start):
        super().report(sim_start)
        rates = {}
        for kind, stamps in self.wrist_stamps.items():
            if len(stamps) > 1:
                rates[kind] = round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2)
        extra = {"wrist_frames_per_sim_s": rates, "wrist_looks": self.look_stats,
                 "diagnosis_requests": self.hook.count, "diagnosis_mode": self.hook.mode}
        path = os.path.expanduser(self.out_path)
        with open(path) as f:
            out = json.load(f)
        out.update(extra)
        with open(path, "w") as f:
            json.dump(out, f, indent=1)
        self.get_logger().info(f"wrist camera: {extra}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cycles", type=int, default=0, help="stop after N items (0 = forever)")
    parser.add_argument("--joint-speed", type=float, default=math.pi, help="joint speed limit [rad/s]")
    parser.add_argument("--joint-accel", type=float, default=3.0, help="joint acceleration limit [rad/s^2]")
    parser.add_argument("--plan-margin", type=float, default=PLAN_MARGIN)
    parser.add_argument("--until-fed", action="store_true",
                        help="stop when a feeder started with feed_limit:=N has fed everything and the last item is gone")
    parser.add_argument("--out", default="~/ros2_ws/log/sim/recovery_result.json", help="result file")
    parser.add_argument("--run-dir", default=os.environ.get("EXC_RUN_DIR", "~/ros2_ws/log/sim/exception/manual"),
                        help="folder for saved wrist frames and diagnosis requests")
    parser.add_argument("--save-frames", choices=["none", "failed", "all"], default="failed",
                        help="which wrist frames to keep: failed = only looked-at ones, all = every item (slower, for tuning)")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    arm_lock = lock_arm()  # noqa: F841
    rclpy.init()
    node = RecoverySorter(args.joint_speed, args.joint_accel, args.plan_margin, args.out, args.run_dir, args.save_frames)
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
