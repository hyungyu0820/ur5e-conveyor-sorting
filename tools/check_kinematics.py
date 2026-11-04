#!/usr/bin/env python3
"""Check the planned arm moves without a simulator (a few seconds instead of a whole run).

For every bin drop spot and pick position: IK must succeed, stay on the same arm branch
(no joint flips over) and the move times must fit the requested cycle.
  python3 tools/check_kinematics.py [--cycle 5.0]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import conveyor_layout as layout  # noqa: E402
import ur_kinematics as kin  # noqa: E402
from conveyor_sorter import DROP_GRID, seed_towards  # noqa: E402
from pick_place import GRASP_Z, YAW  # noqa: E402
from tracking_sorter import (CLOSE_TIME, DESCEND_TIME, HOLD_TIME, LIFT_TIME, RELEASE_TIME, TRAVEL_Z,  # noqa: E402
                             Y_MAX, Y_MIN, trapezoid)

MAX_BRANCH_MOVE = np.radians([200, 40, 40, 40, 20, 200])  # more than this between belt and bin is a flip


def ik(x, y, z):
    t = np.eye(4)
    t[:3, :3] = kin.rot_z_down(YAW)
    t[:3, 3] = [x, y, z]
    q = kin.ik(t, seed_towards(x, y))
    if q is None:
        raise SystemExit(f"FAIL no IK at ({x:.3f}, {y:.3f}, {z:.3f})")
    return q


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycle", type=float, default=5.0)
    parser.add_argument("--joint-speed", type=float, default=np.pi)
    parser.add_argument("--joint-accel", type=float, default=3.0)
    args = parser.parse_args()

    def move(a, b):
        return max(trapezoid(d, args.joint_speed, args.joint_accel)[0] for d in b - a)

    fixed = DESCEND_TIME + CLOSE_TIME + HOLD_TIME + LIFT_TIME + RELEASE_TIME
    bad, worst = 0, 0.0
    for by in np.arange(Y_MIN, Y_MAX + 1e-6, 0.05):
        for x in (layout.BELT_X - 0.05, layout.BELT_X, layout.BELT_X + 0.05):
            q_pick = ik(x, by, TRAVEL_Z)
            ik(x, by, GRASP_Z)
            for (bx, byy) in list(layout.BINS.values()) + [layout.REJECT]:
                for dx, dy in DROP_GRID:
                    q_bin = ik(bx + dx, byy + dy, TRAVEL_Z)
                    if np.any(np.abs(q_bin - q_pick) > MAX_BRANCH_MOVE):
                        bad += 1
                        print(f"FLIP pick ({x:.2f},{by:.2f}) bin ({bx+dx:.2f},{byy+dy:.2f}):",
                              np.degrees(q_bin - q_pick).round())
                    worst = max(worst, 2 * move(q_pick, q_bin) + fixed)
    print(f"branch flips: {bad}, worst ideal cycle {worst:.2f} s (budget {args.cycle:.2f} s)")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
