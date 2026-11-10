#!/usr/bin/env python3
"""Re-run the wrist-camera check on frames saved by recovery_sorter.py and compare with the truth.

  python3 tools/tune_wrist_check.py <run dir> [<run dir> ...] [--png out.png]

Needs runs made with --save-frames all (every item) or failed (only the looked-at ones).
For every saved frame: what the rules call it now, what it really was (from the simulator's grasp
events), and the measured numbers. Use it to set the thresholds in scripts/wrist_check.py.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import ur_kinematics as kin  # noqa: E402
import wrist_check  # noqa: E402


def truth_of_tracks(run_dir):
    """track id -> (true kind, detail) from the first grasp attempt on that track."""
    with open(os.path.join(run_dir, "truth.json")) as f:
        truth = json.load(f)
    with open(os.path.join(run_dir, "robot.json")) as f:
        robot = json.load(f)
    slot_of = {n: s for s in truth["feeder"]["items"] for n in s["names"]}
    grasps = [e for e in robot["events"] if e["event"] == "grasp"]
    real = [e for e in truth["grasp_events"] if e["event"] in ("grasp_fail", "attached")]
    out = {}
    for g, r in zip(grasps, real):
        if g["track"] in out:
            continue
        names = [o.split("::")[0] for o in r["objects"]]
        if not names:
            out[g["track"]] = ("empty", r["reason"])
            continue
        slot = slot_of[names[0]]
        kind = slot["kind"]
        detail = slot.get("how") or slot.get("yaw_deg") or ""
        if kind == "pair" and len(names) == 1:
            kind = "pair(one in reach)"
        out[g["track"]] = (kind, f"{detail} {r['reason']}".strip())
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--png", help="save a contact sheet of the frames")
    args = parser.parse_args()
    T_tool_cam = wrist_check.camera_in_tool()
    tiles = []
    for run in args.runs:
        run = os.path.expanduser(run.rstrip("/"))
        truth = truth_of_tracks(run)
        for path in sorted(glob.glob(os.path.join(run, "frames", "track*.npz"))):
            d = np.load(path)
            track = int(os.path.basename(path)[5:8])
            after = float(d["after_above"]) if "after_above" in d else 0.0
            f = wrist_check.measure(d["rgb"], d["depth"], d["K"], kin.fk(d["q"], tcp=False) @ T_tool_cam, d["expect"], T_tool_cam)
            kind = wrist_check.classify(f)
            real = truth.get(track, ("?", ""))
            nums = (f"h {f['height']:.3f} L {f['length']:.3f} W {f['width']:.3f} ang {np.degrees(f['angle']):5.1f} "
                    f"off ({f['centre'][0]-d['expect'][0]:+.3f},{f['centre'][1]-d['expect'][1]:+.3f}) belt_z {f['belt_z']:.3f} "
                    + " ".join(f"{k[0]}{v:.2f}" for k, v in f["colours"].items())) if f.get("seen") else f"not seen {f}"
            print(f"{os.path.basename(run)[9:24]:15s} #{track:02d} t{after:+.2f} {str(d['label']):7s} rules: {kind:8s} truth: {real[0]:18s} {real[1]:28s} {nums}")
            if args.png:
                import cv2

                tile = cv2.resize(d["rgb"][:, :, ::-1], (320, 240))
                cv2.putText(tile, f"#{track} {kind} / {real[0]}", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                tiles.append(tile)
    if args.png and tiles:
        import cv2

        cols = 6
        while len(tiles) % cols:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
        cv2.imwrite(args.png, np.vstack(rows))
        print("wrote", args.png)


if __name__ == "__main__":
    main()
