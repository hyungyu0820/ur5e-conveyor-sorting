#!/usr/bin/env python3
"""Turn exception-handling runs into numbers: what happened to every fed object, and what it cost.

  python3 tools/exception_report.py <run dir> [<run dir> ...] [--json out.json] [--items]

A run dir is what tools/exception_trial.sh writes (truth.json from the simulator, robot.json from
the sorter). Runs are grouped by mode (the folder name starts with baseline_ or recovery_).

Outcome of one object, from the simulator's truth only:
  sorted      it lies in the right place: its colour bin, or the hold bin for an unknown item
  wrong_bin   it lies in another bin
  dropped     the gripper held it at some time, but it did not end up in a bin
  passed      the gripper never held it and it rode on to the end of the belt
              (passed_tried: a grasp was attempted on it; passed_untouched: never between the fingers)
"""
import argparse
import json
import os
import statistics
import sys

KINDS = {"item": "normal", "long": "long", "pair": "pair", "unk": "unknown"}
BELT_SPEED, SPACING = 0.10, 0.50
BINS = {"red": (0.15, 0.45), "green": (-0.15, 0.45), "blue": (0.15, -0.45), "hold": (-0.15, -0.45)}  # conveyor_layout
BIN_HALF = 0.11


def place_of(pos):
    """Bin whose footprint the object lies in (also on top of a pile), else belt or floor."""
    x, y, z = pos
    for name, (bx, by) in BINS.items():
        if abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF and z < 0.30:
            return name
    return "belt" if abs(x - 0.5) < 0.1 and z > 0.15 else "floor"


def load(run_dir):
    run_dir = os.path.expanduser(run_dir.rstrip("/"))
    with open(os.path.join(run_dir, "truth.json")) as f:
        truth = json.load(f)
    with open(os.path.join(run_dir, "robot.json")) as f:
        robot = json.load(f)
    name = os.path.basename(run_dir)
    gemini = os.path.join(run_dir, "gemini")
    requests = len([f for f in os.listdir(gemini) if f.endswith(".json")]) if os.path.isdir(gemini) else 0
    return name, truth, robot, requests


def analyse(name, truth, robot, requests):
    fed = [n for slot in truth["feeder"]["items"] for n in slot["names"]]
    slot_of = {n: slot for slot in truth["feeder"]["items"] for n in slot["names"]}
    objects = []
    for n in fed:
        info = truth["items"].get(n)
        kind = KINDS[n.split("_")[0]]
        colour = n.rsplit("_", 1)[1]
        want = "hold" if kind == "unknown" else colour
        mine = [e for e in truth["grasp_events"] if any(o.startswith(n + "::") for o in e["objects"])]
        held = any(e["event"] == "attached" for e in mine)
        fails = [e["reason"].replace("fail:", "") for e in mine if e["event"] == "grasp_fail"]
        place = place_of(info["pos"]) if info else "never seen"
        if place == want:
            outcome = "sorted"
        elif place in ("red", "green", "blue", "hold"):
            outcome = "wrong_bin"
        elif held:
            outcome = "dropped"
        else:
            outcome = "passed_tried" if fails else "passed_untouched"
        objects.append({
            "name": n, "slot": slot_of[n]["slot"], "kind": kind, "detail": slot_of[n].get("how") or slot_of[n].get("yaw_deg"),
            "outcome": outcome, "place": place, "attaches": sum(e["event"] == "attached" for e in mine),
            "fail_reasons": fails, "dropped_in_transport": any(e["event"] == "dropped" for e in mine),
            "at_zone": info.get("at_zone") if info else None,
        })

    # were the agreed fault settings really in force at every grasp? (older runs do not log them)
    settings = {(e.get("slip_rate"), e.get("drop_rate"), e.get("seed")) for e in truth["grasp_events"]}
    faults_ok = settings == {(0.1, 0.05, truth["feeder"]["seed"])} if settings and None not in next(iter(settings)) else None

    ev = robot["events"]
    grasps = [e for e in ev if e["event"] == "grasp"]
    first_empty = {}
    for e in ev:
        if e["event"] == "exception" and e["kind"] == "grasp_empty":
            first_empty.setdefault(e["track"], e["t"])
    if not first_empty:  # the sorter without recovery logs no "exception": take its empty grasps
        for e in grasps:
            if not e["holding"]:
                first_empty.setdefault(e["track"], e["t"])
    recovered = [e for e in ev if e["event"] == "recovered"]
    gave_up = [e for e in ev if e["event"] == "gave_up"]
    lost = [e for e in ev if e["event"] == "exception" and e["kind"] == "lost_in_transport"]
    # time the arm was tied up by an exception that was not recovered: empty grasp -> free again
    give_up_s = []
    for e in gave_up:
        if e.get("kind") == "dropped":
            continue
        t0 = first_empty.get(e["track"])
        if t0 is not None:
            give_up_s.append(round(e["t"] - t0, 3))
    counted_sorted = sum(e["event"] == "released" for e in ev)
    drops_truth = [o for o in objects if o["dropped_in_transport"]]

    n_slots = len(truth["feeder"]["items"])
    window = n_slots * SPACING / BELT_SPEED
    count = lambda pred: sum(1 for o in objects if pred(o))  # noqa: E731
    by_kind = {}
    for o in objects:
        k = by_kind.setdefault(o["kind"], {"n": 0, "sorted": 0})
        k["n"] += 1
        k[o["outcome"]] = k.get(o["outcome"], 0) + 1
    # two items that came together: how many of each kind of pile gave up at least one item, or both
    piles = {}
    for slot in truth["feeder"]["items"]:
        if slot["kind"] != "pair":
            continue
        got = sum(1 for o in objects if o["slot"] == slot["slot"] and o["outcome"] == "sorted")
        p = piles.setdefault(slot["how"], {"piles": 0, "one_sorted": 0, "both_sorted": 0})
        p["piles"] += 1
        p["one_sorted"] += got == 1
        p["both_sorted"] += got == 2
    second = [e for e in ev if e["event"] == "second_item_queued"]
    second_gone = [e for e in ev if e["event"] == "missed" and "second item" in e.get("why", "")]
    second_ids = {e["track"] for e in second}
    return {
        "piles": piles,
        "second_items_queued": len(second),
        "second_items_out_of_reach": len(second_gone) + sum(1 for e in ev if e["event"] == "missed" and e["track"] in second_ids
                                                              and "second item" not in e.get("why", "")),
        "run": name, "seed": truth["feeder"]["seed"], "slots": n_slots, "objects": len(objects),
        "sorted": count(lambda o: o["outcome"] == "sorted"),
        "wrong_bin": count(lambda o: o["outcome"] == "wrong_bin"),
        "dropped": count(lambda o: o["outcome"] == "dropped"),
        "passed_tried": count(lambda o: o["outcome"] == "passed_tried"),
        "passed_untouched": count(lambda o: o["outcome"] == "passed_untouched"),
        "by_kind": by_kind,
        "throughput_sorted_per_h": round(count(lambda o: o["outcome"] == "sorted") / window * 3600, 1),
        "input_window_s": window,
        "grasp_attempts": len(grasps),
        "empty_grasps": sum(1 for e in grasps if not e["holding"]),
        "exceptions_noticed": len(first_empty) + len(lost),
        "recovered": len(recovered),
        "recovery_s": [e["seconds"] for e in recovered],
        "recovered_how": {h: sum(1 for e in recovered if e["how"] == h) for h in sorted({e["how"] for e in recovered})},
        "recovery_s_by_how": {h: [e["seconds"] for e in recovered if e["how"] == h] for h in sorted({e["how"] for e in recovered})},
        "gave_up": len(gave_up),
        "give_up_s": give_up_s,
        "gave_up_kinds": {k: sum(1 for e in gave_up if e.get("kind") == k) for k in sorted({str(e.get("kind")) for e in gave_up})},
        "robot_missed_busy": sum(e["event"] == "missed" for e in ev),
        "robot_counted_sorted": counted_sorted,
        "drops_in_transport_truth": len(drops_truth),
        "drops_noticed_by_robot": len(lost),
        "diagnosis_requests": requests,
        "sim_elapsed_s": robot.get("sim_elapsed_s"),
        "fault_settings_verified": faults_ok,
        "wrist_frames_per_sim_s": robot.get("wrist_frames_per_sim_s"),
        "wrist_looks": {k: v for k, v in robot.get("wrist_looks", {}).items() if k != "frames_in_window"} or None,
        "true_slips": sum(1 for e in truth["grasp_events"] if e["reason"] == "fail:slipped"),
        "items": objects,
    }


def total(runs):
    keys = ["slots", "objects", "sorted", "wrong_bin", "dropped", "passed_tried", "passed_untouched", "grasp_attempts",
            "empty_grasps", "exceptions_noticed", "recovered", "gave_up", "robot_missed_busy", "robot_counted_sorted",
            "drops_in_transport_truth", "drops_noticed_by_robot", "diagnosis_requests", "second_items_queued",
            "second_items_out_of_reach"]
    out = {k: sum(r[k] for r in runs) for k in keys}
    out["seeds"] = [r["seed"] for r in runs]
    out["sorted_share"] = round(out["sorted"] / out["objects"], 4)
    out["throughput_sorted_per_h"] = round(out["sorted"] / sum(r["input_window_s"] for r in runs) * 3600, 1)
    rec = [s for r in runs for s in r["recovery_s"]]
    giv = [s for r in runs for s in r["give_up_s"]]
    out["recovery_s"] = {"n": len(rec), "mean": round(statistics.mean(rec), 2) if rec else None,
                         "median": round(statistics.median(rec), 2) if rec else None, "max": max(rec) if rec else None}
    out["give_up_s"] = {"n": len(giv), "mean": round(statistics.mean(giv), 2) if giv else None,
                        "max": max(giv) if giv else None}
    by_kind = {}
    for r in runs:
        for kind, c in r["by_kind"].items():
            k = by_kind.setdefault(kind, {})
            for name, v in c.items():
                k[name] = k.get(name, 0) + v
    out["by_kind"] = by_kind
    out["piles"] = {}
    for r in runs:
        for how, c in r["piles"].items():
            p = out["piles"].setdefault(how, {})
            for name, v in c.items():
                p[name] = p.get(name, 0) + v
    by_how = {}
    for r in runs:
        for h, secs in r["recovery_s_by_how"].items():
            by_how.setdefault(h, []).extend(secs)
    out["recovery_s_by_how"] = {h: {"n": len(v), "mean": round(statistics.mean(v), 2), "min": min(v), "max": max(v)} for h, v in by_how.items()}
    out["recovered_how"] = {}
    out["gave_up_kinds"] = {}
    for r in runs:
        for h, v in r["recovered_how"].items():
            out["recovered_how"][h] = out["recovered_how"].get(h, 0) + v
        for h, v in r["gave_up_kinds"].items():
            out["gave_up_kinds"][h] = out["gave_up_kinds"].get(h, 0) + v
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--json", help="write everything to this file")
    parser.add_argument("--items", action="store_true", help="print the per-object table")
    args = parser.parse_args()
    results = [analyse(*load(r)) for r in args.runs]
    modes = {}
    for r in results:
        modes.setdefault(r["run"].split("_")[0], []).append(r)
    cols = ["sorted", "wrong_bin", "dropped", "passed_tried", "passed_untouched"]
    for r in results:
        print(f"{r['run']}: objects {r['objects']} (slots {r['slots']}) | " + ", ".join(f"{c} {r[c]}" for c in cols)
              + f" | sorted/h {r['throughput_sorted_per_h']} | empty grasps {r['empty_grasps']}, recovered {r['recovered']}"
              f" {r['recovery_s']}, gave up {r['gave_up']}, missed while busy {r['robot_missed_busy']},"
              f" robot counted {r['robot_counted_sorted']} sorted, drops {r['drops_in_transport_truth']}"
              f" (noticed {r['drops_noticed_by_robot']}), diagnosis requests {r['diagnosis_requests']}")
        print("   by kind:", {k: f"{v.get('sorted', 0)}/{v['n']}" for k, v in r["by_kind"].items()},
              "| fault settings verified:", r["fault_settings_verified"], "| true slips", r["true_slips"])
        if args.items:
            for o in r["items"]:
                print(f"   slot {o['slot']:2d} {o['name']:20s} {o['kind']:8s} {str(o['detail']):13s} {o['outcome']:17s}"
                      f" {o['place']:10s} fails {o['fail_reasons']} {'DROPPED' if o['dropped_in_transport'] else ''}")
    summary = {m: total(rs) for m, rs in modes.items()}
    for m, t in summary.items():
        print(f"== {m} (seeds {t['seeds']}): sorted {t['sorted']}/{t['objects']} = {t['sorted_share']:.1%},"
              f" sorted/h {t['throughput_sorted_per_h']}, recovery {t['recovery_s']}, give-up {t['give_up_s']}")
        print("   by kind:", {k: f"{v.get('sorted', 0)}/{v['n']}" for k, v in t["by_kind"].items()})
        print("   piles:", t["piles"], "| recovered how:", t["recovered_how"], "| second items queued", t["second_items_queued"],
              "out of reach", t["second_items_out_of_reach"], "| missed while busy", t["robot_missed_busy"])
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w") as f:
            json.dump({"summary": summary, "runs": results}, f, indent=1, ensure_ascii=False)
        print("wrote", args.json)


if __name__ == "__main__":
    sys.exit(main())
