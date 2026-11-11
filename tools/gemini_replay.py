#!/usr/bin/env python3
"""Send diagnosis requests that were saved in dry-run mode to Gemini, afterwards and outside any run.

  python3 tools/gemini_replay.py <run dir> [<run dir> ...]            # list only, no network
  python3 tools/gemini_replay.py <run dir> ... --live [--max-calls N] # really call, one after the other

A run dir is what tools/exception_trial.sh writes; its gemini/ folder holds NNN.json (+ NNN.png) from
scripts/gemini_diagnosis.py. Every saved request is sent once, with the prompt and image exactly as
saved and the model configured in scripts/gemini_detector.py, and the answer is compared with what
the rules did. The robot is not involved: this never runs inside a measured trial.

Safety: every request counts as a call, failed ones too. The ledger <out>/calls.json keeps the total
over all invocations and the tool refuses to go beyond HARD_CAP. The first failed call stops everything.
The API key is read by gemini_detector.load_api_key() and never printed or saved.
"""
import argparse
import base64
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

HARD_CAP = 20


def load_requests(run_dirs):
    out = []
    for run in run_dirs:
        run = os.path.expanduser(run.rstrip("/"))
        for path in sorted(glob.glob(os.path.join(run, "gemini", "[0-9][0-9][0-9].json"))):
            with open(path) as f:
                rec = json.load(f)
            png = path[:-5] + ".png"
            out.append({"run": os.path.basename(run), "n": rec["n"], "prompt": rec["prompt"], "situation": rec["situation"],
                        "rule": rec["fallback"], "png": png if os.path.exists(png) else None})
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--live", action="store_true", help="really send the requests (default: list them only)")
    parser.add_argument("--max-calls", type=int, default=12, help=f"calls allowed in this invocation (total never above {HARD_CAP})")
    parser.add_argument("--out", default="~/ros2_ws/log/sim/exception/gemini_live", help="folder for the ledger and answers")
    parser.add_argument("--blind", action="store_true",
                        help="leave the rules' own decision out of the prompt (image cases only), to see what the model says unprompted")
    parser.add_argument("--json", help="also write the comparison (no images, no key) to this file")
    args = parser.parse_args()
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    ledger_path = os.path.join(out_dir, "calls.json")
    ledger = json.load(open(ledger_path)) if os.path.exists(ledger_path) else {"hard_cap": HARD_CAP, "calls": []}
    requests = load_requests(args.runs)
    variant = "blind" if args.blind else "as_saved"
    if args.blind:
        requests = [r for r in requests if r["png"]]
        for r in requests:
            lines = r["prompt"].split("\n")
            cut = [i for i, line in enumerate(lines) if line.startswith("What the robot's rules decided")]
            assert len(cut) == 1, "prompt format changed"
            del lines[cut[0]:cut[0] + 2]  # the sentence and the empty line after it
            r["prompt"] = "\n".join(lines).replace("and the robot's own rules could not explain it", "on the belt")
    done = {(c["run"], c["n"]) for c in ledger["calls"] if c.get("variant", "as_saved") == variant}
    todo = [r for r in requests if (r["run"], r["n"]) not in done]
    print(f"saved requests: {len(requests)} ({sum(r['png'] is not None for r in requests)} with an image), "
          f"calls made so far: {len(ledger['calls'])} of at most {HARD_CAP}, still to send: {len(todo)}")
    if not args.live:
        for r in requests:
            print(f"  {r['run']} #{r['n']:03d} image {'yes' if r['png'] else 'no '} rule: {r['rule'].get('action')} ({r['rule'].get('kind')})")
        print("list only, nothing is sent. Add --live to send them.")
        todo = []
    else:
        from google import genai

        from gemini_detector import MODEL, load_api_key

        key = load_api_key()
        client = genai.Client(api_key=key)
    budget = min(args.max_calls, HARD_CAP - len(ledger["calls"]))
    for r in todo:
        if budget <= 0:
            print("call limit reached, stopping")
            break
        budget -= 1
        parts = [{"type": "text", "text": r["prompt"]}]
        if r["png"]:
            with open(r["png"], "rb") as f:
                parts.insert(0, {"type": "image", "data": base64.b64encode(f.read()).decode(), "mime_type": "image/png"})
        call = {"run": r["run"], "n": r["n"], "variant": variant, "model": MODEL, "image": bool(r["png"]), "rule": r["rule"],
                "situation": r["situation"], "at": time.strftime("%Y-%m-%d %H:%M:%S")}
        t0 = time.time()
        try:
            response = client.interactions.create(model=MODEL, input=parts, generation_config={"thinking_level": "low"})
            text = (response.output_text or "").strip()
            call["seconds"] = round(time.time() - t0, 2)
            call["raw"] = text[:600]
            if text.startswith("```"):
                text = text.strip("`").removeprefix("json").strip()
            try:
                call["answer"] = json.loads(text)
            except ValueError:
                call["answer"] = None
        except Exception as e:  # noqa: BLE001
            call["seconds"] = round(time.time() - t0, 2)
            call["error"] = f"{type(e).__name__}: {str(e)[:300]}".replace(key, "<key>")
        ledger["calls"].append(call)
        with open(ledger_path, "w") as f:  # written after every call: the count survives a crash
            json.dump(ledger, f, ensure_ascii=False, indent=1)
        if "error" in call:
            print(f"  {r['run']} #{r['n']:03d} FAILED after {call['seconds']} s: {call['error']}")
            print("a call failed: stopping, nothing more is sent")
            break
        a = call["answer"] if isinstance(call["answer"], dict) else {}
        print(f"  {r['run']} #{r['n']:03d} {call['seconds']:5.2f} s  rule: {r['rule'].get('action'):12s} gemini: "
              f"{str(a.get('action')):12s} | {a.get('kind')} | {a.get('reason')}")

    calls = ledger["calls"]

    def summarise(part):
        ok = [c for c in part if isinstance(c.get("answer"), dict)]
        same = [c for c in ok if c["answer"].get("action") == c["rule"].get("action")]
        secs = sorted(c["seconds"] for c in ok)
        return {
            "calls_made": len(part), "answered": len(ok), "failed": sum("error" in c for c in part),
            "unreadable_answers": sum("error" not in c and not isinstance(c.get("answer"), dict) for c in part),
            "same_action_as_rule": len(same),
            "seconds": {"min": secs[0], "median": secs[len(secs) // 2], "max": secs[-1],
                        "mean": round(sum(secs) / len(secs), 2)} if secs else None,
        }

    summary = {"model": calls[0]["model"] if calls else None, "hard_cap": HARD_CAP, "calls_made_total": len(calls)}
    for v in ("as_saved", "blind"):
        part = [c for c in calls if c.get("variant", "as_saved") == v]
        if part:
            summary[v] = summarise(part)
    print("summary:", json.dumps(summary, ensure_ascii=False))
    if args.json:
        with open(os.path.expanduser(args.json), "w") as f:
            json.dump({"summary": summary, "calls": calls}, f, ensure_ascii=False, indent=1)
        print("wrote", args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
