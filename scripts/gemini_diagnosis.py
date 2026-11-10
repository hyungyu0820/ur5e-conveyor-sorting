"""Diagnosis hook: ask Gemini Robotics-ER what is wrong, only when the rules cannot tell.

    hook = DiagnosisHook(out_dir, log=print)
    answer = hook.ask(rgb, situation, fallback)

`situation` says what the robot saw and did (plain dict), `fallback` is what the rules would do
(dict with at least "action"). ask() never blocks and ALWAYS returns the fallback: on a belt that
does not stop an item stays in the pick zone for about 4 s, and a Gemini call took 5 to 6 s in this
project, so the robot acts on its rules at once and the model's answer is a second opinion that
arrives later (written next to the request, for comparing afterwards and for improving the rules).

Mode, environment variable EXC_GEMINI_MODE:
  dryrun  (default) no network at all. Saves the image and the exact prompt that would be sent.
  live    sends the request in a background thread. Needs a key (GEMINI_API_KEY or ~/.gemini_api_key,
          as in gemini_detector.py) and the google-genai package. NOT exercised yet: written against
          the same API calls as gemini_detector.py but never run.
  off     saves nothing, sends nothing.
Any other value counts as dryrun, so a typo can never cause a paid call.
"""
import base64
import io
import json
import os
import threading
import time

ACTIONS = ("retry", "regrasp_turned", "skip", "hold", "call_person")

PROMPT = """You are helping a robot arm that sorts items from a conveyor belt that never stops.
Something unusual happened and the robot's own rules could not explain it.
The image is from the camera on the robot's wrist, looking down at the item just before the grasp.

What the robot knows:
{situation}

What the robot's rules decided to do meanwhile: {fallback}

Answer only with JSON in this format:
{{"kind": "<what the item or problem is, a few words>", "action": "<one of {actions}>", "reason": "<one sentence>"}}
Meaning of the actions: retry = grasp again the same way; regrasp_turned = grasp again with the
gripper turned; skip = leave the item on the belt for the manual station downstream;
hold = put it in the hold bin; call_person = stop trying and alert a person."""


def mode():
    m = os.environ.get("EXC_GEMINI_MODE", "dryrun").strip().lower()
    return m if m in ("live", "off") else "dryrun"


def _png(rgb):
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="PNG")
    return buf.getvalue()


class DiagnosisHook:
    def __init__(self, out_dir, log=print):
        self.mode = mode()
        self.out_dir = os.path.expanduser(out_dir)
        self.log = log
        self.count = 0
        self.records = []  # one dict per request, also written to <out_dir>/NNN.json
        self.client = None
        if self.mode != "off":
            os.makedirs(self.out_dir, exist_ok=True)
        if self.mode == "live":
            from google import genai

            from gemini_detector import load_api_key

            self.client = genai.Client(api_key=load_api_key())
        self.log(f"diagnosis hook: {self.mode}" + (" (no Gemini call is made)" if self.mode != "live" else ""))

    def ask(self, rgb, situation, fallback):
        """Record (and in live mode send) a diagnosis request. Returns `fallback` with a note on its source."""
        answer = dict(fallback, source="rule" if self.mode == "off" else f"rule_fallback({self.mode})")
        if self.mode == "off":
            return answer
        self.count += 1
        stem = os.path.join(self.out_dir, f"{self.count:03d}")
        prompt = PROMPT.format(
            situation=json.dumps(situation, ensure_ascii=False, indent=1),
            fallback=json.dumps(fallback, ensure_ascii=False), actions=", ".join(ACTIONS),
        )
        record = {"n": self.count, "mode": self.mode, "situation": situation, "fallback": fallback,
                  "prompt": prompt, "image": f"{stem}.png" if rgb is not None else None, "gemini": None}
        png = None
        if rgb is not None:
            png = _png(rgb)
            with open(f"{stem}.png", "wb") as f:
                f.write(png)
        self.records.append(record)
        self._write(stem, record)
        if self.mode == "live":
            threading.Thread(target=self._call, args=(stem, record, png, prompt), daemon=True).start()
        return answer

    def _write(self, stem, record):
        with open(f"{stem}.json", "w") as f:
            json.dump(record, f, ensure_ascii=False, indent=1)

    def _call(self, stem, record, png, prompt):
        from gemini_detector import MODEL

        t0 = time.time()
        try:
            parts = [{"type": "text", "text": prompt}]
            if png is not None:
                parts.insert(0, {"type": "image", "data": base64.b64encode(png).decode(), "mime_type": "image/png"})
            response = self.client.interactions.create(model=MODEL, input=parts, generation_config={"thinking_level": "low"})
            text = (response.output_text or "").strip()
            if text.startswith("```"):
                text = text.strip("`").removeprefix("json").strip()
            record["gemini"] = {"seconds": round(time.time() - t0, 1), "answer": json.loads(text)}
        except Exception as e:  # noqa: BLE001 (network, key, answer format: the robot already acted on its rules)
            record["gemini"] = {"seconds": round(time.time() - t0, 1), "error": f"{type(e).__name__}: {str(e)[:200]}"}
        self._write(stem, record)
        self.log(f"gemini second opinion #{record['n']}: {json.dumps(record['gemini'], ensure_ascii=False)[:200]}")
