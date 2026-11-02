"""Item detector backed by Gemini Robotics-ER (vision-language model).

Same interface as color_detector.detect():

    detect(rgb: np.ndarray[H, W, 3] uint8) -> list[Detection]

Gemini points at each item on the belt and chooses the bin for it. An optional
natural-language rule changes how items are sorted, for example
"초록색 물건은 파란 분류함에 넣어" (put green items in the blue bin).

API key: environment variable GEMINI_API_KEY, or the first line of ~/.gemini_api_key.
Model docs: https://ai.google.dev/gemini-api/docs/robotics-overview
"""
import base64
import io
import json
import os
import time

from PIL import Image

from color_detector import Detection

MODEL = "gemini-robotics-er-2-preview"
BINS = ("red", "green", "blue")

PROMPT = """You are the vision system of a robot sorting items on a conveyor belt.
Point to every item lying on the conveyor belt (ignore the yellow sensor post and the belt itself).
For each item choose the bin it must go to. Bins: {bins}.
Sorting rule: {rule}
If an item does not fit any bin under the rule, use "unknown".
Answer only with JSON in this format:
[{{"point": [y, x], "item": "<short description>", "bin": "<one of {bins} or unknown>"}}]
The points are in [y, x] format normalized to 0-1000."""

DEFAULT_RULE = "put each item in the bin of the same colour"


def load_api_key():
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    path = os.path.expanduser("~/.gemini_api_key")
    if not key and os.path.exists(path):
        with open(path) as f:
            key = f.readline().strip()
    if not key:
        raise RuntimeError("Gemini API 키가 없어요: GEMINI_API_KEY 환경 변수나 ~/.gemini_api_key 파일에 넣어 주세요")
    return key


def _parse(text):
    text = text.strip()
    if text.startswith("```"):  # 코드 블록으로 감싸서 줄 때
        text = text.strip("`").removeprefix("json").strip()
    data = json.loads(text)
    return data if isinstance(data, list) else []


class GeminiDetector:
    def __init__(self, rule=None, thinking_level="low", log=print):
        from google import genai  # 색 인식만 쓸 때는 설치 안 해도 되도록 여기서 불러옴

        self.client = genai.Client(api_key=load_api_key())
        self.rule = rule or DEFAULT_RULE
        self.thinking_level = thinking_level
        self.log = log
        self.last_items = []  # 마지막 응답 (화면 · 로그용)

    def __call__(self, rgb):
        h, w = rgb.shape[:2]
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="PNG")
        t0 = time.time()
        response = self.client.interactions.create(
            model=MODEL,
            input=[
                {"type": "image", "data": base64.b64encode(buf.getvalue()).decode(), "mime_type": "image/png"},
                {"type": "text", "text": PROMPT.format(bins=", ".join(BINS), rule=self.rule)},
            ],
            generation_config={"thinking_level": self.thinking_level},
        )
        items = _parse(response.output_text or "[]")
        self.last_items = items
        self.log(f"gemini {time.time() - t0:.1f}s: {json.dumps(items, ensure_ascii=False)}")

        detections = []
        for it in items:
            try:
                y, x = it["point"]
            except (KeyError, TypeError, ValueError):
                continue
            label = str(it.get("bin", "unknown")).lower()
            detections.append(Detection(label if label in BINS else "unknown", x / 1000 * w, y / 1000 * h, 0))
        return detections
