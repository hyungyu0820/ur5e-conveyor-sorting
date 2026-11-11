#!/usr/bin/env python3
"""Figures for the exception-handling report.

  python3 tools/plot_exception.py docs/data/exception_results.json            -> docs/media/exception_sorted_by_kind.png
  python3 tools/plot_exception.py <results.json> --out name.png               (every mode in the file gets a bar:
                                                                               baseline, recovery, overlap)
  python3 tools/plot_exception.py --wrist <run dir with frames> [track ids]   -> docs/media/exception_wrist_views.png

Bar chart: share of fed objects that ended in the right place, per kind of item, without and with
recovery (one accent colour for "with recovery", grey for "without"; numbers on the bars).
"""
import argparse
import glob
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MEDIA = os.path.join(HERE, "..", "docs", "media")
SURFACE, TEXT, TEXT2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e0"
ACCENT, CONTEXT = "#2a78d6", "#8c8b85"  # with recovery / without (emphasis: one hue + grey)
ACCENT2 = "#eb6834"  # recovery + taking overlapping items apart (second categorical slot, checked against the blue)
MODES = [("baseline", "복구 없음 (지금 방식)", CONTEXT), ("recovery", "복구 있음", ACCENT),
         ("overlap", "복구 + 겹친 물건 떼어 집기", ACCENT2)]
KIND_KO = {"normal": "정상 물건", "long": "돌아간 긴 상자", "pair": "겹친 물건", "unknown": "모르는 물건", "all": "전체"}


def korean_font():
    for path in font_manager.findSystemFonts():
        if os.path.basename(path).lower() in ("malgun.ttf", "nanumgothic.ttf", "notosanscjk-regular.ttc"):
            font_manager.fontManager.addfont(path)
            return font_manager.FontProperties(fname=path).get_name()
    return None


def bars(path, out):
    with open(path) as f:
        summary = json.load(f)["summary"]
    modes = [(summary[key], name, colour) for key, name, colour in MODES if key in summary]
    kinds = ["normal", "long", "pair", "unknown", "all"]

    def share(mode, kind):
        if kind == "all":
            return mode["sorted"], mode["objects"]
        k = mode["by_kind"].get(kind, {"n": 0})
        return k.get("sorted", 0), k["n"]

    n_modes = len(modes)
    h = 0.30 if n_modes == 2 else 0.24
    step = h + 0.04
    fig, ax = plt.subplots(figsize=(8.0, 4.2 if n_modes == 2 else 5.2), dpi=200, facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for i, kind in enumerate(kinds):
        y = len(kinds) - 1 - i
        for j, (mode, _, colour) in enumerate(modes):
            dy = ((n_modes - 1) / 2 - j) * step
            s, n = share(mode, kind)
            v = 100 * s / n if n else 0
            ax.barh(y + dy, v, height=h, color=colour, linewidth=0)
            ax.text(v + 1.5, y + dy, f"{s}/{n}  ({v:.0f}%)", va="center", ha="left", fontsize=9, color=TEXT)
    ax.set_yticks(range(len(kinds)))
    ax.set_yticklabels([KIND_KO[k] for k in reversed(kinds)], fontsize=10, color=TEXT)
    ax.set_xlim(0, 118)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xticklabels(["0", "25", "50", "75", "100%"], fontsize=9, color=TEXT2)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(length=0)
    handles = [plt.Rectangle((0, 0), 1, 1, color=colour) for _, _, colour in modes]
    ax.legend(handles, [name for _, name, _ in modes], loc="lower right", frameon=False, fontsize=9, labelcolor=TEXT)
    seeds = ", ".join(str(s) for s in modes[0][0]["seeds"])
    ax.set_title("제자리에 들어간 물건의 비율 (시뮬레이션, 1배속, seed " + seeds + ")", fontsize=11, color=TEXT, loc="left", pad=12)
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    print("wrote", out)


def wrist(run_dir, tracks, out):
    import numpy as np

    sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
    import ur_kinematics as kin
    import wrist_check

    names = {"single": "물건 1개", "long": "긴 상자", "pair": "옆으로 붙은 2개", "stacked": "위로 쌓인 2개",
             "nothing": "안 보임", "unclear": "판단 불가"}
    T = wrist_check.camera_in_tool()
    files = []
    for t in tracks:
        found = sorted(glob.glob(os.path.join(os.path.expanduser(run_dir), "frames", f"track{int(t):03d}_*.npz")))
        if found:
            files.append(found[0])
    fig, axes = plt.subplots(1, len(files), figsize=(3.0 * len(files), 2.9), dpi=200, facecolor=SURFACE)
    for ax, path in zip(np.atleast_1d(axes), files):
        d = np.load(path)
        f = wrist_check.measure(d["rgb"], d["depth"], d["K"], kin.fk(d["q"], tcp=False) @ T, d["expect"], T)
        kind = wrist_check.classify(f)
        ax.imshow(d["rgb"])
        ax.set_axis_off()
        detail = f"높이 {f['height']*100:.1f} cm, 길이 {f['length']*100:.1f} × 폭 {f['width']*100:.1f} cm" if f.get("seen") else ""
        ax.set_title(f"규칙의 판단: {names[kind]}\n{detail}", fontsize=8.5, color=TEXT)
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    print("wrote", out)


def gemini(path, out, window_s=4.0, clip_s=12.0):
    """Response time of every live diagnosis call against the time an item stays in the pick zone."""
    with open(path) as f:
        data = json.load(f)
    calls = [c for c in data["calls"] if isinstance(c.get("answer"), dict)]
    fig, ax = plt.subplots(figsize=(8.0, 4.4), dpi=200, facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for i, c in enumerate(calls):
        y = len(calls) - 1 - i
        same = c["answer"].get("action") == c["rule"].get("action")
        v = min(c["seconds"], clip_s)
        ax.barh(y, v, height=0.62, color=ACCENT if same else CONTEXT, linewidth=0)
        note = f"{c['seconds']:.1f}초" + ("  (축 밖)" if c["seconds"] > clip_s else "")
        ax.text(v + 0.15, y, note, va="center", ha="left", fontsize=8.5, color=TEXT)
    ax.axvline(window_s, color=TEXT, linewidth=1.0, linestyle=(0, (4, 3)))
    ax.text(window_s + 0.1, len(calls) - 0.35, f"물건이 집는 구간에 머무는 시간 {window_s:.0f}초", fontsize=9, color=TEXT, va="bottom")
    ax.set_yticks(range(len(calls)))
    ax.set_yticklabels([("사진 있음" if c["image"] else "사진 없음") + f" · 규칙: {RULE_KO.get(c['rule'].get('action'), c['rule'].get('action'))}"
                        for c in reversed(calls)], fontsize=8.5, color=TEXT)
    ax.set_xlim(0, clip_s + 2.2)
    ax.set_ylim(-0.6, len(calls) + 0.4)
    ax.set_xticks([0, 2, 4, 6, 8, 10, 12])
    ax.set_xticklabels(["0", "2", "4", "6", "8", "10", "12초"], fontsize=9, color=TEXT2)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(length=0)
    handles = [plt.Rectangle((0, 0), 1, 1, color=ACCENT), plt.Rectangle((0, 0), 1, 1, color=CONTEXT)]
    ax.legend(handles, ["규칙과 같은 판단", "규칙과 다른 판단"], loc="lower right", frameon=False, fontsize=9, labelcolor=TEXT)
    ax.set_title(f"Gemini 응답 시간, 저장해 둔 진단 요청 {len(calls)}건 (실제 호출, 로봇 제어와 무관)", fontsize=11, color=TEXT, loc="left", pad=12)
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    print("wrote", out)


RULE_KO = {"hold": "보류함", "call_person": "사람 호출", "skip": "흘려보냄", "retry": "다시 집기", "regrasp_turned": "돌려 집기"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="?")
    parser.add_argument("--wrist", nargs="+", help="run dir, then track ids")
    parser.add_argument("--gemini", help="docs/data/gemini_live_check.json -> docs/media/gemini_latency.png")
    parser.add_argument("--out", default="exception_sorted_by_kind.png", help="file name of the bar chart in docs/media")
    args = parser.parse_args()
    font = korean_font()
    if font:
        plt.rcParams["font.family"] = font
    plt.rcParams["axes.unicode_minus"] = False
    os.makedirs(MEDIA, exist_ok=True)
    if args.results:
        bars(args.results, os.path.join(MEDIA, args.out))
    if args.gemini:
        gemini(args.gemini, os.path.join(MEDIA, "gemini_latency.png"))
    if args.wrist:
        wrist(args.wrist[0], args.wrist[1:], os.path.join(MEDIA, "exception_wrist_views.png"))


if __name__ == "__main__":
    main()
