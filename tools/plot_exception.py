#!/usr/bin/env python3
"""Figures for the exception-handling report.

  python3 tools/plot_exception.py docs/data/exception_results.json            -> docs/media/exception_sorted_by_kind.png
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
    base, rec = summary["baseline"], summary["recovery"]
    kinds = ["normal", "long", "pair", "unknown", "all"]

    def share(mode, kind):
        if kind == "all":
            return mode["sorted"], mode["objects"]
        k = mode["by_kind"].get(kind, {"n": 0})
        return k.get("sorted", 0), k["n"]

    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=200, facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    h = 0.30
    for i, kind in enumerate(kinds):
        y = len(kinds) - 1 - i
        for mode, colour, dy in ((base, CONTEXT, h / 2 + 0.02), (rec, ACCENT, -h / 2 - 0.02)):
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
    handles = [plt.Rectangle((0, 0), 1, 1, color=CONTEXT), plt.Rectangle((0, 0), 1, 1, color=ACCENT)]
    ax.legend(handles, ["복구 없음 (지금 방식)", "복구 있음"], loc="lower right", frameon=False, fontsize=9, labelcolor=TEXT)
    seeds = ", ".join(str(s) for s in base["seeds"])
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="?")
    parser.add_argument("--wrist", nargs="+", help="run dir, then track ids")
    args = parser.parse_args()
    font = korean_font()
    if font:
        plt.rcParams["font.family"] = font
    plt.rcParams["axes.unicode_minus"] = False
    os.makedirs(MEDIA, exist_ok=True)
    if args.results:
        bars(args.results, os.path.join(MEDIA, "exception_sorted_by_kind.png"))
    if args.wrist:
        wrist(args.wrist[0], args.wrist[1:], os.path.join(MEDIA, "exception_wrist_views.png"))


if __name__ == "__main__":
    main()
