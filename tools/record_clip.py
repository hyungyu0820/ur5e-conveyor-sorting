#!/usr/bin/env python3
"""Record the overview camera for a demo clip, and cut GIFs that play at real speed.

  python3 tools/record_clip.py record <frames dir>                       # until Ctrl-C / SIGTERM
  python3 tools/record_clip.py gif <frames dir> <out.gif> --from T0 --to T1 [--fps 15] [--label "text"]

Software rendering only manages a few frames per second, so a clip recorded in real time stutters.
Record with the simulation slowed down (tools/record_clip.sh does that): every frame carries its
simulation time, and `gif` picks frames at even steps of simulation time, so the result plays at
the real speed of the cell. T0 and T1 are simulation times [s], as in the sorter's event log.
"""
import argparse
import glob
import json
import os
import signal
import sys


def record(out_dir):
    import numpy as np
    import rclpy
    from PIL import Image as PILImage
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Node("clip_recorder")
    count = [0]

    def on_image(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            data = data[:, :, ::-1]
        PILImage.fromarray(np.ascontiguousarray(data)).save(os.path.join(out_dir, f"{t:010.3f}.jpg"), quality=92)
        count[0] += 1

    node.create_subscription(Image, "/overview_camera/image_raw", on_image, qos_profile_sensor_data)
    def stop(*_):  # frames are already on disk: leave at once, a spinning node does not always return
        print(f"saved {count[0]} frames to {out_dir}", flush=True)
        os._exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.spin(node)


def gif(frames_dir, out, t0, t1, fps, label, width):
    from PIL import Image, ImageDraw, ImageFont

    files = sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))
    stamps = [float(os.path.basename(f)[:-4]) for f in files]
    inside = [s for s in stamps if t0 <= s <= t1]
    if len(inside) < 2:
        sys.exit(f"no frames between {t0} and {t1} (recorded: {stamps[0] if stamps else None} .. {stamps[-1] if stamps else None})")
    gaps = [b - a for a, b in zip(inside, inside[1:])]
    font = None
    if label:
        for path in (os.path.expanduser("~/.local/share/fonts/malgun.ttf"), "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
                     "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
            if os.path.exists(path):
                font = ImageFont.truetype(path, 18)
                break
    frames, n = [], int((t1 - t0) * fps)
    for i in range(n + 1):
        t = t0 + i / fps
        j = min(range(len(stamps)), key=lambda k: abs(stamps[k] - t))
        im = Image.open(files[j]).convert("RGB")
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
        if label and font:
            d = ImageDraw.Draw(im)
            d.rectangle([0, 0, im.width, 30], fill=(20, 20, 20))
            d.text((10, 4), label, fill=(255, 255, 255), font=font)
        frames.append(im)
    # one shared palette: per-frame palettes make the colours flicker
    palette = frames[len(frames) // 2].quantize(colors=255, method=Image.MEDIANCUT)
    quantised = [f.quantize(palette=palette, dither=Image.NONE) for f in frames]
    # GIF frame times are whole hundredths of a second: 15 frames per second becomes 70 ms a frame (0.95 x real speed)
    frame_ms = max(round(100 / fps), 2) * 10
    quantised[0].save(out, save_all=True, append_images=quantised[1:], duration=frame_ms, loop=0, optimize=False)
    info = {"gif": os.path.basename(out), "sim_from": t0, "sim_to": t1, "fps": fps, "frames": len(frames),
            "frame_ms": frame_ms, "playback_speed": round(1000 / fps / frame_ms, 2),
            "recorded_frames_inside": len(inside), "largest_gap_between_recorded_frames_s": round(max(gaps), 3),
            "mean_recorded_fps": round((len(inside) - 1) / (inside[-1] - inside[0]), 1)}
    print(json.dumps(info, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("frames_dir")
    g = sub.add_parser("gif")
    g.add_argument("frames_dir")
    g.add_argument("out")
    g.add_argument("--from", dest="t0", type=float, required=True)
    g.add_argument("--to", dest="t1", type=float, required=True)
    g.add_argument("--fps", type=float, default=15.0)
    g.add_argument("--label", default="")
    g.add_argument("--width", type=int, default=560)
    args = parser.parse_args()
    if args.cmd == "record":
        record(os.path.expanduser(args.frames_dir))
    else:
        gif(os.path.expanduser(args.frames_dir), os.path.expanduser(args.out), args.t0, args.t1, args.fps, args.label, args.width)


if __name__ == "__main__":
    main()
