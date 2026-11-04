#!/usr/bin/env python3
"""Grade a sorting run from Gazebo's true object positions; the sorter never sees these.

Item names contain the true colour (item_0007_red), so an item counts as correct when it lies
inside the bin of that colour. Run while the simulation is still up:
  ros2 run ur_pick_place score_bins.py --items 30
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import conveyor_layout as layout  # noqa: E402
from pick_place import object_position  # noqa: E402

BIN_HALF = 0.11
BIN_TOP = 0.10


def where(x, y, z):
    if z < BIN_TOP:
        for name, (bx, by) in list(layout.BINS.items()) + [("hold", layout.REJECT)]:
            if abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF:
                return name
        return "floor"
    return "belt" if abs(x - layout.BELT_X) < 0.1 else "elsewhere"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--items", type=int, default=30, help="check item numbers 0 .. N-1")
    args = parser.parse_args()
    correct = wrong = 0
    other = {}
    for n in range(args.items):
        for color in layout.GAZEBO_COLORS:
            name = f"item_{n:04d}_{color}"
            try:
                pos = object_position(name)
            except (RuntimeError, ValueError, subprocess.TimeoutExpired):
                continue
            place = where(*pos)
            print(f"{name}: {place} ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            if place == color:
                correct += 1
            elif place in layout.BINS:
                wrong += 1
            else:
                other[place] = other.get(place, 0) + 1
    print(f"correct bin {correct}, wrong bin {wrong}, elsewhere {other}")


if __name__ == "__main__":
    main()
