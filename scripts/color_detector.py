"""Item detector for the sorting cell: finds coloured cubes in an RGB image.

This is the only perception code the sorter uses. To try another recogniser
(for example a vision-language model that returns labelled points), write a function with
the same signature and pass it to conveyor_sorter.py:

    detect(rgb: np.ndarray[H, W, 3] uint8) -> list[Detection]

Pixel coordinates are enough; the sorter looks up depth and converts to the world frame.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    label: str      # "red", "green", "blue"
    u: float        # pixel column of the item centre
    v: float        # pixel row of the item centre
    area: int       # pixel count, larger = more confident


# A pixel belongs to a colour when that channel is bright and clearly dominates the others.
# Works for Gazebo's flat Red/Green/Blue materials under the default sun.
CHANNEL = {"red": 0, "green": 1, "blue": 2}
MIN_LEVEL = 60
DOMINANCE = 2.0
MIN_AREA = 150


def detect(rgb):
    img = rgb.astype(np.float32)
    detections = []
    for label, c in CHANNEL.items():
        others = np.delete(img, c, axis=2).max(axis=2)
        mask = ((img[:, :, c] > MIN_LEVEL) & (img[:, :, c] > DOMINANCE * others)).astype(np.uint8)
        n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):  # 0 is the background
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= MIN_AREA:
                u, v = centroids[i]
                detections.append(Detection(label, float(u), float(v), area))
    return detections
