"""What is under the gripper? A look with the wrist RGB-D camera just before a grasp.

The sorter takes one frame while the tool hovers above the item (start of the descent) and only
uses the answer when the grasp then comes up empty. Everything here is plain numpy / OpenCV on one
frame, so it can be tested and tuned offline on saved frames (tools/tune_wrist_check.py).

    features = measure(rgb, depth, K, T_world_cam, expect_xy)
    kind = classify(features)   # single | long | pair | stacked | nothing | unclear

measure() turns the depth image into points in the world frame, keeps the ones on the belt near
where the item should be, and describes the top face of whatever is there: height above the belt,
length and width of its outline seen from straight above, direction of the long side, colours.
"""
import math

import cv2
import numpy as np

import conveyor_layout as layout

# rule thresholds [m]; tuned on saved frames from the simulation, see docs/exception_nonstop_report.md
PARAMS = {
    "roi": 0.11,            # half size of the square around the expected item position
    "belt_half_width": 0.095,
    "min_height": 0.015,    # points lower than this above the belt are the belt
    "top_band": 0.010,      # the top face = points within this of the highest point
    "min_cells": 40,        # 2 mm cells; fewer = nothing there
    "stack_height": 0.060,  # top face higher than this above the belt = one item on another (cube: 0.04)
    "long_length": 0.100,   # outline longer than this = long box (0.12)
    "pair_length": 0.065,   # longer than a cube (0.04) but shorter than a long box = two cubes (0.082)
    "max_width": 0.060,     # a single row of items is 0.04 wide
    "second_colour": 0.20,  # share of the top face in a second colour that makes it two items
}
CELL = 0.002
FINGERTIP_PLANE = 0.093     # [m] along the tool axis from tool0: the fingers end here
GRIPPER_BASE = 0.035        # nearer than this is the gripper body or the camera housing
FINGER_GAP = 0.038          # [m] from the tool axis along the closing direction: beyond it are the open fingers


def _rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera_in_tool(mount="raised_wide"):
    """tool0 -> camera_optical_frame, the same numbers as urdf/ur_gripper.urdf.xacro."""
    origins = {
        "side": ((0.055, 0.0, 0.02), -math.pi / 2),
        "raised": ((0.08, 0.0, -0.06), -2.089),
        "raised_wide": ((0.11, 0.0, -0.03), -2.356),
        "center": ((0.0, 0.0, 0.045), -math.pi / 2),
    }
    xyz, pitch = origins[mount]
    T = np.eye(4)
    T[:3, :3] = _rot("y", pitch) @ (_rot("z", -math.pi / 2) @ _rot("x", -math.pi / 2))
    T[:3, 3] = xyz
    return T


def colour_shares(pixels):
    """Share of red / green / blue / other among RGB pixels (N, 3), same rule as color_detector."""
    if len(pixels) == 0:
        return {}
    p = pixels.astype(np.float32)
    out, used = {}, np.zeros(len(p), bool)
    for name, c in (("red", 0), ("green", 1), ("blue", 2)):
        others = np.delete(p, c, axis=1).max(axis=1)
        m = (p[:, c] > 60) & (p[:, c] > 2.0 * others)
        out[name] = float(m.mean())
        used |= m
    out["other"] = float((~used).mean())
    return out


def measure(rgb, depth, K, T_world_cam, expect_xy, T_tool_cam=None, params=PARAMS):
    """Describe what lies on the belt around expect_xy. Returns a dict, {'seen': False} if nothing."""
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth.astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05) & (z < 1.0)
    if T_tool_cam is not None:  # drop the gripper itself (the open fingers hang into the view)
        rx, ry = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
        yt = (T_tool_cam[1, 0] * rx + T_tool_cam[1, 1] * ry + T_tool_cam[1, 2]) * z + T_tool_cam[1, 3]
        zt = (T_tool_cam[2, 0] * rx + T_tool_cam[2, 1] * ry + T_tool_cam[2, 2]) * z + T_tool_cam[2, 3]
        ok &= (zt > GRIPPER_BASE) & ((zt > FINGERTIP_PLANE) | (np.abs(yt) < FINGER_GAP))
    u, v, z = u[ok], v[ok], z[ok]
    cam = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z, np.ones_like(z)])
    world = (T_world_cam @ cam)[:3]
    colours = rgb[v, u]

    near = (np.abs(world[0] - expect_xy[0]) < params["roi"]) & (np.abs(world[1] - expect_xy[1]) < params["roi"]) \
        & (np.abs(world[0] - layout.BELT_X) < params["belt_half_width"])
    world, colours = world[:, near], colours[near]
    out = {"seen": False, "points": int(near.sum())}
    if world.shape[1] < 200:
        return out
    # belt height from the same frame: the image lags the joint angles a little while the tool descends
    flat = world[2][np.abs(world[2] - layout.BELT_TOP) < 0.03]
    belt_z = float(np.median(flat)) if len(flat) > 100 else layout.BELT_TOP
    height = world[2] - belt_z
    obj = (height > params["min_height"]) & (height < 0.11)
    out["belt_z"] = belt_z
    if obj.sum() < 30:
        return out
    top_h = float(np.percentile(height[obj], 97))
    top = obj & (height > top_h - params["top_band"])
    xy = world[:2, top]

    # outline seen from straight above, on a 2 mm grid (even density, unlike the slanted camera view)
    n = int(2 * params["roi"] / CELL) + 1
    grid = np.zeros((n, n), np.uint8)
    ix = np.clip(((xy[0] - expect_xy[0] + params["roi"]) / CELL).astype(int), 0, n - 1)
    iy = np.clip(((xy[1] - expect_xy[1] + params["roi"]) / CELL).astype(int), 0, n - 1)
    grid[iy, ix] = 1
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cells = np.argwhere(grid > 0)[:, ::-1].astype(np.float32)  # (x, y) cell indices
    out["cells"] = int(len(cells))
    if len(cells) < params["min_cells"]:
        return out
    (cx, cy), (a, b), angle = cv2.minAreaRect(cells)
    if a < b:  # make `a` the long side and `angle` its direction
        a, b, angle = b, a, angle + 90.0
    out.update(
        seen=True,
        height=top_h,
        length=float(a * CELL), width=float(b * CELL),
        angle=float(math.radians(angle) % math.pi),  # direction of the long side in the world, 0 = across the belt (x)
        centre=[float(cx * CELL - params["roi"] + expect_xy[0]), float(cy * CELL - params["roi"] + expect_xy[1])],
        colours=colour_shares(colours[top]),
    )
    # colours of the two ends of the outline, for telling two items lying side by side apart:
    # ends[0] is the end towards -axis from the centre, ends[1] the end towards +axis
    axis = (math.cos(out["angle"]), math.sin(out["angle"]))
    along = (xy[0] - out["centre"][0]) * axis[0] + (xy[1] - out["centre"][1]) * axis[1]
    out["axis"] = [float(axis[0]), float(axis[1])]
    out["ends"] = [colour_shares(colours[top][along < 0]), colour_shares(colours[top][along >= 0])]
    return out


def classify(f, params=PARAMS):
    """Name what measure() saw: single, long, pair, stacked, nothing or unclear (-> ask for a diagnosis)."""
    if not f or not f.get("seen"):
        return "nothing"
    shares = sorted((s for k, s in f["colours"].items()), reverse=True)
    named = sorted((s for k, s in f["colours"].items() if k != "other"), reverse=True)
    two_colours = len(named) > 1 and named[1] > params["second_colour"]
    if f["height"] > params["stack_height"]:
        return "stacked"
    if f["width"] > params["max_width"]:
        return "unclear"
    if f["length"] > params["long_length"]:
        return "unclear" if two_colours else "long"
    if f["length"] > params["pair_length"]:
        across = min(f["angle"], math.pi - f["angle"]) < math.radians(25)
        # two cubes always lie across the belt; a long box cut off by the image edge lies along it
        return "pair" if two_colours or across else "long"
    if two_colours or not shares:
        return "unclear"
    return "single"


def grasp_yaw_for(angle, default_yaw):
    """Tool yaw that closes the fingers across the short side of an outline whose long side points along `angle`.

    The fingers close at right angles to the tool x axis, so the tool x axis goes along the long side.
    Of the two equivalent yaws (180 degrees apart) take the one nearer the usual yaw.
    """
    yaw = angle
    while yaw - default_yaw > math.pi / 2:
        yaw -= math.pi
    while yaw - default_yaw < -math.pi / 2:
        yaw += math.pi
    return yaw
