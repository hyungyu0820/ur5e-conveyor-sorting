"""Layout of the conveyor sorting cell, shared by conveyor_scene.py and conveyor_sorter.py.

All positions are in the world frame [m]. The conveyor model itself is placed by
launch/ur_gripper_sim.launch.py (scene:=conveyor) and its size is in models/conveyor.sdf.
"""

BELT_TOP = 0.20
BELT_X = 0.50            # belt centre line
SENSOR_Y = 0.0           # light curtain = pick position
SPAWN_Y = -0.65          # where new items are dropped on the belt
BELT_SPEED = 0.05        # default belt speed, launch belt_speed:=
ITEM_SPACING = 0.50      # default distance between items on the belt, launch item_spacing:=

# colour label -> bin centre (x, y) on the floor beside the robot (short moves from the belt)
BINS = {
    "red": (0.15, 0.45),
    "green": (-0.15, 0.45),
    "blue": (0.15, -0.45),
}
# hold bin (보류함): items the robot cannot or should not sort go here
REJECT = (-0.15, -0.45)

# fixed camera above the upstream part of the belt (models/belt_camera.sdf, spawned by the launch file)
# looks straight down: image right = world -y (downstream is to the left), image down = world -x
BELT_CAMERA = (0.50, -0.45, 0.85)
BELT_CAMERA_HFOV = 0.70  # [rad]
BELT_CAMERA_SIZE = (640, 480)

# Gazebo material for each label
GAZEBO_COLORS = {"red": "Red", "green": "Green", "blue": "Blue"}

# fault mode (launch faults:=true): share of each item kind, and gripper fault rates
FAULT_MIX = {"normal": 0.55, "long": 0.20, "pair": 0.15, "unknown": 0.10}
SLIP_RATE = 0.10   # grasp comes up empty
DROP_RATE = 0.05   # held item falls out during transport
UNKNOWN_COLOR = "Purple"
