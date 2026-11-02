"""Layout of the conveyor sorting cell, shared by conveyor_scene.py and conveyor_sorter.py.

All positions are in the world frame [m]. The conveyor model itself is placed by
launch/ur_gripper_sim.launch.py (scene:=conveyor) and its size is in models/conveyor.sdf.
"""

BELT_TOP = 0.20
BELT_X = 0.50            # belt centre line
SENSOR_Y = 0.0           # light curtain = pick position
SPAWN_Y = -0.65          # where new items are dropped on the belt
BELT_SPEED = 0.1         # must match <speed> in models/conveyor.sdf
ITEM_SPACING = 0.25      # distance between items on the belt

# colour label -> bin centre (x, y) on the floor behind the robot
BINS = {
    "red": (-0.40, -0.30),
    "green": (-0.45, 0.0),
    "blue": (-0.40, 0.30),
}
# hold bin (보류함): items the robot cannot or should not sort go here
REJECT = (0.05, -0.45)

# Gazebo material for each label
GAZEBO_COLORS = {"red": "Red", "green": "Green", "blue": "Blue"}

# fault mode (launch faults:=true): share of each item kind, and gripper fault rates
FAULT_MIX = {"normal": 0.55, "long": 0.20, "pair": 0.15, "unknown": 0.10}
SLIP_RATE = 0.10   # grasp comes up empty
DROP_RATE = 0.05   # held item falls out during transport
UNKNOWN_COLOR = "Purple"
