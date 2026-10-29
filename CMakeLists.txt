cmake_minimum_required(VERSION 3.8)
project(ur_pick_place)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Gazebo plugin: welds the grasped object to the wrist (services /grasp/attach, /grasp/detach)
add_library(ur_grasp_attacher SHARED src/grasp_attacher.cpp)
ament_target_dependencies(ur_grasp_attacher gazebo_dev gazebo_ros rclcpp std_srvs)
# Gazebo plugin: conveyor belt with a light curtain (/conveyor/power, /conveyor/running, /conveyor/object_at_sensor)
add_library(ur_conveyor_belt SHARED src/conveyor_belt.cpp)
ament_target_dependencies(ur_conveyor_belt gazebo_dev gazebo_ros rclcpp std_msgs std_srvs)

install(TARGETS ur_grasp_attacher ur_conveyor_belt LIBRARY DESTINATION lib)

install(DIRECTORY urdf config launch rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gripper.py scripts/pick_place.py scripts/ur_kinematics.py scripts/spawn_object.py
  scripts/conveyor_layout.py scripts/conveyor_scene.py scripts/conveyor_sorter.py scripts/color_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
