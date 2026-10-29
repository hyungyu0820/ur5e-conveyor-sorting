# UR + parallel gripper + wrist camera in Gazebo Classic.
# Reuses ur_simulation_gazebo/ur_sim_control.launch.py and only swaps in this package's
# description, controller config and RViz config.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="Type/series of used UR robot."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument("spawn_objects", default_value="true", description="Spawn the scene objects?"),
        DeclareLaunchArgument(
            "scene", default_value="table",
            description="table: table + one cube; conveyor: conveyor belt, bins and an endless item feeder",
        ),
    ]

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gazebo"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": "true",
            "runtime_config_package": "ur_pick_place",
            "controllers_file": "ur_gripper_controllers.yaml",
            "description_package": "ur_pick_place",
            "description_file": "ur_gripper.urdf.xacro",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # scene objects (positions must match scripts/pick_place.py and scripts/conveyor_layout.py)
    def scene_is(name):
        return IfCondition(PythonExpression([
            "'", LaunchConfiguration("spawn_objects"), "' == 'true' and '",
            LaunchConfiguration("scene"), "' == '", name, "'",
        ]))

    def spawn(name, x, y, z, scene):
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name=f"spawn_{name}",
            arguments=[
                "-entity", name,
                "-file", PathJoinSubstitution([FindPackageShare("ur_pick_place"), "models", f"{name}.sdf"]),
                "-x", str(x), "-y", str(y), "-z", str(z),
            ],
            output="screen",
            condition=scene_is(scene),
        )

    objects = [
        spawn("table", 0.5, 0.0, 0.0, "table"),
        spawn("cube", 0.45, -0.25, 0.22, "table"),
        spawn("conveyor", 0.5, -0.15, 0.0, "conveyor"),
        # colour bins + endless items on the belt
        Node(
            package="ur_pick_place",
            executable="conveyor_scene.py",
            output="screen",
            condition=scene_is("conveyor"),
        ),
    ]

    return LaunchDescription(declared_arguments + [ur_sim, gripper_controller_spawner] + objects)
