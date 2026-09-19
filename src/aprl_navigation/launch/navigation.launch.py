from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("aprl_navigation"))
    params = LaunchConfiguration("params_file")
    sim_time = {"use_sim_time": True}
    nodes = []
    for package, executable in [
        ("nav2_map_server", "map_server"),
        ("nav2_amcl", "amcl"),
        ("nav2_planner", "planner_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]:
        extra = [{"yaml_filename": LaunchConfiguration("map")}] if executable == "map_server" else []
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[params, sim_time, *extra], output="screen",
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ))
    nodes.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[sim_time, {"autostart": True, "bond_timeout": 8.0,
                    "node_names": ["map_server", "amcl", "planner_server", "controller_server",
                                   "behavior_server", "bt_navigator"]}],
    ))
    nodes.append(Node(
        package="aprl_navigation", executable="command_mux", name="aprl_command_mux",
        parameters=[params], output="screen",
    ))
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=str(share / "maps/office_floor1.yaml")),
        DeclareLaunchArgument("params_file", default_value=str(share / "config/nav2.yaml")),
        *nodes,
    ])
