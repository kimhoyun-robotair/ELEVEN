"""Start RViz and optionally the textual monitor. Simulator runs separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = Path(get_package_share_directory("amr_tools")) / "rviz" / "amr.rviz"
    return LaunchDescription([
        DeclareLaunchArgument("monitor", default_value="true"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(rviz)],
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="amr_tools", executable="monitor", output="screen",
             condition=IfCondition(LaunchConfiguration("monitor"))),
    ])
