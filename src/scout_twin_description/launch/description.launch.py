"""URDF, TF and RViz for the Isaac Sim robot; no odometry TF is generated here."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    share = Path(get_package_share_directory("scout_twin_description"))
    urdf_path = Path(LaunchConfiguration("urdf").perform(context)).expanduser()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"Robot URDF does not exist: {urdf_path}")
    robot_description = ParameterValue(urdf_path.read_text(encoding="utf-8"), value_type=str)
    simulation = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            condition=IfCondition(LaunchConfiguration("publish_state")),
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": simulation,
                "publish_frequency": 50.0,
            }],
        ),
        # This optional node is only for inspecting the URDF without Isaac Sim.
        # In the default mode, Isaac Sim owns /joint_states.
        Node(
            package="joint_state_publisher", executable="joint_state_publisher",
            name="preview_joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("preview_joints")),
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": simulation, "rate": 20}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", str(share / "rviz" / "scout_twin.rviz"),
                       "-f", LaunchConfiguration("fixed_frame")],
            parameters=[{"use_sim_time": simulation}],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("scout_twin_description"))
    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=str(share / "urdf" / "scout_twin.urdf")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fixed_frame", default_value="sim_world"),
        DeclareLaunchArgument("publish_state", default_value="false",
                              description="Enable for standalone preview; APRL already publishes description and TF."),
        DeclareLaunchArgument("preview_joints", default_value="false",
                              description="Enable only without Isaac Sim joint states."),
        OpaqueFunction(function=_nodes),
    ])
