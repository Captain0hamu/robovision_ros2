from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    config = str(Path(get_package_share_directory("rgbd_cloud_demo")) / "rviz" / "cloud.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="rgbd_cloud_demo", executable="rgbd_cloud", output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
            package="rviz2", executable="rviz2", arguments=["-d", config],
            parameters=[{"use_sim_time": sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
