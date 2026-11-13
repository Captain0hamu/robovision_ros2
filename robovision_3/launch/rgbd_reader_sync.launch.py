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
    config = str(
        Path(get_package_share_directory("robovision_rgbd")) / "rviz" / "rgbd_reader_sync.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robovision_rgbd",
            executable="rgbd_reader_sync",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", config],
            parameters=[{"use_sim_time": sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
