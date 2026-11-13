from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_path = LaunchConfiguration("video_path")
    return LaunchDescription([
        DeclareLaunchArgument("video_path"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robovision_images",
            executable="monocular_depth_publisher.py",
            name="monocular_depth_publisher",
            output="screen",
            parameters=[{"video_path": video_path}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("robovision_images"), "rviz", "monocular_depth.rviz"
            ])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
