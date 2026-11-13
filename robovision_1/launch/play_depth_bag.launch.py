from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    return LaunchDescription([
        DeclareLaunchArgument("bag"),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--loop"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("robovision_images"), "rviz", "depth_bag.rviz"
            ])],
        ),
    ])
