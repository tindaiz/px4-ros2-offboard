from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_visualizer = LaunchConfiguration("start_visualizer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_visualizer",
                default_value="true",
                description="Start the PX4 RViz visualization node.",
            ),
            Node(
                package="px4_offboard",
                executable="offboard_control",
                name="offboard_control",
                output="screen",
            ),
            Node(
                package="px4_offboard",
                executable="px4_visualizer",
                name="px4_visualizer",
                output="screen",
                condition=IfCondition(start_visualizer),
            ),
        ]
    )