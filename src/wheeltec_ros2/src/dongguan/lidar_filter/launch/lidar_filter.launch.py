import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

"""
    Launches the lidar filter. Currently no paramters are given.
    Maybe in the future it will have paramters to set the blindspots.
"""


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="lidar_filter",
                executable="lidar_filter",
                parameters=[{}],
            ),
        ]
    )
