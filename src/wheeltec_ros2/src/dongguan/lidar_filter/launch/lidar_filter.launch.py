import os
from pathlib import Path

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    Turn_on_dir = get_package_share_directory("turn_on_wheeltec_robot")
    Turn_on_launch_dir = os.path.join(Turn_on_dir, "launch")

    Lidar_filter_dir = get_package_share_directory("lidar_filter")
    Lidar_filter_src_dir = os.path.join(Lidar_filter_dir, "src")

    Turn_on_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(Turn_on_launch_dir, "wheeltec_lidar.launch.py")
        ),
    )

    Turn_on_lidar_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(Lidar_filter_src_dir, "lidar_filter_node.py")
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    """
    Please select your lidar here, options include:
    Lsm10_m10p_uart、Lsm10_m10p_net、Lsm10_m10p_uart、Lsm10_m10p_net、Lsn10、Ld14、Ld06.
    1.If you are using LS* lidar (including lsn10, lsm10*), please don't forget to
    modify the tf conversion parameters of robot_mode_description.launch.py
    according to the user guide file.
    2.If you are using m10 lidar, please pay attention to distinguish whether it is m10p or not.
    """
    ld.add_action(Turn_on_lidar)
    ld.add_action(Turn_on_lidar_filter)

    return ld
