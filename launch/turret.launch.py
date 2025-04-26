import os
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    # Path to YAML config file for parameters
    param_config_dir = os.path.join(
        get_package_share_directory("rotating_lidar_ctrl"), "config", "config.yaml"
    )

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="rotating_lidar_ctrl",
                executable="turret",
                output="screen",
                parameters=[param_config_dir],  # Load parameters from the YAML file
            ),
        ]
    )
