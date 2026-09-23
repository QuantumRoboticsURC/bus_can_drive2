#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios
    bus_can_drive_dir = get_package_share_directory('bus_can_drive2')

    # Archivo de parametros
    params_file = os.path.join(
        bus_can_drive_dir, 'config', 'drive_can_tank_params_copy.yaml'
    )

    bus_can_drive = Node(
        package='bus_can_drive2',
        executable='drive_only_can_tank',
        name='drive_ctre',   # debe coincidir con el nombre del nodo en el YAML
        output='screen',
        parameters=[params_file]
    )


    return LaunchDescription([
        bus_can_drive,

    ])
