#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios
    epos_control_dir = get_package_share_directory('epos_control')

    
    bus_can_drive = Node(
        package='bus_can_drive2',
        executable='drive_can_tank',
        name='drive_can_tank',
        output='screen'
    )
    
    epos_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(epos_control_dir, 'launch', 'epos_control_launch.py')  # o el nombre de tu launch
        )
    )
  
    return LaunchDescription([

        bus_can_drive,
        epos_launch
        
    ])