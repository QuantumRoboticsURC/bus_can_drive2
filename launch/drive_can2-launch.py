from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bus_can_drive2',
            executable='drive_can',
            name='drive_can',
            output='screen'
        )
    ])