from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_and_place',
            executable='detect',
            name='detect'
        ),
        Node(
            package='pick_and_place',
            executable='move_to_point',
            name='move_to_point'
        ),
    ])
