from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dollar_bill',
            executable='pos_from_pixel',
            name='pos_from_pixel'
        ),
        Node(
            package='dollar_bill',
            executable='move_to_point',
            name='move_to_point'
        ),
        Node(
            package='dollar_bill',
            executable='image_to_pixel',
            name='image_to_pixel'
        ),
        Node(
            package='dollar_bill',
            executable='run',
            name='run'
        ),
    ])
