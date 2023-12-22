from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autorace',
            executable='detector',
            name='detector',
        ),
        Node(
            package='autorace',
            executable='corrector',
            name='corrector',
        ),
        Node(
            package='autorace',
            executable='pid',
            name='pid',
        )
    ])
