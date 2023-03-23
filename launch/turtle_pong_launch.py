#! usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='ping_pong',
            executable='paddles',
            name='sim'
        ),
        Node(
            package='ping_pong',
            executable='ball',
            name='sim'
        ),
        # Node(
        #     package='ping_pong',
        #     executable='key_teleop',
        #     name='sim'
        # ),

    ])