#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Execute the turtle_gui with turtlesim node.']),

        Node(
            namespace='turtle1',
            package='turtle_gui',
            executable='turtle_gui',
            name='turtle_gui',
            output='screen'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen')
    ])
