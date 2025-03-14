#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the nodes to be launched

    # Node 1: Bio Decoder
    #bio_controller = Node(
    #    package='bio_package',
    #    executable='bio_controller',
    #    name='bio_controller',
    #    output='screen',
    #    parameters=[
    #    ]
    #)
    joining_node = Node(
        package='bio_package',
        executable='joining_node',
        name='joining_node',
        output='screen',
        parameters=[
        ]
    )
    node_mcu_interface = Node(
        package='bio_package',
        executable='node_mcu_interface',
        name='node_mcu_interface',
        output='screen',
        parameters=[
        ]
    )
    # Node 2: Relay Controller
    relay = Node(
        package='bio_package',
        executable='relay',
        name='relay',
        output='screen',
        parameters=[
        ]
    )

    # Node 3: cameras
    '''
    cameras = Node(
        package='bio_package',
        executable='cameras',
        name='cameras',
        output='screen',
        parameters=[
        ]
    )
    '''

    # Node 4: pid
    pid = Node(
        package='bio_package',
        executable='pid',
        name='pid',
        output='screen',
        parameters=[
        ]
    )

    # Create the launch description and populate
    return LaunchDescription([
        relay,
        pid,
        joining_node,
        node_mcu_interface
    ])
