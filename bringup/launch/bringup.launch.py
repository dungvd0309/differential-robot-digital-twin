#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path package 
    package_name = 'bringup' 

    # Node odom publisher
    odom_node = Node(
        package=package_name,
        executable='odom_publisher', 
        name='odom_publisher',
        output='screen'
    )

    # Node TF broadcaster
    tf_node = Node(
        package=package_name,
        executable='tf_odom', 
        name='odom_tf_broadcaster',
        output='screen'
    )

    # encoders_publisher node
    encoders_publisher = Node(
        package=package_name,
        executable='encoders_publisher',
        name='encoders_publisher',
        output='screen'
    )
    

    ld = LaunchDescription()
    ld.add_action(odom_node)
    # ld.add_action(tf_node)
    ld.add_action(encoders_publisher)

    return ld
