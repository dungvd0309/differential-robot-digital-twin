#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path package nếu cần
    package_name = 'bringup'  # Tên package của bạn

    # Node odom publisher
    odom_node = Node(
        package=package_name,
        executable='odom',      # executable được khai báo trong setup.py entry_points
        name='diff_drive_odom',
        output='screen',
        parameters=[{
            'wheel_radius': 0.0325,
            'wheel_separation': 0.2336,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )

    # Node TF broadcaster
    tf_node = Node(
        package=package_name,
        executable='tf_odom',   # executable được khai báo trong setup.py entry_points
        name='odom_tf_broadcaster',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(odom_node)
    ld.add_action(tf_node)

    return ld

