#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path package 
    package_name = 'bringup' 

    # Node odom publisher
    odom_publisher = Node(
        package=package_name,
        executable='odom_publisher', 
        name='odom_publisher',
        output='screen'
    )

    # encoders_publisher node
    encoders_publisher = Node(
        package=package_name,
        executable='encoders_publisher',
        name='encoders_publisher',
        output='screen'
    )

    # image_publisher node
    image_publisher = Node(
        package=package_name,
        executable='image_publisher',
        name='image_publisher',
        output='screen'
    )
    
    # Include EKF launch
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.path.dirname(__file__), 
                'ekf.launch.py'
            )
        ])
    )

    ld = LaunchDescription()
    ld.add_action(odom_publisher)
    ld.add_action(encoders_publisher)
    ld.add_action(image_publisher)
    ld.add_action(ekf_launch)

    return ld
