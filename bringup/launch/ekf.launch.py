import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        # 2. Chạy EKF Robot Localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path],
            remappings=[('/odometry/filtered', '/filtered_odom')] 
        ),
        
        # 3. Phát TF tam cho IMU 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'imu_link']
        ),
    ])