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

    # EKF Robot Localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_path],
        remappings=[('/odometry/filtered', '/filtered_odom')] 
    )
    
    # Phát TF tam cho IMU 
    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'imu_link']
    )


    ld = LaunchDescription()
    ld.add_action(ekf_node)
    ld.add_action(imu_tf_node)

    return ld