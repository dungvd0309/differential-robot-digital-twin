import os
import logging
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ĐƯỜNG DẪN ĐẾN FILE YAML BẠN VỪA TẠO
    # Hãy sửa lại đường dẫn này cho đúng vị trí file trên máy bạn
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'ekf.yaml'
    )

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    logger.info(f"Config file path: {config_file_path}")

    print(os.path.abspath(config_file_path))

    return LaunchDescription([

        # 2. Chạy EKF Robot Localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path],
            # Remap đầu ra nếu cần. Mặc định nó ra /odometry/filtered
            remappings=[('/odometry/filtered', '/odom2')] 
        ),
        
        # 3. (Tạm thời) Phát TF tĩnh cho IMU nếu chưa có URDF
        # Giả sử IMU nằm trùng tâm robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'imu_link']
        ),
    ])