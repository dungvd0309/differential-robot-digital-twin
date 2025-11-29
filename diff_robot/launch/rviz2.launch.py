import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bringup'),
                'launch',
                'bringup.launch.py'  # tên file launch cũ
            )
        )
    )
    # --- 1. Load URDF ---
    path_urdf = os.path.join(
        get_package_share_directory('diff_robot'),
        'urdf',
        'diff_robot.urdf'
    )
    with open(path_urdf, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- 2. Robot State Publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }]
    )

    # --- 3. Joint State Publisher GUI ---
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    # --- 4. Rviz2 Node ---
    rviz_config_file = os.path.join(
        get_package_share_directory('diff_robot'),
        'rviz',
        'diff_robot.rviz'  # <-- tạo sẵn file config Rviz nếu muốn custom
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]  # nếu không có file config thì bỏ ['-d', ...]
    )

    ld = LaunchDescription()
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)
    ld.add_action(rviz_node)
    ld.add_action(bringup_launch)

    return ld

