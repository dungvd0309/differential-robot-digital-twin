import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path_sdf = os.path.join(get_package_share_directory('diff_robot'), 'urdf', 'diff_robot.urdf')
    with open(path_sdf, 'r') as rb:
        model_rb = rb.read()

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='0')
    z_pose = LaunchConfiguration('z_pose', default='0')
    yaw = LaunchConfiguration('yaw', default='0')

    world = os.path.join(
        get_package_share_directory('diff_robot'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publish_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': model_rb,
        }],
    )

    start_gazebo_ros_spawner_cmd = TimerAction(
        period=2.0,  # Delay 2 giây để robot_state_publisher publish URDF
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'dif_robot_5',
                    '-topic', '/robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-Y', yaw
                ],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publish_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld

