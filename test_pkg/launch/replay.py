from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction  # ✅ 반드시 필요
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('test_pkg'), 'config', 'config.yaml')
    pkg_path = get_package_share_directory('test_pkg')
    urdf_path = os.path.join(pkg_path, 'models', 'model.urdf')
    rviz_config_file = os.path.join(pkg_path, 'config', 'config.rviz')

    return LaunchDescription([
        # 즉시 실행되는 노드
        Node(
            package='test_pkg',
            executable='data_decryptor',
            name='data_decryptor',
            parameters=[config_file],
            output='screen'
        ),

        # 2초 후 실행될 노드들
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='test_pkg',
                    executable='trajectory_generator',
                    name='trajectory_generator',
                    parameters=[config_file],
                    output='screen',
                    remappings=[
                        ('/pen/EE_des_xyzYaw', '/pen/EE_des_xyzYaw_unused')
                    ]
                ),
                Node(
                    package='test_pkg',
                    executable='su_fkik',
                    name='su_fkik',
                    parameters=[config_file],
                    output='screen'
                ),
                Node(
                    package='test_pkg',
                    executable='su_rviz',
                    name='su_rviz',
                    parameters=[config_file],
                    output='screen'
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{'robot_description': open(urdf_path).read()}],
                    output='screen'
                ),
            ]
        )
    ])

