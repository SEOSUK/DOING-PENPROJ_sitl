from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_global_params(config_file):
    """config.yaml 내 global_config > ros__parameters 블록 로드"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config.get('global_config', {}).get('ros__parameters', {})


def generate_launch_description():
    # 📦 패키지 경로
    test_pkg_path = get_package_share_directory('test_pkg')
    crazyflie_pkg_path = get_package_share_directory('crazyflie')

    # 🗂 경로 설정
    config_file = os.path.join(test_pkg_path, 'config', 'config.yaml')
    urdf_path = os.path.join(test_pkg_path, 'models', 'model.urdf')
    rviz_config_file = os.path.join(test_pkg_path, 'config', 'config.rviz')

    # 📥 config.yaml에서 global 파라미터 불러오기
    global_params = load_global_params(config_file)

    # 🛠 launch argument
    backend_arg = LaunchConfiguration('backend', default='cflib')

    # 🚀 Simulation이 True일 경우에만 crazyflie launch 실행
    crazyflie_launch = None
    if global_params.get('Simulation', False):
        crazyflie_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(crazyflie_pkg_path, 'launch', 'launch.py')
            ),
            launch_arguments={'backend': backend_arg}.items()
        )

    # ⏱ 2초 지연 후 실행할 노드들
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[

            Node(
                package='test_pkg',
                executable='su_fkik',
                name='su_fkik',
                parameters=[config_file, global_params],
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
                package='test_pkg',
                executable='trajectory_generator',
                name='trajectory_generator',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='wrench_bridge',
                name='wrench_bridge',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='test_pkg',
                executable='data_logging',
                name='data_logging',
                parameters=[config_file],
                output='screen'
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': open(urdf_path).read()}],
                output='screen'
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
        ]
    )

    # 📦 Launch 목록 구성 (Simulation 조건에 따라 crazyflie_launch 포함 여부 결정)
    launch_items = []

    if crazyflie_launch:
        launch_items.append(crazyflie_launch)

    launch_items.append(delayed_nodes)

    return LaunchDescription(launch_items)
