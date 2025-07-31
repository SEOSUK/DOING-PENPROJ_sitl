from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown
import os
import yaml

def load_global_params(config_file):
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config.get('global_config', {}).get('ros__parameters', {})

def load_node_params(config_file, node_name):
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    params = config.get(node_name, {}).get('ros__parameters', {})
    print(f"[DEBUG] Loaded params for {node_name}: {params}")
    return params

def launch_setup(context, *args, **kwargs):
    test_pkg_path = get_package_share_directory('test_pkg')
    crazyflie_pkg_path = get_package_share_directory('crazyflie')

    config_file = os.path.join(test_pkg_path, 'config', 'config.yaml')
    urdf_path = os.path.join(test_pkg_path, 'models', 'model.urdf')
    rviz_config_file = os.path.join(test_pkg_path, 'config', 'config.rviz')

    global_params = load_global_params(config_file)
    simulation_enabled = global_params.get('Simulation', False)

    backend_value = LaunchConfiguration('backend')

    launch_items = []

    # Crazyflie backend 포함 (sim일 경우만)
    if simulation_enabled:
        crazyflie_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(crazyflie_pkg_path, 'launch', 'launch.py')
            ),
            launch_arguments=[('backend', backend_value)]
        )
        launch_items.append(crazyflie_launch)

    # cf_communicator vs cf_communicator_sim 선택 실행
    cf_comm_node_name = 'cf_communicator_sim' if simulation_enabled else 'cf_communicator'
    cf_comm_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='test_pkg',
                executable=cf_comm_node_name,
                name=cf_comm_node_name,
                parameters=[load_node_params(config_file, cf_comm_node_name)],
                output='screen',
                on_exit=Shutdown()
            )
        ]
    )
    launch_items.append(cf_comm_node)

    # wrench node
    if simulation_enabled:
        wrench_node = Node(
            package='test_pkg',
            executable='wrench_observer',
            name='wrench_observer',
            parameters=[load_node_params(config_file, 'wrench_observer')],
            output='screen'
        )
    else:
        wrench_node = Node(
            package='test_pkg',
            executable='wrench_observer',
            name='wrench_observer',
            parameters=[load_node_params(config_file, 'wrench_observer')],
            output='screen'
        )

    # robot_description 설정
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()
    else:
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 나머지 delayed 노드
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='test_pkg',
                executable='su_fkik',
                name='su_fkik',
                parameters=[
                    global_params,
                    load_node_params(config_file, 'su_fkik')
                ],
                output='screen'
            ),
            Node(
                package='test_pkg',
                executable='su_rviz',
                name='su_rviz',
                parameters=[load_node_params(config_file, 'su_rviz')],
                output='screen'
            ),
            Node(
                package='test_pkg',
                executable='trajectory_generator',
                name='trajectory_generator',
                parameters=[load_node_params(config_file, 'trajectory_generator')],
                output='screen'
            ),
            wrench_node,
            Node(
                package='test_pkg',
                executable='data_logging',
                name='data_logging',
                parameters=[load_node_params(config_file, 'data_decryptor')],
                output='screen'
            ),
            robot_state_publisher_node,
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

    launch_items.append(delayed_nodes)
    return launch_items

def generate_launch_description():
    backend_arg = DeclareLaunchArgument('backend', default_value='cflib')
    return LaunchDescription([
        backend_arg,
        OpaqueFunction(function=launch_setup)
    ])
