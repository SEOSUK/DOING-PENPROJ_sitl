from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
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

    global_params = load_global_params(config_file)
    simulation_enabled = global_params.get('Simulation', False)

    backend_value = LaunchConfiguration('backend')

    launch_items = []

    if simulation_enabled:
        # ✅ Crazyflie 백엔드 실행
        crazyflie_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(crazyflie_pkg_path, 'launch', 'launch.py')
            ),
            launch_arguments=[('backend', backend_value)]
        )
        launch_items.append(crazyflie_launch)

        # ✅ cf_communicator 실행 (3초 지연)
        cf_communicator_node = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='test_pkg',
                    executable='cf_communicator',
                    name='cf_communicator',
                    parameters=[load_node_params(config_file, 'cf_communicator')],
                    output='screen',
                    on_exit=Shutdown()
                )
            ]
        )
        launch_items.append(cf_communicator_node)

    return launch_items

def generate_launch_description():
    backend_arg = DeclareLaunchArgument('backend', default_value='cflib')
    return LaunchDescription([
        backend_arg,
        OpaqueFunction(function=launch_setup)
    ])
