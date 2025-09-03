from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_visual_share = get_package_share_directory('px4_visual')
    drone_urdf_path = os.path.join(px4_visual_share, 'models', 'model.urdf')
    rviz_config_path = os.path.join(px4_visual_share, 'config', 'config.rviz')

    # URDF 로드
    with open(drone_urdf_path, 'r') as f:
        robot_description = f.read()

    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='drone_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args,
        ),
        Node(
            package='px4_visual',
            executable='px4_rviz',
            name='px4_rviz',
            output='screen',
        ),
    ])

