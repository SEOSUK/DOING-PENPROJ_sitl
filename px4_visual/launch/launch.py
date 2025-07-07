from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    px4_visual_share = FindPackageShare('px4_visual')
    drone_urdf_path = PathJoinSubstitution([
        px4_visual_share,
        'models', 'model.urdf'
    ])

    rviz_config_path = PathJoinSubstitution([
        px4_visual_share,
        'config', 'config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='drone_state_publisher',
            parameters=[{
                'robot_description': Command(['cat ', drone_urdf_path])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
        Node(
            package='px4_visual',
            executable='px4_rviz',
            name='px4_rviz',
            output='screen',
        ),
        Node(
            package='px4_visual',
            executable='px4_player',
            name='px4_player',
            output='screen',
        ),
    ])

