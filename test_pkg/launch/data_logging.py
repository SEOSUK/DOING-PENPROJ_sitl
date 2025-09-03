# test_pkg/launch/launch.py
import os
import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup_launch(context, *args, **kwargs):
    base_dir = LaunchConfiguration('base_dir').perform(context)
    topic    = LaunchConfiguration('topic').perform(context)

    # base_dir 보장
    os.makedirs(base_dir, exist_ok=True)

    # 폴더명: YYYYMMDD-HHMMSS
    ts = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    bag_dir = os.path.join(base_dir, ts)

    # 데이터 로거 노드 (실행 파일명은 패키지에서 빌드한 이름으로 맞춰주세요)
    data_node = Node(
        package='test_pkg',
        executable='data_logging',   # ← 너의 실행 파일명으로 맞춰!
        name='data_logging',
        output='screen'
    )

    # rosbag record
    record_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', topic, '-o', bag_dir],
        output='screen'
    )

    # record 종료 후 CSV로 변환
    convert_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'convert', '-i', bag_dir, '-f', 'csv'],
        output='screen'
    )
    on_record_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=record_proc,
            on_exit=[convert_proc]
        )
    )

    return [data_node, record_proc, on_record_exit]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_dir',
            default_value='/home/seosuk/sitl_ws/src/test_pkg/bag',
            description='ros2 bag 기록을 저장할 기본 폴더'
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/data_logging_msg',
            description='record할 토픽'
        ),
        OpaqueFunction(function=setup_launch),
    ])

