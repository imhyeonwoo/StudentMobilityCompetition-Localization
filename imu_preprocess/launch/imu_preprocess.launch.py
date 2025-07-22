from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_preprocess',
            executable='imu_preprocess_node',
            name='imu_preprocess_node',
            parameters=['config/default.yaml'],
            remappings=[('/imu/data', '/imu/data')]  # 필요 시 /imu/data_raw 로 변경
        )
    ])
