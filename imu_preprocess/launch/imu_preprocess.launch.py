from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_yaml = PathJoinSubstitution([
        FindPackageShare('imu_preprocess'), 'config', 'default.yaml'
    ])

    return LaunchDescription([
        Node(
            package   = 'imu_preprocess',
            executable= 'imu_preprocess_node',
            name      = 'imu_preprocess_node',
            parameters= [default_yaml],
            output    = 'screen',                       # ← 로그를 콘솔로
            remappings= [('/imu/data', '/imu/data')]    # 필요 시 변경
        )
    ])
