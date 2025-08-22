from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('gps_imu_fusion')
    imu_preprocess_pkg = FindPackageShare('imu_preprocess')
    
    # 파라미터 파일 경로
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'fusion_params.yaml'
    ])
    
    # Calibration 파일 경로 (기본값)
    default_calib_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'improved_imu_calibration.json'
    ])
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'imu_calibration_file',
            default_value=default_calib_file,
            description='Path to the IMU calibration JSON file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # EKF Fusion Node
        Node(
            package='gps_imu_fusion',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'imu_calibration_file': LaunchConfiguration('imu_calibration_file')
                }
            ],
            remappings=[
                ('imu/processed', '/imu/processed'),
                ('/ublox_gps_node/fix', '/ublox_gps_node/fix'),
                ('/ublox_gps_node/fix_velocity', '/ublox_gps_node/fix_velocity')
            ]
        ),
        
        # Static transform: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])