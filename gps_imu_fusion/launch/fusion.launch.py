from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # EKF 파라미터 파일 경로
    ekf_yaml = PathJoinSubstitution([
        FindPackageShare('gps_imu_fusion'), 'config', 'ekf_local.yaml'
    ])

    return LaunchDescription([
        # ──────────────────────────────────────────────────────────────
        # ① NavSatFix + IMU → ENU(UTM) 변환  (navsat_transform_node)
        #    • 입력  :  /imu/processed  ,  /ublox_gps_node/fix
        #    • 출력  :  /gps/odom  (Odometry, frame: reference → gps_antenna)
        # ──────────────────────────────────────────────────────────────
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[{
                'frequency': 20.0,
                'base_link_frame': 'gps_antenna',
                'world_frame':  'reference',
                'use_sim_time': True
            }],
            remappings=[
                ('/imu/data', '/imu/processed'),
                ('/gps/fix',  '/ublox_gps_node/fix')
            ],
            output='screen'
        ),

        # ──────────────────────────────────────────────────────────────
        # ② EKF  (GPS + IMU 퓨전)
        #    • 입력 : /imu/processed  ,  /gps/odom
        #    • 출력 : /odometry/filtered   + TF(reference → gps_antenna)
        # ──────────────────────────────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_yaml],
            output='screen'
        ),

        # ──────────────────────────────────────────────────────────────
        # ③ imu_link  →  gps_antenna  (정적 TF)   ― IMU 센서 위치·방향
        # ──────────────────────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=[
                '0', '0', '0',        # x  y  z  (m)
                '0', '0', '0', '1',   # qx qy qz qw (no rotation)
                'gps_antenna', 'imu_link'   # parent  child
            ]
        ),

        # ──────────────────────────────────────────────────────────────
        # ④ gps       →  gps_antenna  (정적 TF)   ― GPS Fix frame 맞춤
        # ──────────────────────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf',
            arguments=[
                '0', '0', '0',        # x  y  z
                '0', '0', '0', '1',   # qx qy qz qw
                'gps_antenna', 'gps'  # parent  child
            ]
        ),
    ])
