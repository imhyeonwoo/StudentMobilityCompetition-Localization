from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Optional: temporary compatibility alias for reference frame
        DeclareLaunchArgument('publish_reference_tf', default_value='false', description='Publish map->reference identity for legacy Planning nodes'),

        # Sensor offsets relative to base_link
        DeclareLaunchArgument('os_sensor_x', default_value='-0.3'),
        DeclareLaunchArgument('os_sensor_y', default_value='0.0'),
        DeclareLaunchArgument('os_sensor_z', default_value='0.7'),

        DeclareLaunchArgument('gps_x', default_value='0.5'),
        DeclareLaunchArgument('gps_y', default_value='0.0'),
        DeclareLaunchArgument('gps_z', default_value='0.2'),

        DeclareLaunchArgument('imu_x', default_value='-0.3'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.7'),

        # map -> odom (identity until SLAM/localization owns it)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # base_link -> os_sensor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_os_sensor',
            arguments=[
                LaunchConfiguration('os_sensor_x'),
                LaunchConfiguration('os_sensor_y'),
                LaunchConfiguration('os_sensor_z'),
                '0', '0', '0',
                'base_link', 'os_sensor'
            ]
        ),

        # base_link -> gps
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=[
                LaunchConfiguration('gps_x'),
                LaunchConfiguration('gps_y'),
                LaunchConfiguration('gps_z'),
                '0', '0', '0',
                'base_link', 'gps'
            ]
        ),

        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=[
                LaunchConfiguration('imu_x'),
                LaunchConfiguration('imu_y'),
                LaunchConfiguration('imu_z'),
                '0', '0', '0',
                'base_link', 'imu_link'
            ]
        ),

        # Optional: map -> reference identity for short-term compatibility
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_reference_tf',
            condition=IfCondition(LaunchConfiguration('publish_reference_tf')),
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'reference']
        ),
    ])
