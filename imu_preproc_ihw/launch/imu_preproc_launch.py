from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('imu_preproc_ihw')
    cfg = os.path.join(pkg, 'config', 'imu_preproc.yaml')

    imu_preproc_node = Node(
        package='imu_preproc_ihw',
        executable='imu_preproc_node',
        name='imu_preproc',
        output='screen',
        parameters=[cfg],
        emulate_tty=True,
    )

    return LaunchDescription([imu_preproc_node])
