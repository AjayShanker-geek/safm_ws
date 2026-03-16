import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('conversion_pkg'), 'params', 'conversion.yaml'
    )

    return LaunchDescription([
        Node(
            package='conversion_pkg',
            executable='sensor_combined_to_imu_node',
            name='sensor_combined_to_imu',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='conversion_pkg',
            executable='odom_to_visual_odom',
            name='odom_to_visual_odom',
            output='screen',
            parameters=[params_file],
        ),
    ])
