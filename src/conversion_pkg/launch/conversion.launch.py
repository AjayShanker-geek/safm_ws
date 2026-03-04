from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='conversion_pkg',
            executable='local_pose_conversion',
            name='local_pose_conversion',
            output='screen',
        ),
        Node(
            package='conversion_pkg',
            executable='setpoint_conversion',
            name='setpoint_conversion',
            output='screen',
        ),
    ])
