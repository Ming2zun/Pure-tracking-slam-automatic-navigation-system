from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_steering_angle',
            executable='ackermann_steering_angle',
            name='ackermann_steering_angle',
            output='screen',
            parameters=[{
                'reference_frame': 'base_link',
                'left_steering_link': 'front_left_steering_link',
                'right_steering_link': 'front_right_steering_link',
                'publish_rate': 20.0,
            }],
        ),
    ])
