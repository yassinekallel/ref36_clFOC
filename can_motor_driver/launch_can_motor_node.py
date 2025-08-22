# Example launch file for can_motor_driver
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_motor_driver',
            executable='can_motor_node',
            name='can_motor_node',
            parameters=[
                {'can_id': 1},
                {'ack_id': 190},
                {'can_interface': 'can0'},
            ],
            output='screen',
        ),
    ])
