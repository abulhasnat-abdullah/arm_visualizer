import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_visualizer',
            executable='joint_visualizer',
            name='arm_visualizer_node',
            output='screen',
        ),
    ])
