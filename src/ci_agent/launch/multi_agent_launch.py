# ~/ros2_ws/src/ci_agent/launch/multi_agent_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ci_agent',
            executable='ci_agent',
            name='ci_agent',
            output='screen'
        ),
        Node(
            package='bi_agent',
            executable='bi_agent',
            name='bi_agent',
            output='screen'
        ),
        Node(
            package='visitor_agent',
            executable='visitor_agent',
            name='visitor_agent',
            output='screen'
        ),
    ])
