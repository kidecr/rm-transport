from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    transport_node = Node(
        package='transport',
        executable='transport_node',
        name='transport',
        output='log'
    )
    keyboard_control_node = Node(
        package='transport',
        executable='KeyboardControl',
        name='KeyboardControl',
        output='screen'
    )
    return LaunchDescription([transport_node, keyboard_control_node])
