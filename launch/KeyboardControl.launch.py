from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    transport_node = Node(
        package='transport',
        executable='transport_node',
        name='transport',
        output='log'
    )
    keyboard_control_node = Node(
        package='transport',
        executable='KeyboardControlROS',
        name='KeyboardControlROS',
        output='screen',
        arguments=['--stdio-path', os.ttyname(0)]
    )
    
    return LaunchDescription([transport_node, keyboard_control_node])
