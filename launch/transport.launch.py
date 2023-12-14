from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    transport_node = Node(
        package='transport',
        executable='transport_node',
        name='transport',
        output='screen'
    )
    return LaunchDescription([transport_node])
