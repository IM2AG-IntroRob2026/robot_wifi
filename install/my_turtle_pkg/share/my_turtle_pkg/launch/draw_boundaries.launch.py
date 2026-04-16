from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='my_turtle_pkg',
            executable='draw_boundaries_node',
            name='draw_boundaries',
            output='screen'
        )
    ])