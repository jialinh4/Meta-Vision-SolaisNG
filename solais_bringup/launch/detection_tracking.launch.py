from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='solais_detection',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='solais_tracking',
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        ),
        Node(
            package='solais_communicator',
            executable='communicator',
            name='uart_communicator',
            output='screen'
        ),
    ])

