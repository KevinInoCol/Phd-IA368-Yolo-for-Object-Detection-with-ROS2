from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ia368_pkg',
            executable='battery_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='bumper_and_velocity_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='charging_base_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='docking_node',
            output='screen'
        )
    ])
