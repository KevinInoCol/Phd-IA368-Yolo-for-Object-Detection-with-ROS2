from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value= '0',
        description='Use dummy or not'
    )
    dummy = LaunchConfiguration('dummy')
    return LaunchDescription([
    
    	dummy_arg,
        Node(
            package='ia368_pkg',
            namespace='',
            executable='pos_control_node',
            name='pos_control_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='',
            executable='vel_node',
            name='vel_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='',
            executable='target_node',
            name='target_node',
            output='screen'
        )
    ])
