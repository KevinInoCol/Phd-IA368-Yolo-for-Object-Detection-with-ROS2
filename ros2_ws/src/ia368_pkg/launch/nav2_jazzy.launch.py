from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': 'config/nav2_params_jazzy.yaml'
        }.items()
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ia368_pkg'),
                    'launch',
                    'slam_toolbox.launch.py'
                ])
            ])
        ),
        Node(
            package='ia368_pkg',
            executable='ground_truth_node',
            output='screen'
        ),
        TimerAction(
            actions = [nav2_launch], period = 10.0)
    ])
