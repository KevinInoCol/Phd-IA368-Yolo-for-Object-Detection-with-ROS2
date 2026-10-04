"""Full pipeline:
Kinect RGB-D -> YOLO detection -> 3D position -> persistent object map -> potential field -> wheels.

kinect_node and tf_node are the course nodes (ia368_pkg); kinect_node also starts the simulation.
RViz opens with the persistent map, live detections, TF and the YOLO image (rviz:=false to skip it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('yolo_pf'), 'rviz', 'yolo_pf.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        Node(package='ia368_pkg', executable='kinect_node', name='kinect_node', output='screen'),
        Node(package='ia368_pkg', executable='tf_node', name='tf_node', output='screen'),
        Node(package='yolo_pf', executable='velocity_node', name='velocity_node', output='screen'),
        Node(package='yolo_pf', executable='detector_node', name='detector_node', output='screen'),
        Node(package='yolo_pf', executable='object_map_node', name='object_map_node', output='screen'),
        Node(package='yolo_pf', executable='potential_field_node', name='potential_field_node', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
