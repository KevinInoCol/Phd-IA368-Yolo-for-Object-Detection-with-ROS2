from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'qos':1,
          'subscribe_odom_info':False,
          'subscribe_depth':True,
          'Grid/FromDepth':True,
          'approx_sync':True}]
          #'wait_for_transform': "0,5"}]    # if tfs are really slow

    remappings=[
          ('/myRobot/odom', '/odom')]
    
    return LaunchDescription([
        Node(
            package='ia368_pkg',
            executable='kinect_node_rtabmap_rgb',
            name='kinect_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='odom_node',
            name='odom_node',
            remappings=remappings,
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            executable='tf_node_rtabmap_rgb',
            output='screen'
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            arguments=['-d']
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "base_link", "kinect"])
    ])