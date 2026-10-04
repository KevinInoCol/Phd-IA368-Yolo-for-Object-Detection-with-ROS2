"""Publish the robot's CoppeliaSim world pose in the map frame."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__('gt_node')
        self.publisher = self.create_publisher(PoseStamped, 'myRobot/gt_pose', 10)
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.robot = self.sim.getObject('/myRobot')
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('Publishing ground truth pose in map frame')

    def publish_pose(self):
        try:
            position = self.sim.getObjectPosition(self.robot, self.sim.handle_world)
            orientation = self.sim.getObjectOrientation(self.robot, self.sim.handle_world)
            # The Kinect is mounted on local +Y, which is the robot's front.
            # Rotate the reported frame so +X points toward that front.
            body_rotation = Rotation.from_euler('xyz', orientation)
            forward_rotation = Rotation.from_euler('z', math.pi / 2)
            quaternion = (body_rotation * forward_rotation).as_quat()
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = quaternion
            self.publisher.publish(msg)
        except Exception as error:
            self.get_logger().error(f'Cannot read ground truth pose: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GroundTruthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
