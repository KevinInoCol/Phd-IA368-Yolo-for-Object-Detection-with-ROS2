import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'myRobot/odom', 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient() # put your IP/hostname here, like 'host=meu_PC'
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim successfully.')

            # Get robot and motor handles
            self.robotHandle = self.sim.getObject('/myRobot')
            self.ground_truth = self.sim.getObjectHandle("/myRobot/Ground_truth")

        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # Timer to publish odometry at 10Hz
        self.timer = self.create_timer(0.1, self.publish_odom)

        # Populate odometry msg with constant data
        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.get_logger().info('ROS2 → CoppeliaSim Odom node started.')

    def publish_odom(self):
        try:
            # Get myRobot position and orientation (quaternion) from CoppeliaSim
            pos_gt = self.sim.getObjectPosition(self.ground_truth, -1)
            quat_gt = self.sim.getObjectQuaternion(self.ground_truth, -1)

            # Update odometry msg with pose info
            self.odom_msg.pose.pose.position.x = pos_gt[0]
            self.odom_msg.pose.pose.position.y = pos_gt[1]
            self.odom_msg.pose.pose.position.z = 0.0
            self.odom_msg.pose.pose.orientation = Quaternion(
                x=quat_gt[0], y=quat_gt[1], z=quat_gt[2], w=quat_gt[3]
            )

            # Publish
            self.odom_publisher.publish(self.odom_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()