"""cmd_vel -> left/right wheel velocities of myRobot (differential drive).

Same idea as the course velocity_node, with myRobot's geometry: wheel radius 0.05 m,
wheel separation 0.20 m, and a negative joint velocity moves the robot forward.
Commands older than STALE seconds are replaced by zero (safety stop).
"""
import time

import rclpy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Twist
from rclpy.node import Node

WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.20
STALE = 0.5


class VelocityNode(Node):
    def __init__(self):
        super().__init__('myrobot_velocity')
        self.declare_parameter('zmq_port', 23000)
        client = RemoteAPIClient(port=self.get_parameter('zmq_port').value)
        self.sim = client.require('sim')
        self.left = self.sim.getObject('/myRobot/leftMotor')
        self.right = self.sim.getObject('/myRobot/rightMotor')
        self.v_left = self.v_right = 0.0
        self.stamp = 0.0
        self.create_subscription(Twist, '/myRobot/cmd_vel', self.cmd_callback, 10)
        self.create_timer(0.05, self.send)
        self.get_logger().info('myRobot velocity node ready')

    def cmd_callback(self, msg):
        v, w = msg.linear.x, msg.angular.z
        # inverse kinematics of the differential drive (wheel linear speeds)
        self.v_right = v + w * WHEEL_BASE / 2
        self.v_left = v - w * WHEEL_BASE / 2
        self.stamp = time.monotonic()

    def send(self):
        if time.monotonic() - self.stamp > STALE:
            self.v_left = self.v_right = 0.0
        self.sim.setJointTargetVelocity(self.left, -self.v_left / WHEEL_RADIUS)
        self.sim.setJointTargetVelocity(self.right, -self.v_right / WHEEL_RADIUS)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.v_left = node.v_right = 0.0
    node.stamp = 0.0
    try:
        node.send()
    except Exception:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
