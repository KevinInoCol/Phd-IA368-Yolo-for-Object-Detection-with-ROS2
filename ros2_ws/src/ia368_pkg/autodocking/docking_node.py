import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *


class DockingSetter(Node):
    def __init__(self):
        super().__init__('docking_Setter')

        # Subscribe to the docking mode topic
        self.subscription = self.create_subscription(
            Int32,
            '/myRobot/docking_mode',
            self.docking_mode_callback,
            10
        )
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        self.get_logger().info('ROS2 → CoppeliaSim Docking setter started.')

    def docking_mode_callback(self, msg):
        docking_state = msg.data
        try:
            self.sim.setInt32Signal(str(self.robotHandle)+"Docking", docking_state)
            self.get_logger().info(f"Setting docking state to: {docking_state}")
        except Exception as e:
            self.get_logger().error(f'Failed to set docking state in CoppeliaSim: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DockingSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()