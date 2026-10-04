import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class PublisherLaserscan(Node):
    def __init__(self):
        super().__init__('laserscan_bridge')

        # Create publisher for laserscan
        self.laserscan_publisher = self.create_publisher(LaserScan,'/myRobot/scan', 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient() # put your IP/hostname here, like 'host=meu_PC'
            self.sim = self.client.getObject('sim')
            self.sim.startSimulation()
            self.robotHandle = self.sim.getObject('/myRobot')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Define constants used to populate Laserscan message data 
        self.timer_period_sec = 0.1

        self.laserscan_msg = LaserScan()
        self.laserscan_msg.header.frame_id = "laser_link"
        self.laserscan_msg.angle_increment = 180*(math.pi/180)/452
        self.laserscan_msg.angle_min = -90 * (math.pi/180)                                         # angle correspond to FIRST beam in scan (in rad)
        self.laserscan_msg.angle_max = (90* (math.pi/180)) - self.laserscan_msg.angle_increment    # angle correspond to LAST beam in scan (in rad)
        self.laserscan_msg.time_increment = (1/50)/452                                             # sensor scans every 50ms with 512 beams. Each beam is measured in (50 ms/512)
        self.laserscan_msg.scan_time = self.timer_period_sec
        self.laserscan_msg.range_min = 0.055
        self.laserscan_msg.range_max = 10.0                                                        # scan can measure up to this range

        # Timer to publish laserscan data 10 Hz
        self.timer = self.create_timer(self.timer_period_sec, self.publish_laserscan)
        
        self.get_logger().info('ROS 2 → CoppeliaSim Laserscan bridge started.')

    def publish_laserscan(self):
        distance = []
        distance = self.sim.getFloatArrayProperty(self.robotHandle, "signal.distSignal", {'noError' : True})
        self.laserscan_msg.header.stamp = self.get_clock().now().to_msg()
        self.laserscan_msg.ranges = distance

        # Publish the message
        self.laserscan_publisher.publish(self.laserscan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherLaserscan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
