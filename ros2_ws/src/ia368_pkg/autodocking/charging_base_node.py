import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class ChargingBaseBridge(Node):
    def __init__(self):
        super().__init__('charging_base_bridge')
        
        # Create publishers for charging base state
        self.strengthSignal_publisher = self.create_publisher(Float32,'/myRobot/charging_base/strengthSignal', 10)
        self.relativeAngle_publisher = self.create_publisher(Float32,'/myRobot/charging_base/relativeAngle', 10)
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Timer to publish charging base data every 10Hz
        self.timer = self.create_timer(0.1, self.publish_charging_base_sensor)
        
        self.get_logger().info('ROS2 → CoppeliaSim Charging base bridge started.')
    
    def publish_charging_base_sensor(self):
        try:
            self.sim.setInt32Signal("Beacon",self.robotHandle) 
            strengthSignal = self.sim.getFloatSignal(str(self.robotHandle) + "StrengthSignal")
            # Publish strength signal if available
            if strengthSignal is not None:
                self.sim.clearFloatSignal(str(self.robotHandle) + "StrengthSignal")
                strengthSignal_msg = Float32()
                strengthSignal_msg.data = strengthSignal
                self.strengthSignal_publisher.publish(strengthSignal_msg)
            else:
                pass#self.get_logger().warn('Strength signal not found')
              
            relativeAngle = self.sim.getFloatSignal(str(self.robotHandle) + "RelativeAngle")
            # Publish relative angle if available 
            if relativeAngle is not None:
                self.sim.clearFloatSignal(str(self.robotHandle) + "RelativeAngle")
                relativeAngle_msg = Float32()
                relativeAngle_msg.data = relativeAngle
                self.relativeAngle_publisher.publish(relativeAngle_msg)
            else:
                pass #self.get_logger().warn('Relative angle signal not found')
                
        except Exception as e:
            self.get_logger().error(f'Error reading charging base: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ChargingBaseBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
