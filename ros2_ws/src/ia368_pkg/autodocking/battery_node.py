import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class BatteryBridge(Node):
    def __init__(self):
        super().__init__('battery_bridge')
        
        # Create publisher for battery state
        self.battery_publisher = self.create_publisher(BatteryState,'/myRobot/battery_state', 10)
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            
            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Timer to publish battery data every 10Hz
        self.timer = self.create_timer(0.1, self.publish_battery_state)
        
        self.get_logger().info('ROS2 → CoppeliaSim Battery bridge started.')
    
    def publish_battery_state(self):
        try:
            # Read battery level and charging status from CoppeliaSim
            battery_level = self.sim.getFloatSignal(str(self.robotHandle) + "Battery")
            charging_status = self.sim.getInt32Signal(str(self.robotHandle) + "Charging")
            
            if battery_level is not None:
                # Create battery state message
                battery_msg = BatteryState()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = "base_link"
                
                # Fill battery data
                battery_msg.percentage = max(0.0, min(100.0, battery_level))  # 0-100%
                battery_msg.capacity = 100.0 
                
                # Set power supply status based on charging signal
                if charging_status == 1:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                else:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                
                # Publish the message
                self.battery_publisher.publish(battery_msg)
            else:
                self.get_logger().warn('Battery signal not found')
                
        except Exception as e:
            self.get_logger().error(f'Error reading battery/charging: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()