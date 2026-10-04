import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class KinectNode(Node):
    info_msg = None

    def __init__(self):
        super().__init__('kinect_node')
        
        # Create publishers for kinect data
        self.rgb_publisher = self.create_publisher(Image, '/rgb/image', 10)
        self.depth_publisher = self.create_publisher(Image, '/depth/image', 10)
        self.camerainfo_publisher = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            self.depthCam=self.sim.getObject('/myRobot/kinect/depth')
            self.colorCam=self.sim.getObject('/myRobot/kinect/rgb')

            self.sim.startSimulation()

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Prepare info_msg with default settings
        self.populate_info_msg()
        
        # Timer to publish kinect data every 10Hz
        self.timer_rgb = self.create_timer(0.1, self.publish_camera_rgb)
        self.timer_depth = self.create_timer(0.1, self.publish_camera_depth)
        self.timer_info = self.create_timer(0.1, self.publish_camera_info)
        
        self.get_logger().info('ROS2 → CoppeliaSim Kinect node started.')
    
    def populate_info_msg(self):
        self.info_msg = CameraInfo()
        self.info_msg.roi = RegionOfInterest()

        # Fill message
        self.info_msg.header.frame_id = "kinect"

        resolution = self.sim.getVisionSensorResolution(self.depthCam)
        if resolution is not None:
            self.info_msg.height = resolution[1]
            self.info_msg.width = resolution[0]
        else:
            self.get_logger().warn('Kinect camera info not available')

        self.info_msg.distortion_model = 'plumb_bob'
        self.info_msg.d = [0.262383,-0.953104,-0.005358,0.002628,1.163314]
        self.info_msg.k = [517.306408, 0.0, 318.643040, 0.0, 516.469215, 255.313989, 0.0, 0.0, 1.0]
        self.info_msg.p = [517.306408, 0.0, 318.643040, 0.0, 0.0, 516.469215, 255.313989, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_msg.binning_x = 0
        self.info_msg.binning_y = 0

        self.info_msg.roi.x_offset = 0
        self.info_msg.roi.y_offset = 0
        self.info_msg.roi.height = 0
        self.info_msg.roi.width = 0
        self.info_msg.roi.do_rectify = False     

    def publish_camera_rgb(self):
        #try:
        # Get vision sensor RGB image from CoppeliaSim
        data, resolution = self.sim.getVisionSensorImg(self.colorCam)
        data = self.sim.transformImage(data,resolution,4)
        
        if data is not None:
            # Create image message
            rgb_msg = Image()

            # Fill image data
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "kinect"
            rgb_msg.height = resolution[1]
            rgb_msg.width = resolution[0]
            rgb_msg.encoding = 'rgb8'
            rgb_msg.is_bigendian = 1
            rgb_msg.step = resolution[0]*3
            rgb_msg.data = data
            
            # Publish the message
            self.rgb_publisher.publish(rgb_msg)
        else:
            self.get_logger().warn('Kinect RGB image not found')
    
    def publish_camera_depth(self):
        # Get vision sensor RGB depth from CoppeliaSim
        data = self.sim.getVisionSensorDepthBuffer(self.depthCam+self.sim.handleflag_codedstring)
        
        resolution, nearClippingPlane = self.sim.getObjectFloatParameter(self.depthCam,self.sim.visionfloatparam_near_clipping)
        resolution, farClippingPlane = self.sim.getObjectFloatParameter(self.depthCam,self.sim.visionfloatparam_far_clipping)
        nearClippingPlane = nearClippingPlane*1000 # we want mm
        farClippingPlane = farClippingPlane*1000 # we want mm
        data = self.sim.transformBuffer(data,self.sim.buffer_float,farClippingPlane-nearClippingPlane,nearClippingPlane,self.sim.buffer_uint16)
        
        if data is not None:
            # Create image message
            depth_msg = Image()

            # Fill image data
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "kinect"
            depth_msg.height = self.info_msg.height
            depth_msg.width = self.info_msg.width
            depth_msg.encoding = '16UC1'
            depth_msg.is_bigendian = 0
            depth_msg.step = depth_msg.width * 2
            depth_msg.data = data

            # Publish the message
            self.depth_publisher.publish(depth_msg)
        else:
            self.get_logger().warn('Kinect depth image not found')

    def publish_camera_info(self):
        # Fill message
        self.info_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.camerainfo_publisher.publish(self.info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()