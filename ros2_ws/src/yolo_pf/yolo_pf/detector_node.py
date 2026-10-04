"""YOLO detector: Kinect RGB + depth -> 3D position of bananas and poops.

Adapted from the course yolo_3d_detection node. Differences:
  * uses our YOLO model trained on the simulation (classes banana / poop);
  * depth is converted with the real Kinect clipping planes (near 0.01, far 3.5);
  * the 3D point uses the calibrated pinhole model of the vision sensor;
  * depth is taken as a low percentile inside the box (robust to floor pixels).

Publishes one Marker per detection on /yolo/object_3d_point, in the
camera_color_optical_frame published by the course tf_node (text = class name).
"""
import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from visualization_msgs.msg import Marker


def image_to_numpy(msg: Image):
    if msg.encoding in ('8UC1', 'mono8', '8U'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    channels = msg.step // msg.width
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding.lower() in ('rgb8', 'rgb'):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class DetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_pf_detector')
        default_model = os.path.join(get_package_share_directory('yolo_pf'), 'models', 'banana_poop.pt')
        self.declare_parameter('model', default_model)
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('near', 0.01)          # Kinect near clipping plane (m)
        self.declare_parameter('far', 3.5)            # Kinect far clipping plane (m)
        self.declare_parameter('fov', 0.9948376)      # perspective angle (rad), larger image side
        # position of the rgb sensor inside camera_ref (the frame published by tf_node)
        self.declare_parameter('sensor_offset', [0.014, 0.0, 0.029])
        # far detections have coarse depth (8-bit image) and tiny boxes: ignore them
        self.declare_parameter('max_range', 2.5)

        self.model = YOLO(self.get_parameter('model').value)
        self.conf = self.get_parameter('confidence').value
        self.near = self.get_parameter('near').value
        self.far = self.get_parameter('far').value
        self.fov = self.get_parameter('fov').value
        self.offset = np.array(self.get_parameter('sensor_offset').value)
        self.max_range = self.get_parameter('max_range').value
        self.get_logger().info(f'model classes: {self.model.names}')

        self.depth = None
        self.create_subscription(Image, '/depth/image', self.depth_callback, 10)
        self.create_subscription(Image, '/rgb/image', self.rgb_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/yolo/object_3d_point', 50)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)

    def depth_callback(self, msg):
        self.depth = image_to_numpy(msg)

    def depth_at(self, x1, y1, x2, y2):
        """Distance (m) of the object inside the box: low percentile of the central region."""
        w, h = x2 - x1, y2 - y1
        a, b = int(x1 + 0.25 * w), int(x2 - 0.25 * w) + 1
        c, d = int(y1 + 0.2 * h), int(y2 - 0.2 * h) + 1
        patch = self.depth[c:d, a:b].astype(np.float32)
        patch = patch[(patch > 0) & (patch < 254)]     # 255 = beyond far plane
        if patch.size == 0:
            return None
        return self.near + np.percentile(patch, 30) / 255.0 * (self.far - self.near)

    def rgb_callback(self, msg):
        if self.depth is None:
            return
        img = image_to_numpy(msg)
        H, W = img.shape[:2]
        f = (max(W, H) / 2) / math.tan(self.fov / 2)
        res = self.model(img, verbose=False, conf=self.conf)[0]

        for box, cls, conf in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                                  res.boxes.conf.cpu().numpy()):
            x1, y1, x2, y2 = box
            if x1 <= 2 or x2 >= W - 3:
                continue                      # cut by the image border: its centre is wrong
            z = self.depth_at(x1, y1, x2, y2)
            if z is None or z > self.max_range:
                continue
            u, v = (x1 + x2) / 2, (y1 + y2) / 2
            # calibrated pinhole model of the CoppeliaSim vision sensor: u = W/2 - f x/z, v = H/2 - f y/z
            p = np.array([(W / 2 - u) * z / f, (H / 2 - v) * z / f, z]) + self.offset

            m = Marker()
            m.header.frame_id = 'camera_color_optical_frame'
            m.header.stamp = msg.header.stamp
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.lifetime.nanosec = 500_000_000   # live detections fade out in RViz
            name = self.model.names[int(cls)]
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 0.0, 1.0) if name == 'banana' else (0.5, 0.25, 0.0, 1.0)
            m.text = name
            m.ns = f'{conf:.2f}'
            self.marker_pub.publish(m)

        annotated = res.plot()
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = annotated.tobytes()
        self.image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
