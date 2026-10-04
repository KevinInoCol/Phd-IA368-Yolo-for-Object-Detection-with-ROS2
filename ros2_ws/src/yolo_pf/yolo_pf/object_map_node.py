"""Persistent object map of bananas and obstacles (poops).

Every YOLO detection (camera frame) is transformed into the map frame. If it falls
within MERGE_RADIUS of a known object of the same class, it refines that object
(running mean) instead of being inserted again. An object is only published once
it has been seen CONFIRM_HITS times, which filters out spurious detections.
Objects stay in the map when they are no longer visible.

Subscribes : /yolo/object_3d_point (Marker), /yolo_pf/collected (PointStamped)
Publishes  : /yolo_pf/bananas (PoseArray, uncollected), /yolo_pf/obstacles (PoseArray),
             /yolo_pf/map_markers (MarkerArray, for RViz)
"""
import math

import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

# m, same-class detections closer than this are the same object. Poops use a larger
# radius (merging two close poops still leaves an obstacle there); bananas a smaller
# one because two real bananas can lie close to each other.
MERGE_RADIUS = {'banana': 0.30, 'poop': 0.45}
# a banana and a poop this close are the same object seen with two labels
# (in the scene bananas are always >= 0.5 m from any poop): the label seen more often wins
CROSS_RADIUS = 0.25
CONFIRM_HITS = 3       # detections needed before an object is trusted
MAX_WEIGHT = 30        # cap of the running mean so late, closer views still refine it
MAX_AGE = 1.0          # s, older detections are dropped (the robot has moved since)


class TrackedObject:
    def __init__(self, x, y):
        self.x, self.y, self.hits, self.collected = x, y, 1, False

    def update(self, x, y):
        w = min(self.hits, MAX_WEIGHT)
        self.x = (self.x * w + x) / (w + 1)
        self.y = (self.y * w + y) / (w + 1)
        self.hits += 1

    def dist(self, x, y):
        return math.hypot(self.x - x, self.y - y)


class ObjectMapNode(Node):
    def __init__(self):
        super().__init__('object_map')
        self.objects = {'banana': [], 'poop': []}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Marker, '/yolo/object_3d_point', self.detection_callback, 50)
        self.create_subscription(PointStamped, '/yolo_pf/collected', self.collected_callback, 10)
        self.banana_pub = self.create_publisher(PoseArray, '/yolo_pf/bananas', 10)
        self.obstacle_pub = self.create_publisher(PoseArray, '/yolo_pf/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo_pf/map_markers', 10)
        self.create_timer(0.2, self.publish_map)

    def to_map(self, msg: Marker):
        """Camera point -> map frame, never blocking (a blocked callback builds a stale backlog)."""
        p = PointStamped()
        p.header.frame_id = msg.header.frame_id
        p.header.stamp = msg.header.stamp
        p.point = msg.pose.position
        if not self.tf_buffer.can_transform('map', p.header.frame_id, Time.from_msg(p.header.stamp)):
            p.header.stamp = Time().to_msg()   # stamped TF not available yet: use the latest one
        try:
            return self.tf_buffer.transform(p, 'map')
        except Exception as e:
            self.get_logger().warn(f'TF to map failed: {e}', throttle_duration_sec=2.0)
            return None

    def detection_callback(self, msg: Marker):
        cls = msg.text
        if cls not in self.objects:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > MAX_AGE:
            return
        p = self.to_map(msg)
        if p is None:
            return
        x, y = p.point.x, p.point.y
        known = self.objects[cls]
        nearest = min(known, key=lambda o: o.dist(x, y), default=None)
        if nearest is not None and nearest.dist(x, y) < MERGE_RADIUS[cls]:
            if not nearest.collected:
                nearest.update(x, y)
                if nearest.hits == CONFIRM_HITS:
                    self.get_logger().info(f'new {cls} at ({nearest.x:.2f}, {nearest.y:.2f})')
        else:
            known.append(TrackedObject(x, y))

    def collected_callback(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        cand = [o for o in self.objects['banana'] if not o.collected]
        nearest = min(cand, key=lambda o: o.dist(x, y), default=None)
        if nearest is not None and nearest.dist(x, y) < MERGE_RADIUS['banana']:
            nearest.collected = True
            left = sum(1 for o in self.objects['banana'] if o.hits >= CONFIRM_HITS and not o.collected)
            done = sum(1 for o in self.objects['banana'] if o.collected)
            self.get_logger().info(f'banana collected at ({nearest.x:.2f}, {nearest.y:.2f}); '
                                   f'collected={done}, known pending={left}')

    def consolidate(self):
        """Merge entries of the same class that converged within the merge radius."""
        for cls, objs in self.objects.items():
            i = 0
            while i < len(objs):
                a = objs[i]
                j = i + 1
                while j < len(objs):
                    b = objs[j]
                    if a.dist(b.x, b.y) < MERGE_RADIUS[cls] and a.collected == b.collected:
                        w = a.hits + b.hits
                        a.x, a.y = (a.x * a.hits + b.x * b.hits) / w, (a.y * a.hits + b.y * b.hits) / w
                        a.hits = w
                        objs.pop(j)
                    else:
                        j += 1
                i += 1
        # cross-class conflicts: drop the label with fewer detections
        for b in list(self.objects['banana']):
            for p in list(self.objects['poop']):
                if b.dist(p.x, p.y) < CROSS_RADIUS:
                    if b.hits >= p.hits:
                        self.objects['poop'].remove(p)
                    else:
                        self.objects['banana'].remove(b)
                        break

    def confirmed(self, cls):
        return [o for o in self.objects[cls] if o.hits >= CONFIRM_HITS]

    def publish_map(self):
        self.consolidate()
        now = self.get_clock().now().to_msg()
        arrays = {}
        for cls, pub in (('banana', self.banana_pub), ('poop', self.obstacle_pub)):
            pa = PoseArray()
            pa.header.frame_id, pa.header.stamp = 'map', now
            for o in self.confirmed(cls):
                if cls == 'banana' and o.collected:
                    continue
                pose = Pose()
                pose.position.x, pose.position.y = o.x, o.y
                pose.orientation.w = 1.0
                pa.poses.append(pose)
            pub.publish(pa)
            arrays[cls] = pa

        ma = MarkerArray()
        for i, (cls, color) in enumerate((('banana', (1.0, 0.9, 0.0)), ('poop', (0.45, 0.25, 0.05)))):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', now
            m.ns, m.id, m.type, m.action = cls, i, Marker.SPHERE_LIST, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            m.pose.orientation.w = 1.0
            m.points = [Point(x=p.position.x, y=p.position.y, z=0.05) for p in arrays[cls].poses]
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
