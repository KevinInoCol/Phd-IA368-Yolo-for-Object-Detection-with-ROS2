"""Potential field controller fed by the persistent object map.

Attractive potential  : U_att = 1/2 k_att d^2 (conic beyond D_ATT_MAX) toward the selected banana
Repulsive potential   : U_rep = 1/2 k_rep (1/rho - 1/rho0)^2 for every detected obstacle
Resultant force       : F = -grad U_att - sum grad U_rep  ->  heading  ->  (v, w)  ->  cmd_vel

Search behaviour when no uncollected banana is known: turn in place to look around;
if nothing shows up after a full turn, the attractive goal becomes a random exploration
point (still moving under the same field, so poops are avoided); once it is reached
(or the robot stops making progress) the robot turns again.

Subscribes : /yolo_pf/bananas, /yolo_pf/obstacles (PoseArray), TF map -> base_link
Publishes  : /myRobot/cmd_vel (Twist), /yolo_pf/collected (PointStamped)
"""
import math
import random

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseArray, Twist
from rclpy.node import Node
from rclpy.time import Time

# robot / control
V_MAX, W_MAX, K_HEADING = 0.30, 2.0, 2.5
# attractive field
K_ATT, D_ATT_MAX = 1.0, 1.0
# repulsive field (same tuning as the signal-based exercise)
K_REP, RHO0, K_VORTEX, D_GOAL_FADE = 0.40, 0.80, 0.6, 0.50
# goal handling
REACHED_DIST = 0.15
ARENA = 2.3            # exploration points are drawn inside [-ARENA, ARENA]^2
SEARCH_TURN = 2 * math.pi
STUCK_TIME = 20.0      # s without getting 0.1 m closer to the target -> give it up for a while
BLACKLIST_TIME = 60.0  # s an abandoned target is ignored
EXPLORE_MIN_DIST = 1.5 # exploration points are at least this far from the robot


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field')
        self.bananas, self.obstacles = [], []
        self.target = None                 # (x, y) of the selected banana
        self.mode = 'search_turn'
        self.turned, self.last_th = 0.0, None
        self.explore_goal = None
        self.collected = 0
        self.best_dist, self.progress_t = math.inf, None
        self.blacklist = []                # [(x, y, until_time_s)]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseArray, '/yolo_pf/bananas', self.bananas_cb, 10)
        self.create_subscription(PoseArray, '/yolo_pf/obstacles', self.obstacles_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/myRobot/cmd_vel', 10)
        self.collected_pub = self.create_publisher(PointStamped, '/yolo_pf/collected', 10)
        self.create_timer(0.05, self.step)

    def bananas_cb(self, msg):
        self.bananas = [(p.position.x, p.position.y) for p in msg.poses]

    def obstacles_cb(self, msg):
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # the robot front is its local +Y axis
        return t.transform.translation.x, t.transform.translation.y, wrap(yaw + math.pi / 2)

    # ---------------------------------------------------------------- forces
    def attractive(self, x, y, gx, gy):
        dx, dy = gx - x, gy - y
        d = math.hypot(dx, dy)
        if d <= D_ATT_MAX:
            return K_ATT * dx, K_ATT * dy
        return K_ATT * D_ATT_MAX * dx / d, K_ATT * D_ATT_MAX * dy / d

    def repulsive(self, x, y, ax, ay, d_goal):
        fx = fy = 0.0
        fade = min(1.0, d_goal / D_GOAL_FADE)  # keeps a banana next to a poop reachable
        for ox, oy in self.obstacles:
            dx, dy = x - ox, y - oy
            rho = max(math.hypot(dx, dy), 0.05)
            if rho >= RHO0:
                continue
            mag = fade * K_REP * (1 / rho - 1 / RHO0) / rho ** 2
            ux, uy = dx / rho, dy / rho
            side = -1 if (ax * uy - ay * ux) >= 0 else 1   # slide around on the goal side
            fx += mag * (ux - K_VORTEX * side * uy)
            fy += mag * (uy + K_VORTEX * side * ux)
        return fx, fy

    def drive_to(self, x, y, th, gx, gy):
        fax, fay = self.attractive(x, y, gx, gy)
        frx, fry = self.repulsive(x, y, fax, fay, math.hypot(gx - x, gy - y))
        fx, fy = fax + frx, fay + fry
        err = wrap(math.atan2(fy, fx) - th)
        v = min(V_MAX, math.hypot(fx, fy)) * max(0.0, math.cos(err))
        w = max(-W_MAX, min(W_MAX, K_HEADING * err))
        return v, w

    # ---------------------------------------------------------------- main loop
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def available(self):
        t = self.now_s()
        self.blacklist = [b for b in self.blacklist if b[2] > t]
        return [b for b in self.bananas
                if all(math.hypot(b[0] - k[0], b[1] - k[1]) > 0.3 for k in self.blacklist)]

    def check_progress(self, d):
        """Local-minimum guard: abandon a target we stop getting closer to."""
        t = self.now_s()
        if d < self.best_dist - 0.1 or self.progress_t is None:
            self.best_dist, self.progress_t = d, t
        elif t - self.progress_t > STUCK_TIME:
            self.get_logger().warn(f'no progress toward ({self.target[0]:.2f}, {self.target[1]:.2f}): '
                                   f'ignoring it for {BLACKLIST_TIME:.0f} s')
            self.blacklist.append((self.target[0], self.target[1], t + BLACKLIST_TIME))
            self.target, self.best_dist, self.progress_t = None, math.inf, None

    def select_target(self, x, y):
        bananas = self.available()
        if not bananas:
            return None
        if self.target is not None:
            # keep the current target (its estimate may move slightly between updates)
            near = min(bananas, key=lambda b: math.hypot(b[0] - self.target[0], b[1] - self.target[1]))
            if math.hypot(near[0] - self.target[0], near[1] - self.target[1]) < 0.3:
                return near
        self.best_dist, self.progress_t = math.inf, None   # new target: restart progress watch
        return min(bananas, key=lambda b: math.hypot(b[0] - x, b[1] - y))

    def step(self):
        pose = self.pose()
        if pose is None:
            return
        x, y, th = pose
        cmd = Twist()

        self.target = self.select_target(x, y)
        if self.target is not None:
            if self.mode != 'go':
                self.get_logger().info(f'target banana at ({self.target[0]:.2f}, {self.target[1]:.2f})')
            self.mode = 'go'
            gx, gy = self.target
            self.check_progress(math.hypot(gx - x, gy - y))
            if self.target is None:
                pass                      # target abandoned: a new one is chosen next step
            elif math.hypot(gx - x, gy - y) < REACHED_DIST:
                p = PointStamped()
                p.header.frame_id, p.header.stamp = 'map', self.get_clock().now().to_msg()
                p.point.x, p.point.y = gx, gy
                self.collected_pub.publish(p)
                self.bananas = [b for b in self.bananas if b != self.target]
                self.collected += 1
                self.get_logger().info(f'banana reached ({self.collected} collected)')
                self.target = None
            else:
                cmd.linear.x, cmd.angular.z = self.drive_to(x, y, th, gx, gy)
        else:
            if self.mode == 'go':
                self.get_logger().info('no known banana: searching')
                self.mode, self.turned, self.last_th = 'search_turn', 0.0, None
            if self.mode == 'search_turn':
                if self.last_th is not None:
                    self.turned += abs(wrap(th - self.last_th))
                self.last_th = th
                cmd.angular.z = 1.0
                if self.turned >= SEARCH_TURN:
                    self.mode = 'explore'
                    while True:
                        g = (random.uniform(-ARENA, ARENA), random.uniform(-ARENA, ARENA))
                        if math.hypot(g[0] - x, g[1] - y) > EXPLORE_MIN_DIST:
                            break
                    self.explore_goal, self.best_dist, self.progress_t = g, math.inf, None
                    self.get_logger().info(f'exploring toward ({g[0]:.2f}, {g[1]:.2f})')
            elif self.mode == 'explore':
                gx, gy = self.explore_goal
                d = math.hypot(gx - x, gy - y)
                t = self.now_s()
                if d < self.best_dist - 0.1 or self.progress_t is None:
                    self.best_dist, self.progress_t = d, t
                if d < 0.3 or t - self.progress_t > STUCK_TIME:
                    self.mode, self.turned, self.last_th = 'search_turn', 0.0, None
                else:
                    cmd.linear.x, cmd.angular.z = self.drive_to(x, y, th, gx, gy)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'mode={self.mode} pose=({x:.2f},{y:.2f},{math.degrees(th):.0f}deg) '
            f'target={self.target} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} '
            f'bananas={len(self.bananas)} obstacles={len(self.obstacles)}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
