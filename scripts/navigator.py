#!/usr/bin/env python3
"""Simple navigator: drive toward each waypoint, turn away if something is in front."""
 
import math
import random
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
 
# Obstacles in course.sdf as (x, y, radius). Used only to keep waypoints out of them.
OBSTACLES = [(3, 0, 1.5), (-2, 3, 1.5), (0, -4, 1.5),
             (-5, -2, 1.2), (-5, 0, 1.2), (-5, 2, 1.2)]
 
GOAL_TOLERANCE = 0.3   # m: counts as "arrived" when this close
SAFE_DISTANCE = 0.8    # m: turn if something in front is closer than this
TIMEOUT = 90.0         # s: give up on a waypoint after this long
 
 
def random_waypoints(n):
    points = []
    while len(points) < n:
        p = (random.uniform(-8, 8), random.uniform(-8, 8))
        clear = all(math.dist(p, (ox, oy)) > r + 0.7 for ox, oy, r in OBSTACLES)
        if clear and math.hypot(*p) > 1.5:          # not inside an obstacle or at the start
            points.append(p)
    return points
 
 
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        seed = self.declare_parameter('seed', 0).value
        random.seed(seed or None)                   # 0 = different waypoints every run
        self.goals = random_waypoints(4)
        self.get_logger().info(f'Waypoints: {[(round(x, 1), round(y, 1)) for x, y in self.goals]}')
 
        self.x = self.y = self.yaw = None
        self.front = self.left = self.right = float('inf')
        self.goal_start = self.now()
 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_timer(0.1, self.loop)           # run loop() 10 times per second
 
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9
 
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
 
    def on_scan(self, msg):
        def closest(lo, hi):   # nearest reading between two angles (0 = straight ahead)
            return min((r for i, r in enumerate(msg.ranges)
                        if lo <= msg.angle_min + i * msg.angle_increment <= hi
                        and math.isfinite(r)), default=float('inf'))
        self.front = closest(-0.5, 0.5)
        self.left = closest(0.5, 1.5)
        self.right = closest(-1.5, -0.5)
 
    def loop(self):
        if not self.goals:
            self.cmd_pub.publish(Twist())           # all done: stay stopped
            return
        if self.x is None:
            return                                  # no position yet
 
        gx, gy = self.goals[0]
        dist = math.hypot(gx - self.x, gy - self.y)
 
        # Arrived, or taking too long -> move on to the next waypoint
        if dist < GOAL_TOLERANCE or self.now() - self.goal_start > TIMEOUT:
            result = 'Reached' if dist < GOAL_TOLERANCE else 'Gave up on'
            self.get_logger().info(f'{result} waypoint ({gx:.1f}, {gy:.1f})')
            self.goals.pop(0)
            self.goal_start = self.now()
            if not self.goals:
                self.get_logger().info('All waypoints done')
            return
 
        cmd = Twist()
        if self.front < SAFE_DISTANCE:
            # Blocked: turn in place toward the side with more room
            cmd.angular.z = 1.0 if self.left > self.right else -1.0
        else:
            # Clear: steer toward the goal
            error = math.atan2(gy - self.y, gx - self.x) - self.yaw
            error = math.atan2(math.sin(error), math.cos(error))    # wrap to [-pi, pi]
            cmd.angular.z = max(-1.0, min(1.0, 1.5 * error))
            cmd.linear.x = 0.5 if abs(error) < 0.5 else 0.1
        self.cmd_pub.publish(cmd)
 
 
def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
 
 
if __name__ == '__main__':
    main()
