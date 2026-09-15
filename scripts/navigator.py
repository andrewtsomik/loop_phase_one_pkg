#!/usr/bin/env python3
"""
navigator.py - autonomous waypoint navigation with lidar obstacle avoidance.
 
Subscribes:  /odom  (nav_msgs/Odometry)      -> where the vehicle is
             /scan  (sensor_msgs/LaserScan)  -> what's around it
Publishes:   /cmd_vel (geometry_msgs/Twist)  -> how it should move
 
Strategy: move toward the goal, turn if blocked.
  GO_TO_GOAL : steer toward the current waypoint
  AVOID      : obstacle ahead -> turn in place toward the more open side
  CLEAR      : path ahead is clear again -> drive straight briefly before
               re-aiming at the goal (stops left/right oscillation at walls)
  RECOVER    : no progress for a while -> back up and turn
  DONE       : all waypoints handled -> stop
"""
 
import math
import random
 
import rclpy
from rclpy.node import Node
 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
 
 
# Known obstacles as (x, y, radius) in meters. EDIT THESE to match course.sdf.
# Only used to keep random waypoints from spawning inside obstacles --
# the actual avoidance while driving uses the lidar.
OBSTACLES = [
    (3.0, 0.0, 1.5),     # boulder_1
    (-2.0, 3.0, 1.5),    # boulder_2
    (0.0, -4.0, 1.5),    # boulder_3
    (-5.0, -2.0, 1.2),   # inner_wall (x=-5, y=-3..3) approximated by 3 circles
    (-5.0, 0.0, 1.2),
    (-5.0, 2.0, 1.2),
]
 
 
def wrap_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))
 
 
def clamp(v, limit):
    return max(-limit, min(limit, v))
 
 
class Navigator(Node):
    GO = 'GO_TO_GOAL'
    AVOID = 'AVOID'
    CLEAR = 'CLEAR'
    RECOVER = 'RECOVER'
    DONE = 'DONE'
 
    def __init__(self):
        super().__init__('navigator')
 
        # ---- Parameters (override with --ros-args -p name:=value) ----
        params = {
            'seed': 0,                  # 0 = new random waypoints every run
            'num_goals': 4,
            'area': 8.0,                # waypoints within [-area, area] in x and y
            'min_goal_spacing': 3.0,    # meters between waypoints
            'goal_tolerance': 0.3,      # "arrived" when closer than this
            'obstacle_distance': 0.8,   # start avoiding when something is this close ahead
            'max_speed': 0.5,           # m/s
            'max_turn': 1.0,            # rad/s
            'goal_timeout': 90.0,       # s before giving up on a waypoint
            'stuck_window': 8.0,        # s between progress checks
            'stuck_distance': 0.15,     # m moved per window, less = stuck
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
 
        # ---- Waypoints ----
        seed = self.seed or random.randrange(1, 1_000_000)
        random.seed(seed)
        self.goals = self.make_goals(self.num_goals)
        self.get_logger().info(f'Seed {seed} -> waypoints: '
                               + ', '.join(f'({x:.1f}, {y:.1f})' for x, y in self.goals))
        self.goal_idx = 0
        self.results = []
 
        # ---- Sensor state ----
        self.pose = None                  # (x, y, yaw)
        self.have_scan = False
        self.front = self.left = self.right = float('inf')
 
        # ---- Control state ----
        now = self.now()
        self.state = self.GO
        self.state_start = now
        self.turn_dir = 1.0               # +1 = left, -1 = right
        self.goal_start = now
        self.check_time = now
        self.check_pos = None
 
        # ---- ROS interfaces ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_timer(0.1, self.loop)  # 10 Hz control loop
 
    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def now(self):
        """Current time in seconds (follows sim time if use_sim_time is set)."""
        return self.get_clock().now().nanoseconds / 1e9
 
    def is_free(self, p, margin=0.7):
        """True if point p is not inside/near a known obstacle or the start."""
        if math.hypot(*p) < 1.5:          # too close to the start point
            return False
        return all(math.dist(p, (ox, oy)) > r + margin for ox, oy, r in OBSTACLES)
 
    def make_goals(self, n):
        goals = []
        for _ in range(10_000):
            if len(goals) == n:
                break
            p = (random.uniform(-self.area, self.area),
                 random.uniform(-self.area, self.area))
            if self.is_free(p) and all(math.dist(p, g) >= self.min_goal_spacing for g in goals):
                goals.append(p)
        if len(goals) < n:
            self.get_logger().warn(f'Only generated {len(goals)} of {n} waypoints')
        return goals
 
    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
            self.state_start = self.now()
 
    def next_goal(self, reached):
        goal = self.goals[self.goal_idx]
        self.results.append((goal, reached))
        if reached:
            self.get_logger().info(f'Reached waypoint {self.goal_idx + 1}: ({goal[0]:.1f}, {goal[1]:.1f})')
        else:
            self.get_logger().warn(f'SKIPPING waypoint {self.goal_idx + 1}: ({goal[0]:.1f}, {goal[1]:.1f})')
 
        self.goal_idx += 1
        now = self.now()
        self.goal_start = now
        self.check_time = now
        self.check_pos = None
 
        if self.goal_idx >= len(self.goals):
            hits = sum(r for _, r in self.results)
            self.get_logger().info(f'Finished: reached {hits}/{len(self.results)} waypoints')
            self.set_state(self.DONE)
        else:
            self.set_state(self.GO)
 
    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
 
    def on_scan(self, msg):
        """Reduce the scan to the closest reading in three zones (angle 0 = straight ahead)."""
        def closest(a_min, a_max):
            best = float('inf')
            for i, r in enumerate(msg.ranges):
                a = msg.angle_min + i * msg.angle_increment
                if a_min <= a <= a_max and math.isfinite(r) and r >= msg.range_min:
                    best = min(best, r)
            return best
 
        self.front = closest(-0.5, 0.5)     # about +-30 deg
        self.left = closest(0.5, 1.5)       # about 30..85 deg
        self.right = closest(-1.5, -0.5)    # about -85..-30 deg
        self.have_scan = True
 
    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def loop(self):
        cmd = Twist()
 
        if self.state == self.DONE:
            self.cmd_pub.publish(cmd)       # stay stopped
            return
        if self.pose is None or not self.have_scan:
            return                          # wait for sensors
 
        now = self.now()
        x, y, yaw = self.pose
        gx, gy = self.goals[self.goal_idx]
        dist = math.hypot(gx - x, gy - y)
 
        # Status line every 2 s (for debugging and for the recording)
        self.get_logger().info(
            f'[{self.state}] pos=({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}deg '
            f'goal={self.goal_idx + 1} dist={dist:.2f} '
            f'front={self.front:.2f} left={self.left:.2f} right={self.right:.2f}',
            throttle_duration_sec=2.0)
 
        # 1. Arrived?
        if dist < self.goal_tolerance:
            self.next_goal(reached=True)
            self.cmd_pub.publish(cmd)
            return
 
        # 2. Taking too long? (unreachable waypoint)
        if now - self.goal_start > self.goal_timeout:
            self.next_goal(reached=False)
            self.cmd_pub.publish(cmd)
            return
 
        # 3. Stuck? (not enough movement in the last window)
        if self.check_pos is None:
            self.check_pos = (x, y)
        elif now - self.check_time > self.stuck_window:
            moved = math.dist((x, y), self.check_pos)
            self.check_time, self.check_pos = now, (x, y)
            if moved < self.stuck_distance and self.state != self.RECOVER:
                self.get_logger().warn(f'Stuck (moved {moved:.2f} m) - recovering')
                self.set_state(self.RECOVER)
 
        # 4. Decide how to move
        if self.state == self.RECOVER:
            if now - self.state_start < 1.5:
                cmd.linear.x = -0.2                     # back up
                cmd.angular.z = self.turn_dir * 0.5     # while turning
            else:
                self.set_state(self.AVOID)
 
        elif self.front < self.obstacle_distance:
            if self.state != self.AVOID:
                # Pick a side once and stick with it, so it doesn't flip-flop
                self.turn_dir = 1.0 if self.left > self.right else -1.0
                self.set_state(self.AVOID)
            cmd.angular.z = self.turn_dir * self.max_turn
 
        elif self.state == self.AVOID:
            self.set_state(self.CLEAR)
            cmd.linear.x = 0.6 * self.max_speed
 
        elif self.state == self.CLEAR:
            if now - self.state_start < 1.5:
                cmd.linear.x = 0.6 * self.max_speed     # get past the obstacle edge
            else:
                self.set_state(self.GO)
 
        else:  # GO_TO_GOAL
            err = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
            cmd.angular.z = clamp(1.5 * err, self.max_turn)
            # Full speed when facing the goal, none when facing away
            speed = self.max_speed * max(0.0, math.cos(err))
            speed = min(speed, 0.5 * dist + 0.1)        # slow down near the goal
            if self.front < 2 * self.obstacle_distance:
                speed *= 0.5                            # slow down near obstacles
            cmd.linear.x = speed
 
        self.cmd_pub.publish(cmd)
 
 
def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # stop the vehicle on exit
        node.destroy_node()
        rclpy.try_shutdown()
 
 
if __name__ == '__main__':
    main()
