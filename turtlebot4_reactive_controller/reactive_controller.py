#!/usr/bin/env python3

# Reactive controller using subsumption architecture for the TurtleBot 4.
# Behaviors are ordered from highest to lowest priority:
#     1. Halt if collision detected by "bumper".
#     2. Accept keyboard movement commands from teleop.
#     3. Escape from roughly symmetric obstacles within 1 ft.
#     4. Avoid asymmetric obstacles within 1 ft.
#     5. Turn randomly (uniformly sampled within +/-15 deg) after every 1 ft.
#     6. Drive forward.

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import HazardDetection

# The TurtleBot 4 RPLIDAR's angle 0 points to the robot's left.
# Adding pi/2 rotates it so that 0 in our code means robot-forward.
LIDAR_OFFSET = math.pi / 2

class ReactiveController(Node):
    # Subsumption-style reactive controller for the TurtleBot 4.

    def __init__(self):
        super().__init__('reactive_controller')

        # Sensor state
        self.latest_scan = None
        self.current_yaw = 0.0

        # Keyboard state
        self.last_key_cmd = Twist()
        self.key_time = self.get_clock().now()

        # Odometry bookkeeping for random turn
        self.last_pose = None
        self.dist_since_turn = 0.0

        # Escape fixed-action-pattern state
        self.escape_dir = 0.0
        self.escape_end = None
        self.halted = False  # True for one cycle after collision, before escape begins

        # Avoid reflex state (minimum turn duration)
        self.avoid_dir = 0.0
        self.avoid_end = None

        # QoS profile to match BEST_EFFORT sensor publishers
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.key_sub = self.create_subscription(
            Twist, '/cmd_vel_key', self.key_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos)
        
        # Bumper state
        self.bumper_hit = False

        # Subscribe to Create 3 hazard detection
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, '/hazard_detection',
            self.hazard_callback, sensor_qos)

        # Publisher (Changed to use Stamped)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.behavior_loop)
        self.get_logger().info('ReactiveController started.')

    def scan_callback(self, msg):
        # Store the latest laser scan.
        self.latest_scan = msg

    def key_callback(self, msg):
        # Store the latest keyboard command and its timestamp.
        self.last_key_cmd = msg
        self.key_time = self.get_clock().now()

    def odom_callback(self, msg):
        # Accumulate distance traveled and track current yaw.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.dist_since_turn += math.hypot(dx, dy)
        self.last_pose = (x, y)

    def hazard_callback(self, msg):
        # Check if any hazard in the vector is a bumper hit.
        self.bumper_hit = any(
            d.type == HazardDetection.BUMP for d in msg.detections
        )

    # Heleper to convert Twist to TwistStamped
    def publish_cmd(self, twist_msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = twist_msg
        self.cmd_pub.publish(stamped)

    def behavior_loop(self):
        # Main control loop implementing the subsumption behavior priority.
        if self.latest_scan is None:
            return

        msg = Twist()
        now = self.get_clock().now()
        scan = self.latest_scan

        # -- Build front-left and front-right minimums (+/-45 deg cone) --
        left_min = 10.0
        right_min = 10.0
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min:
                continue
            raw = scan.angle_min + i * scan.angle_increment
            angle = raw + LIDAR_OFFSET  # Apply offset so 0 = robot-forward
            angle = math.atan2(math.sin(angle), math.cos(angle))  # Wrap to [-pi, pi]
            if 0.0 <= angle <= math.radians(45) and r < left_min:
                left_min = r
            elif -math.radians(45) <= angle < 0.0 and r < right_min:
                right_min = r

        front_min = min(left_min, right_min)
        threshold = 0.5248  # 1 + bumper distance ft

        # -- Manage escape and avoid timers --
        esc_active = self.escape_end is not None and now < self.escape_end
        if self.escape_end is not None and now >= self.escape_end:
            self.escape_end = None
            self.escape_dir = 0.0
            self.halted = False

        if self.avoid_end is not None and now >= self.avoid_end:
            if front_min > threshold:
                self.avoid_end = None
                self.avoid_dir = 0.0

        avoid_active = self.avoid_end is not None

        # Check if keyboard input is recent enough
        key_age = (now - self.key_time).nanoseconds * 1e-9
        key = self.last_key_cmd
        key_active = (key_age < 0.5
                      and (abs(key.linear.x) > 0.01
                           or abs(key.angular.z) > 0.01))

        # PRIORITY 1: Halt
        # Uses actual bumper now
        if self.bumper_hit:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publish_cmd(msg)
            self.timer.cancel()
            self.get_logger().warn('Bumper hit - halting')
            return

        # PRIORITY 2: Keyboard
        elif key_active:
            msg = key
            self.escape_dir = 0.0
            self.escape_end = None
            self.halted = False
            self.avoid_dir = 0.0
            self.avoid_end = None

        # PRIORITY 3: Escape symmetric obstacles within 1 ft
        elif esc_active or (
                front_min <= threshold
                and abs(left_min - right_min) < 0.10):
            if self.escape_end is None:
                self.escape_dir = random.choice([-1.0, 1.0])
                self.escape_end = now + rclpy.duration.Duration(seconds=3.5)
                self.get_logger().info('Symmetric obstacle detected')
            msg.linear.x = 0.0
            msg.angular.z = self.escape_dir * 0.8

        # PRIORITY 4: Avoid asymmetric obstacles within 1 ft (reflex)
        elif front_min <= threshold or avoid_active:
            if self.avoid_end is None:
                self.avoid_dir = -1.0 if left_min < right_min else 1.0
                self.avoid_end = now + rclpy.duration.Duration(seconds=1.5)
                if left_min < right_min:
                    self.get_logger().info('Obstacle on left')
                else:
                    self.get_logger().info('Obstacle on right')
            msg.linear.x = 0.0
            msg.angular.z = self.avoid_dir * 0.8

        # PRIORITY 5: Random turn every 1 ft
        elif self.dist_since_turn >= threshold:
            delta = random.uniform(-math.radians(15), math.radians(15))
            msg.linear.x = 0.0
            msg.angular.z = max(-1.5, min(1.5, delta / 0.1))
            self.dist_since_turn = 0.0

        # PRIORITY 6: Drive forward
        else:
            msg.linear.x = 0.12
            msg.angular.z = 0.0

        self.publish_cmd(msg)


def main():
    # Entry point.
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()