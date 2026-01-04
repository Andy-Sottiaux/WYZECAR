#!/usr/bin/env python3
"""
Advanced Follower Node for WYZECAR

Features:
- Target position smoothing (filters jitter)
- Velocity estimation (anticipates movement)
- Search behavior (rotates when target lost)
- Adaptive speed (slows when turning sharply)
- Priority on keeping target in frame

Topics:
    Subscribed:
        /target_person (geometry_msgs/PointStamped): Target position
    Published:
        /cmd_vel (geometry_msgs/Twist): Velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import time
import math


class TargetTracker:
    """Smooths target position and estimates velocity"""
    
    def __init__(self, smoothing=0.3):
        self.smoothing = smoothing  # 0-1, lower = more smoothing
        self.x = 0.0
        self.y = 0.0
        self.distance = 0.5
        self.vx = 0.0  # Velocity in x
        self.last_update = 0.0
        self.valid = False
    
    def update(self, x, y, distance):
        now = time.time()
        dt = now - self.last_update if self.last_update > 0 else 0.05
        
        if self.valid and dt > 0:
            # Estimate velocity
            new_vx = (x - self.x) / dt
            self.vx = self.vx * 0.7 + new_vx * 0.3  # Smooth velocity
        
        # Exponential smoothing
        if self.valid:
            self.x = self.x * (1 - self.smoothing) + x * self.smoothing
            self.y = self.y * (1 - self.smoothing) + y * self.smoothing
            self.distance = self.distance * (1 - self.smoothing) + distance * self.smoothing
        else:
            self.x = x
            self.y = y
            self.distance = distance
        
        self.last_update = now
        self.valid = True
    
    def predict(self, dt=0.1):
        """Predict future position"""
        return self.x + self.vx * dt
    
    def age(self):
        """Time since last update"""
        return time.time() - self.last_update if self.last_update > 0 else 999
    
    def reset(self):
        self.valid = False
        self.vx = 0.0


class PIDController:
    """PID controller with anti-windup"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.1, max_integral=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
    
    def compute(self, error, dt=None):
        now = time.time()
        if dt is None:
            dt = now - self.last_time
        if dt <= 0:
            dt = 0.01
        
        # Proportional
        p = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i = self.ki * self.integral
        
        # Derivative (with filtering)
        derivative = (error - self.last_error) / dt
        d = self.kd * derivative
        
        self.last_error = error
        self.last_time = now
        
        return p + i + d
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()


class FollowerNode(Node):
    # States
    STATE_IDLE = 0
    STATE_TRACKING = 1
    STATE_SEARCHING = 2
    
    def __init__(self):
        super().__init__('follower')
        
        # Core parameters
        self.declare_parameter('target_distance', 0.4)
        self.declare_parameter('max_linear_speed', 0.55)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('search_timeout', 8.0)
        
        # Tuning
        self.declare_parameter('centering_priority', 0.7)  # How much to prioritize centering vs distance
        self.declare_parameter('turn_slowdown', 0.6)  # Slow forward speed when turning hard
        self.declare_parameter('prediction_time', 0.15)  # How far ahead to predict
        
        # PID gains
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('angular_ki', 0.05)
        self.declare_parameter('angular_kd', 0.2)
        self.declare_parameter('linear_kp', 0.6)
        self.declare_parameter('linear_ki', 0.03)
        self.declare_parameter('linear_kd', 0.15)
        
        # Load parameters
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.lost_timeout = self.get_parameter('lost_timeout').get_parameter_value().double_value
        self.search_timeout = self.get_parameter('search_timeout').get_parameter_value().double_value
        self.centering_priority = self.get_parameter('centering_priority').get_parameter_value().double_value
        self.turn_slowdown = self.get_parameter('turn_slowdown').get_parameter_value().double_value
        self.prediction_time = self.get_parameter('prediction_time').get_parameter_value().double_value
        
        # Initialize controllers
        self.angular_pid = PIDController(
            self.get_parameter('angular_kp').get_parameter_value().double_value,
            self.get_parameter('angular_ki').get_parameter_value().double_value,
            self.get_parameter('angular_kd').get_parameter_value().double_value
        )
        self.linear_pid = PIDController(
            self.get_parameter('linear_kp').get_parameter_value().double_value,
            self.get_parameter('linear_ki').get_parameter_value().double_value,
            self.get_parameter('linear_kd').get_parameter_value().double_value
        )
        
        # Target tracker with smoothing
        self.tracker = TargetTracker(smoothing=0.4)
        
        # State machine
        self.state = self.STATE_IDLE
        self.search_start_time = 0.0
        self.search_direction = 1  # 1 = right, -1 = left
        self.last_known_direction = 0  # Where target was last seen
        
        # ROS interfaces
        self.target_sub = self.create_subscription(
            PointStamped, '/target_person', self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 25 Hz
        self.create_timer(0.04, self.control_loop)
        self.create_timer(2.0, self.log_status)
        
        self.cmd_count = 0
        self.get_logger().info('Advanced Follower started')
        self.get_logger().info(f'  Target distance: {self.target_distance}')
        self.get_logger().info(f'  Max speeds: linear={self.max_linear}, angular={self.max_angular}')
    
    def log_status(self):
        state_names = ['IDLE', 'TRACKING', 'SEARCHING']
        if self.tracker.valid:
            self.get_logger().info(
                f'[FOLLOWER] {state_names[self.state]} | '
                f'x={self.tracker.x:.2f} vx={self.tracker.vx:.2f} dist={self.tracker.distance:.2f} | '
                f'cmds={self.cmd_count}'
            )
        else:
            self.get_logger().info(f'[FOLLOWER] {state_names[self.state]} | No target | cmds={self.cmd_count}')
        self.cmd_count = 0
    
    def target_callback(self, msg):
        """Update tracker with new detection"""
        self.tracker.update(msg.point.x, msg.point.y, msg.point.z)
        
        # Remember which direction target was last seen
        if abs(msg.point.x) > 0.1:
            self.last_known_direction = 1 if msg.point.x > 0 else -1
    
    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        target_age = self.tracker.age()
        
        # State transitions
        if target_age < self.lost_timeout:
            self.state = self.STATE_TRACKING
        elif target_age < self.search_timeout:
            if self.state != self.STATE_SEARCHING:
                self.state = self.STATE_SEARCHING
                self.search_start_time = time.time()
                # Search in direction target was last seen
                self.search_direction = self.last_known_direction if self.last_known_direction != 0 else 1
                self.get_logger().info(f'Target lost, searching {"right" if self.search_direction > 0 else "left"}...')
        else:
            if self.state != self.STATE_IDLE:
                self.state = self.STATE_IDLE
                self.get_logger().warn('Search timeout, going idle')
                self.tracker.reset()
                self.angular_pid.reset()
                self.linear_pid.reset()
        
        # Execute state behavior
        if self.state == self.STATE_TRACKING:
            cmd = self._tracking_behavior()
        elif self.state == self.STATE_SEARCHING:
            cmd = self._searching_behavior()
        # IDLE = stop (cmd is already zero)
        
        self.cmd_count += 1
        self.cmd_pub.publish(cmd)
    
    def _tracking_behavior(self):
        """Active tracking - keep target centered and at distance"""
        cmd = Twist()
        
        # Use predicted position for smoother tracking
        predicted_x = self.tracker.predict(self.prediction_time)
        
        # Angular control - keep target centered (PRIORITY)
        # Negative because positive x = right, we turn right (negative angular)
        angular_error = -predicted_x
        angular_cmd = self.angular_pid.compute(angular_error)
        
        # Linear control - maintain distance
        distance_error = self.tracker.distance - self.target_distance
        linear_cmd = self.linear_pid.compute(distance_error)
        
        # Adaptive speed - slow down when turning hard (keeps target in frame)
        turn_factor = 1.0 - abs(angular_cmd / self.max_angular) * (1.0 - self.turn_slowdown)
        linear_cmd *= turn_factor
        
        # Prioritize centering over forward motion
        # If target is off-center, reduce forward speed
        off_center = abs(self.tracker.x)
        if off_center > 0.3:
            center_factor = 1.0 - (off_center - 0.3) * self.centering_priority
            linear_cmd *= max(0.2, center_factor)
        
        # Don't drive forward into target
        if self.tracker.distance < self.target_distance * 0.4:
            linear_cmd = min(0, linear_cmd)  # Only reverse
        
        # Clamp
        cmd.linear.x = float(max(-self.max_linear, min(self.max_linear, linear_cmd)))
        cmd.angular.z = float(max(-self.max_angular, min(self.max_angular, angular_cmd)))
        
        return cmd
    
    def _searching_behavior(self):
        """Search for lost target by rotating"""
        cmd = Twist()
        
        # Rotate in direction target was last seen
        search_speed = 0.4  # Slower rotation for search
        cmd.angular.z = float(self.search_direction * search_speed)
        
        # Alternate direction periodically
        search_duration = time.time() - self.search_start_time
        if search_duration > 3.0:
            # Flip direction every 3 seconds
            cycles = int(search_duration / 3.0)
            if cycles % 2 == 1:
                cmd.angular.z = -cmd.angular.z
        
        return cmd


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
