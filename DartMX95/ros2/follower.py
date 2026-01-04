#!/usr/bin/env python3
"""
Follower Node for WYZECAR Vision-Based Following System

Receives target person position from the human detector and generates
velocity commands to follow the person while maintaining a safe distance.

Topics:
    Subscribed:
        /target_person (geometry_msgs/PointStamped): Target position (normalized)
    Published:
        /cmd_vel (geometry_msgs/Twist): Velocity commands

Usage:
    ros2 run wyzecar_control follower
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import time
import math


class PIDController:
    """Simple PID controller for smooth following"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.01
        
        # Proportional
        p = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        i = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.last_error) / dt
        d = self.kd * derivative
        
        self.last_error = error
        self.last_time = current_time
        
        return p + i + d
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')
        
        # Parameters - gentle, smooth following
        self.declare_parameter('target_distance', 0.45)  # Normalized (0=very close, 1=far)
        self.declare_parameter('max_linear_speed', 0.2)  # m/s equivalent (gentle speed)
        self.declare_parameter('max_angular_speed', 0.35)  # rad/s equivalent (smooth turns)
        self.declare_parameter('lost_timeout', 2.0)  # seconds
        self.declare_parameter('dead_zone', 0.15)  # Larger dead zone = less jitter
        
        # PID parameters - gentle, smooth control
        self.declare_parameter('linear_kp', 0.4)
        self.declare_parameter('linear_ki', 0.02)
        self.declare_parameter('linear_kd', 0.15)
        self.declare_parameter('angular_kp', 0.5)
        self.declare_parameter('angular_ki', 0.01)
        self.declare_parameter('angular_kd', 0.2)
        
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.lost_timeout = self.get_parameter('lost_timeout').get_parameter_value().double_value
        self.dead_zone = self.get_parameter('dead_zone').get_parameter_value().double_value
        
        # Initialize PID controllers
        linear_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        linear_ki = self.get_parameter('linear_ki').get_parameter_value().double_value
        linear_kd = self.get_parameter('linear_kd').get_parameter_value().double_value
        angular_kp = self.get_parameter('angular_kp').get_parameter_value().double_value
        angular_ki = self.get_parameter('angular_ki').get_parameter_value().double_value
        angular_kd = self.get_parameter('angular_kd').get_parameter_value().double_value
        
        self.linear_pid = PIDController(linear_kp, linear_ki, linear_kd)
        self.angular_pid = PIDController(angular_kp, angular_ki, angular_kd)
        
        # Subscribers
        self.target_sub = self.create_subscription(
            PointStamped,
            '/target_person',
            self.target_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State
        self.last_target_time = 0.0
        self.target_lost = True
        self.current_target = None
        
        # Timer for control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Follower Node started')
        self.get_logger().info(f'  Target distance: {self.target_distance}')
        self.get_logger().info(f'  Max linear speed: {self.max_linear}')
        self.get_logger().info(f'  Max angular speed: {self.max_angular}')
        self.get_logger().info(f'  Lost timeout: {self.lost_timeout}s')
        
        # Status logging
        self.cmd_count = 0
        self.create_timer(2.0, self.log_status)
    
    def log_status(self):
        """Periodic status logging"""
        if self.current_target and not self.target_lost:
            t = self.current_target
            self.get_logger().info(
                f'[FOLLOWER] Target: x={t["x"]:.2f} dist={t["distance"]:.2f} | Cmds sent: {self.cmd_count}'
            )
        self.cmd_count = 0

    def target_callback(self, msg):
        """Receive target person position"""
        self.current_target = {
            'x': msg.point.x,      # -1 (left) to 1 (right)
            'y': msg.point.y,      # -1 (top) to 1 (bottom)
            'distance': msg.point.z  # 0 (close) to 1 (far)
        }
        self.last_target_time = time.time()
        self.target_lost = False

    def control_loop(self):
        """Main control loop - runs at 20 Hz"""
        cmd = Twist()
        
        # Check if target is lost
        if time.time() - self.last_target_time > self.lost_timeout:
            if not self.target_lost:
                self.get_logger().warn('Target lost! Stopping.')
                self.target_lost = True
                self.linear_pid.reset()
                self.angular_pid.reset()
            
            # Publish zero velocity
            self.cmd_pub.publish(cmd)
            return
        
        if self.current_target is None:
            self.cmd_pub.publish(cmd)
            return
        
        target = self.current_target
        
        # Calculate errors
        # Angular: target x position (we want it centered at 0)
        angular_error = -target['x']  # Negative because left is negative x
        
        # Linear: distance from target (we want to maintain target_distance)
        distance_error = target['distance'] - self.target_distance
        
        # Apply dead zones
        if abs(angular_error) < self.dead_zone:
            angular_error = 0.0
        if abs(distance_error) < self.dead_zone:
            distance_error = 0.0
        
        # Compute PID outputs
        angular_cmd = self.angular_pid.compute(angular_error)
        linear_cmd = self.linear_pid.compute(distance_error)
        
        # Clamp to max speeds
        linear_cmd = max(-self.max_linear, min(self.max_linear, linear_cmd))
        angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))
        
        # If target is very close, don't move forward
        if target['distance'] < self.target_distance * 0.5:
            linear_cmd = min(0, linear_cmd)  # Only allow reversing
        
        # Set velocity command (must be float)
        cmd.linear.x = float(linear_cmd)
        cmd.angular.z = float(angular_cmd)
        
        self.cmd_count += 1
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'node' in locals() and node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

