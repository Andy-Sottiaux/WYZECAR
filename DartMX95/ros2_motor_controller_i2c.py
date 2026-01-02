#!/usr/bin/env python3
"""
ROS2 Motor Controller Node for WYZECAR Vision-Based Human Following System

This node runs on DART-MX95 and provides the interface between ROS2 cmd_vel
messages and the ESP32 motor controller via I2C communication.

Features:
- Subscribes to geometry_msgs/Twist on /cmd_vel topic
- Converts linear/angular velocity to differential drive motor commands
- Sends I2C commands to ESP32 (motor speeds and servo control)
- Monitors ESP32 status and implements safety systems
- Publishes motor status feedback
- Watchdog timeout protection

Hardware Setup:
- ESP32 connected via I2C3 bus (/dev/i2c-3)
- ESP32 I2C slave address: 0x42
- Protocol: Binary commands with status responses

Usage:
    ros2 run wyzecar_control motor_controller_i2c
    
Or with custom I2C bus:
    ros2 run wyzecar_control motor_controller_i2c --ros-args -p i2c_bus:=3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8MultiArray
import threading
import time
import math
import struct

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

class MotorControllerI2CNode(Node):
    def __init__(self):
        super().__init__('motor_controller_i2c')
        
        # Check if smbus2 is available
        if not SMBUS_AVAILABLE:
            self.get_logger().error('smbus2 not installed. Install with: pip3 install smbus2')
            return
        
        # ROS2 Parameters
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('esp32_address', 0x42)
        self.declare_parameter('wheel_base', 0.3)  # Distance between wheels (meters)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('command_timeout', 2.0)  # seconds
        self.declare_parameter('servo_center', 90)  # Center position (1500µs)
        self.declare_parameter('servo_range', 90)   # Full range: 0-180° maps to 1300-1700µs
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.esp32_address = self.get_parameter('esp32_address').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        self.servo_center = self.get_parameter('servo_center').get_parameter_value().integer_value
        self.servo_range = self.get_parameter('servo_range').get_parameter_value().integer_value
        
        # ROS2 Subscribers and Publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.motor_status_pub = self.create_publisher(
            UInt8MultiArray,
            '/motor_status',
            10
        )
        
        self.motor_debug_pub = self.create_publisher(
            String,
            '/motor_debug',
            10
        )
        
        # I2C connection
        self.i2c_bus_handle = None
        self.i2c_lock = threading.Lock()
        
        # State tracking
        self.last_cmd_time = time.time()
        self.esp32_connected = False
        self.emergency_stop_active = False
        self.current_servo_angle = 90  # Center position
        
        # Initialize I2C connection
        self.init_i2c_connection()
        
        # Start background threads
        self.start_background_threads()
        
        self.get_logger().info(f'Motor Controller I2C Node started on bus {self.i2c_bus}, address 0x{self.esp32_address:02X}')
        self.get_logger().info(f'Servo: {self.servo_center}° center, ±{self.servo_range}° range (ESP32 limits to 1300-1700µs)')

    def init_i2c_connection(self):
        """Initialize I2C connection to ESP32"""
        try:
            self.i2c_bus_handle = smbus2.SMBus(self.i2c_bus)
            
            # Test communication with status request
            if self.request_status():
                self.esp32_connected = True
                self.get_logger().info('ESP32 I2C connection established')
            else:
                self.get_logger().warn('ESP32 not responding on I2C')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C bus {self.i2c_bus}: {e}')
            self.esp32_connected = False

    def send_motor_command(self, left_speed, right_speed, servo_angle=None):
        """Send motor speed command via I2C"""
        if not self.i2c_bus_handle:
            return False
            
        if servo_angle is None:
            servo_angle = self.current_servo_angle
        else:
            self.current_servo_angle = servo_angle
            
        # Clamp values to valid ranges
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        servo_angle = max(0, min(180, servo_angle))
        
        # Convert to unsigned bytes for I2C transmission
        left_byte = left_speed if left_speed >= 0 else 256 + left_speed
        right_byte = right_speed if right_speed >= 0 else 256 + right_speed
        
        command = [0x01, left_byte, right_byte, servo_angle]
        
        with self.i2c_lock:
            try:
                self.i2c_bus_handle.write_i2c_block_data(self.esp32_address, command[0], command[1:])
                return True
            except Exception as e:
                self.get_logger().error(f'I2C write error: {e}')
                return False

    def send_emergency_stop(self):
        """Send emergency stop command via I2C"""
        if not self.i2c_bus_handle:
            return False
            
        with self.i2c_lock:
            try:
                self.i2c_bus_handle.write_byte(self.esp32_address, 0x02)
                return True
            except Exception as e:
                self.get_logger().error(f'I2C emergency stop error: {e}')
                return False

    def request_status(self):
        """Request status from ESP32 via I2C"""
        if not self.i2c_bus_handle:
            return False
            
        with self.i2c_lock:
            try:
                # Send status request
                self.i2c_bus_handle.write_byte(self.esp32_address, 0x03)
                
                # Read response (3 bytes)
                time.sleep(0.01)  # Small delay for ESP32 to prepare response
                response = self.i2c_bus_handle.read_i2c_block_data(self.esp32_address, 0, 3)
                
                # Parse response
                left_speed_raw = response[0]
                right_speed_raw = response[1]
                status_flags = response[2]
                
                # Convert back to signed speeds
                left_speed = left_speed_raw if left_speed_raw < 128 else left_speed_raw - 256
                right_speed = right_speed_raw if right_speed_raw < 128 else right_speed_raw - 256
                
                # Parse status flags
                motor_left_enabled = bool(status_flags & 0x01)
                motor_right_enabled = bool(status_flags & 0x02)
                emergency_stop = bool(status_flags & 0x04)
                motor_fault = bool(status_flags & 0x08)
                
                self.publish_motor_status(left_speed, right_speed, status_flags)
                
                # Debug output
                debug_msg = String()
                debug_msg.data = f"L:{left_speed}% R:{right_speed}% Flags:0x{status_flags:02X}"
                self.motor_debug_pub.publish(debug_msg)
                
                return True
                
            except Exception as e:
                self.get_logger().error(f'I2C status request error: {e}')
                return False

    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands"""
        if self.emergency_stop_active:
            return
            
        # Update command timestamp
        self.last_cmd_time = time.time()
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Clamp velocities to maximum limits
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        # Convert to differential drive motor speeds
        left_speed, right_speed = self.diff_drive_kinematics(linear_x, angular_z)
        
        # Convert angular velocity to servo angle (limited range around center)
        # angular_z: negative = turn left, positive = turn right
        # servo: < 90 = left, > 90 = right
        normalized_angular = angular_z / self.max_angular_speed  # -1 to 1
        servo_offset = int(normalized_angular * self.servo_range)
        servo_angle = self.servo_center + servo_offset
        servo_angle = max(self.servo_center - self.servo_range, 
                         min(self.servo_center + self.servo_range, servo_angle))
        
        # Send motor command to ESP32 with servo angle
        success = self.send_motor_command(left_speed, right_speed, servo_angle)
        
        if not success:
            self.get_logger().warn('Failed to send I2C motor command')

    def diff_drive_kinematics(self, linear_x, angular_z):
        """
        Convert linear and angular velocity to left/right wheel speeds
        
        Differential drive kinematics:
        left_speed = linear_x - (angular_z * wheel_base / 2)
        right_speed = linear_x + (angular_z * wheel_base / 2)
        """
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)
        
        # Convert to percentage (assuming max speed = max_linear_speed)
        left_percent = int((left_speed / self.max_linear_speed) * 100)
        right_percent = int((right_speed / self.max_linear_speed) * 100)
        
        # Clamp to -100 to +100
        left_percent = max(-100, min(100, left_percent))
        right_percent = max(-100, min(100, right_percent))
        
        return left_percent, right_percent

    def publish_motor_status(self, left_speed, right_speed, status_flags):
        """Publish ESP32 motor status"""
        status_msg = UInt8MultiArray()
        
        # Convert signed speeds back to unsigned for publishing
        left_byte = left_speed if left_speed >= 0 else 256 + left_speed
        right_byte = right_speed if right_speed >= 0 else 256 + right_speed
        
        status_msg.data = [left_byte, right_byte, status_flags]
        self.motor_status_pub.publish(status_msg)

    def emergency_stop(self):
        """Execute emergency stop"""
        self.emergency_stop_active = True
        self.send_emergency_stop()
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def reset_emergency_stop(self):
        """Reset emergency stop (manual reset required)"""
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop reset')

    def watchdog_monitor(self):
        """Background thread to monitor command timeouts"""
        while rclpy.ok():
            current_time = time.time()
            
            # Check for command timeout
            if current_time - self.last_cmd_time > self.command_timeout:
                if not self.emergency_stop_active:
                    self.get_logger().warn('Command timeout - executing emergency stop')
                    self.emergency_stop()
            
            time.sleep(0.1)

    def status_monitor(self):
        """Background thread to monitor ESP32 status"""
        while rclpy.ok():
            if self.esp32_connected:
                # Request status every 2 seconds
                self.request_status()
            
            time.sleep(2.0)

    def start_background_threads(self):
        """Start all background monitoring threads"""
        
        # Watchdog monitor thread
        watchdog_thread = threading.Thread(target=self.watchdog_monitor, daemon=True)
        watchdog_thread.start()
        
        # Status monitor thread  
        status_thread = threading.Thread(target=self.status_monitor, daemon=True)
        status_thread.start()

    def destroy_node(self):
        """Clean shutdown"""
        if self.i2c_bus_handle:
            self.send_emergency_stop()  # Stop motors before shutdown
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_controller = MotorControllerI2CNode()
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()