#!/usr/bin/env python3
"""
ROS2 Motor Controller Node for WYZECAR Vision-Based Human Following System

This node runs on DART-MX95 and provides the interface between ROS2 cmd_vel
messages and the ESP32 motor controller via UART communication.

Features:
- Subscribes to geometry_msgs/Twist on /cmd_vel topic
- Converts linear/angular velocity to differential drive motor commands
- Sends UART commands to ESP32 (MOTOR A/B FORWARD/BACKWARD speed)
- Monitors ESP32 status and implements safety systems
- Publishes motor status feedback
- Watchdog timeout protection

Hardware Setup:
- ESP32 connected via UART (typically /dev/ttyUSB0 or /dev/ttyACM0)
- Baud rate: 115200
- Protocol: Text-based commands ending with \n

Usage:
    ros2 run wyzecar_control motor_controller
    
Or with custom serial port:
    ros2 run wyzecar_control motor_controller --ros-args -p serial_port:=/dev/ttyACM0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading
import time
import math

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # ROS2 Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.3)  # Distance between wheels (meters)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('command_timeout', 1.0)  # seconds
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        
        # ROS2 Subscribers and Publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.motor_status_pub = self.create_publisher(
            String,
            '/motor_status',
            10
        )
        
        # Serial connection
        self.serial_connection = None
        self.serial_lock = threading.Lock()
        
        # State tracking
        self.last_cmd_time = time.time()
        self.esp32_connected = False
        self.emergency_stop_active = False
        
        # Initialize serial connection
        self.init_serial_connection()
        
        # Start background threads
        self.start_background_threads()
        
        self.get_logger().info(f'Motor Controller Node started on {self.serial_port}')

    def init_serial_connection(self):
        """Initialize serial connection to ESP32"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            
            # Wait for ESP32 to initialize
            time.sleep(2.0)
            
            # Test communication
            if self.send_command("STATUS"):
                self.esp32_connected = True
                self.get_logger().info('ESP32 connection established')
            else:
                self.get_logger().warn('ESP32 not responding to STATUS command')
                
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.esp32_connected = False

    def send_command(self, command):
        """Send command to ESP32 and wait for response"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
            
        with self.serial_lock:
            try:
                # Send command
                self.serial_connection.write((command + '\n').encode())
                self.serial_connection.flush()
                
                # Wait for response (with timeout)
                response = self.serial_connection.readline().decode().strip()
                
                if response.startswith('OK'):
                    return True
                elif response.startswith('ERROR'):
                    self.get_logger().warn(f'ESP32 error: {response}')
                    return False
                elif response.startswith('STATUS'):
                    self.publish_motor_status(response)
                    return True
                else:
                    self.get_logger().debug(f'ESP32 response: {response}')
                    return True
                    
            except Exception as e:
                self.get_logger().error(f'Serial communication error: {e}')
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
        
        # Send motor commands to ESP32
        self.send_motor_commands(left_speed, right_speed)

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

    def send_motor_commands(self, left_speed_percent, right_speed_percent):
        """Send motor speed commands to ESP32"""
        
        # Motor A (Left)
        if left_speed_percent > 0:
            direction_a = "FORWARD"
            speed_a = abs(left_speed_percent)
        elif left_speed_percent < 0:
            direction_a = "BACKWARD"  
            speed_a = abs(left_speed_percent)
        else:
            direction_a = "STOP"
            speed_a = 0
            
        # Motor B (Right)
        if right_speed_percent > 0:
            direction_b = "FORWARD"
            speed_b = abs(right_speed_percent)
        elif right_speed_percent < 0:
            direction_b = "BACKWARD"
            speed_b = abs(right_speed_percent)
        else:
            direction_b = "STOP"
            speed_b = 0
        
        # Send commands
        cmd_a = f"MOTOR A {direction_a} {speed_a}"
        cmd_b = f"MOTOR B {direction_b} {speed_b}"
        
        success_a = self.send_command(cmd_a)
        success_b = self.send_command(cmd_b)
        
        if not (success_a and success_b):
            self.get_logger().warn('Failed to send motor commands')

    def publish_motor_status(self, status_response):
        """Parse and publish ESP32 status"""
        status_msg = String()
        status_msg.data = status_response
        self.motor_status_pub.publish(status_msg)

    def emergency_stop(self):
        """Execute emergency stop"""
        self.emergency_stop_active = True
        self.send_command("STOP")
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
                self.send_command("STATUS")
            
            time.sleep(2.0)

    def serial_reader(self):
        """Background thread to read serial data from ESP32"""
        while rclpy.ok() and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode().strip()
                    
                    if line.startswith('WATCHDOG'):
                        # ESP32 heartbeat received
                        pass
                    elif line.startswith('STATUS'):
                        self.publish_motor_status(line)
                    elif line.startswith('ERROR'):
                        self.get_logger().warn(f'ESP32 error: {line}')
                    elif line:
                        self.get_logger().debug(f'ESP32: {line}')
                        
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
                
            time.sleep(0.01)

    def start_background_threads(self):
        """Start all background monitoring threads"""
        
        # Watchdog monitor thread
        watchdog_thread = threading.Thread(target=self.watchdog_monitor, daemon=True)
        watchdog_thread.start()
        
        # Status monitor thread  
        status_thread = threading.Thread(target=self.status_monitor, daemon=True)
        status_thread.start()
        
        # Serial reader thread
        if self.serial_connection:
            serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            serial_thread.start()

    def destroy_node(self):
        """Clean shutdown"""
        if self.serial_connection and self.serial_connection.is_open:
            self.send_command("STOP")  # Stop motors before shutdown
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_controller = MotorControllerNode()
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
