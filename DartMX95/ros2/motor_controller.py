#!/usr/bin/env python3
"""
Smooth Motor Controller for WYZECAR

Industry-standard motion control with:
- Acceleration/deceleration ramping
- Velocity smoothing
- Gentle starts and stops
- Power-friendly operation

Hardware Setup:
- ESP32 connected via I2C3 bus (/dev/i2c-3)
- ESP32 I2C slave address: 0x42
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8MultiArray
import threading
import time
import math

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


class SmoothMotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        if not SMBUS_AVAILABLE:
            self.get_logger().error('smbus2 not installed!')
            return
        
        # ROS2 Parameters
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('esp32_address', 0x42)
        self.declare_parameter('max_speed_percent', 70)  # Strong tracking speed
        self.declare_parameter('acceleration_rate', 30.0)  # % per second (responsive ramp)
        self.declare_parameter('deceleration_rate', 35.0)  # % per second (responsive braking)
        self.declare_parameter('servo_slew_rate', 120.0)  # degrees per second (responsive steering)
        self.declare_parameter('command_timeout', 2.0)
        self.declare_parameter('control_rate', 25.0)  # Hz - smooth motion
        self.declare_parameter('velocity_smoothing', 0.3)  # Moderate smoothing
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.esp32_address = self.get_parameter('esp32_address').get_parameter_value().integer_value
        self.max_speed_percent = self.get_parameter('max_speed_percent').get_parameter_value().integer_value
        self.accel_rate = self.get_parameter('acceleration_rate').get_parameter_value().double_value
        self.decel_rate = self.get_parameter('deceleration_rate').get_parameter_value().double_value
        self.servo_slew_rate = self.get_parameter('servo_slew_rate').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.velocity_smoothing = self.get_parameter('velocity_smoothing').get_parameter_value().double_value
        
        # Motion state
        self.target_speed = 0.0  # Desired speed from commands
        self.current_speed = 0.0  # Actual ramped speed
        self.target_servo = 90  # Desired servo angle
        self.current_servo = 90.0  # Actual servo angle (float for smooth ramping)
        self.smoothed_linear = 0.0  # Smoothed input velocity
        self.smoothed_angular = 0.0
        
        # Timing
        self.last_cmd_time = time.time()
        self.last_control_time = time.time()
        self.last_i2c_time = 0.0
        
        # I2C
        self.i2c_bus_handle = None
        self.i2c_lock = threading.Lock()
        self.i2c_errors = 0
        
        # Initialize I2C
        self._init_i2c()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.debug_pub = self.create_publisher(String, '/motor_debug', 10)
        
        # Control loop timer (runs at control_rate Hz)
        control_period = 1.0 / self.control_rate
        self.create_timer(control_period, self.control_loop)
        self.create_timer(2.0, self.log_status)
        
        # Stats
        self.cmd_count = 0
        self.last_sent_speed = 0
        self.last_sent_servo = 90
        
        self.get_logger().info('Smooth Motor Controller started')
        self.get_logger().info(f'  Max speed: {self.max_speed_percent}%')
        self.get_logger().info(f'  Acceleration: {self.accel_rate}%/s')
        self.get_logger().info(f'  Deceleration: {self.decel_rate}%/s')
        self.get_logger().info(f'  Servo slew: {self.servo_slew_rate}°/s')
        self.get_logger().info(f'  Control rate: {self.control_rate} Hz')

    def _init_i2c(self):
        """Initialize I2C connection."""
        try:
            self.i2c_bus_handle = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f'I2C bus {self.i2c_bus} opened, address 0x{self.esp32_address:02X}')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C: {e}')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands - apply smoothing."""
        self.last_cmd_time = time.time()
        self.cmd_count += 1
        
        # Apply exponential smoothing to input commands
        alpha = self.velocity_smoothing
        self.smoothed_linear = alpha * msg.linear.x + (1 - alpha) * self.smoothed_linear
        self.smoothed_angular = alpha * msg.angular.z + (1 - alpha) * self.smoothed_angular
        
        # Convert smoothed velocity to target speed percentage
        # Assuming max_linear_speed of 0.5 m/s maps to max_speed_percent
        speed_ratio = min(1.0, abs(self.smoothed_linear) / 0.5)
        self.target_speed = speed_ratio * self.max_speed_percent
        if self.smoothed_linear < 0:
            self.target_speed = -self.target_speed
        
        # Convert angular to servo angle (90 = center, range 45-135)
        # Assuming max_angular_speed of 1.0 rad/s
        angular_ratio = max(-1.0, min(1.0, self.smoothed_angular / 1.0))
        self.target_servo = 90 + int(angular_ratio * 45)

    def control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        now = time.time()
        dt = now - self.last_control_time
        self.last_control_time = now
        
        # Check for command timeout - ramp down smoothly
        if now - self.last_cmd_time > self.command_timeout:
            self.target_speed = 0.0
            self.target_servo = 90
        
        # Apply acceleration/deceleration ramping to speed
        self.current_speed = self._ramp_speed(
            self.current_speed, 
            self.target_speed, 
            dt
        )
        
        # Apply slew rate limiting to servo
        self.current_servo = self._ramp_servo(
            self.current_servo,
            float(self.target_servo),
            dt
        )
        
        # Send to motors
        self._send_command()

    def _ramp_speed(self, current: float, target: float, dt: float) -> float:
        """Ramp speed with acceleration/deceleration limits."""
        diff = target - current
        
        if abs(diff) < 0.5:
            return target  # Close enough
        
        # Determine rate based on acceleration vs deceleration
        if abs(target) > abs(current):
            # Accelerating (or changing direction)
            rate = self.accel_rate
        else:
            # Decelerating
            rate = self.decel_rate
        
        # Calculate maximum change this timestep
        max_change = rate * dt
        
        # Apply limited change
        if diff > 0:
            return min(current + max_change, target)
        else:
            return max(current - max_change, target)

    def _ramp_servo(self, current: float, target: float, dt: float) -> float:
        """Ramp servo angle with slew rate limiting."""
        diff = target - current
        
        if abs(diff) < 0.5:
            return target
        
        max_change = self.servo_slew_rate * dt
        
        if diff > 0:
            return min(current + max_change, target)
        else:
            return max(current - max_change, target)

    def _send_command(self):
        """Send motor command via I2C."""
        if not self.i2c_bus_handle:
            return
        
        # Convert to integer values
        speed_int = int(round(self.current_speed))
        servo_int = int(round(self.current_servo))
        
        # Clamp values
        speed_int = max(-100, min(100, speed_int))
        servo_int = max(0, min(180, servo_int))
        
        # Only send if values changed (reduce I2C traffic)
        if speed_int == self.last_sent_speed and servo_int == self.last_sent_servo:
            return
        
        # Rate limit I2C to max 10 Hz
        now = time.time()
        if now - self.last_i2c_time < 0.1:
            return
        
        # Convert signed speed to unsigned byte
        speed_byte = speed_int if speed_int >= 0 else 256 + speed_int
        
        # Send command: [CMD=0x01, rear_motor, front_motor, servo]
        # Both motors get same speed (car-style steering)
        command = [speed_byte, speed_byte, servo_int]
        
        with self.i2c_lock:
            try:
                self.i2c_bus_handle.write_i2c_block_data(
                    self.esp32_address, 0x01, command
                )
                self.last_sent_speed = speed_int
                self.last_sent_servo = servo_int
                self.last_i2c_time = now
                self.i2c_errors = 0
            except Exception as e:
                self.i2c_errors += 1
                if self.i2c_errors <= 3:
                    self.get_logger().warn(f'I2C error: {e}')

    def log_status(self):
        """Log status periodically."""
        age = time.time() - self.last_cmd_time
        self.get_logger().info(
            f'[MOTOR] Speed: {self.current_speed:.1f}% (target: {self.target_speed:.1f}%) | '
            f'Servo: {self.current_servo:.0f}° | CmdAge: {age:.1f}s | Cmds: {self.cmd_count}'
        )
        self.cmd_count = 0

    def destroy_node(self):
        """Clean shutdown - stop motors."""
        if self.i2c_bus_handle:
            try:
                # Send stop command
                self.i2c_bus_handle.write_i2c_block_data(
                    self.esp32_address, 0x01, [0, 0, 90]
                )
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SmoothMotorController()
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
