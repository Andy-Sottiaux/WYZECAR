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
        self.declare_parameter('max_speed_percent', 60)  # 60% max speed
        self.declare_parameter('acceleration_rate', 40.0)  # % per second (quick ramp)
        self.declare_parameter('deceleration_rate', 300.0)  # % per second (very fast braking when key released)
        self.declare_parameter('servo_slew_rate', 500.0)  # degrees per second (max speed steering)
        # Throttle mapping / stiction compensation
        self.declare_parameter('max_linear_speed', 0.6)  # m/s - 60% speed
        self.declare_parameter('throttle_expo', 0.75)  # <1 increases low-end authority (helps overcome stiction)
        self.declare_parameter('min_moving_speed_percent', 12)  # min commanded speed when nonzero (reduces PWM whine)
        self.declare_parameter('startup_kick_enabled', True)
        self.declare_parameter('startup_kick_percent', 18)  # brief initial kick to start turning
        self.declare_parameter('startup_kick_duration', 0.18)  # seconds
        # Steering mapping (cmd_vel angular.z -> servo angle)
        # IMPORTANT: These defaults are aligned with the ESP32 firmware:
        # - Firmware clamps commanded servoAngle to [0..180]
        # - Firmware maps 0..180 to a safe PWM range (SERVO_MIN_US..SERVO_MAX_US)
        # If your physical linkage binds before full lock, narrow these limits via params.
        self.declare_parameter('servo_center_angle', 90)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 180)
        # angular.z value that should correspond to full steering deflection.
        # If your controller sends smaller values (e.g. +/-0.8), reduce this to get full range.
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('command_timeout', 2.0)
        self.declare_parameter('control_rate', 50.0)  # Hz - higher rate for lower latency
        self.declare_parameter('velocity_smoothing', 0.3)  # Moderate smoothing
        # ESP32 expects periodic I2C commands; it has a ~2s watchdog.
        # Keepalive ensures motors keep running even if command values are steady.
        self.declare_parameter('i2c_keepalive_rate', 5.0)  # Hz (must be <10Hz I2C limiter)
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.esp32_address = self.get_parameter('esp32_address').get_parameter_value().integer_value
        self.max_speed_percent = self.get_parameter('max_speed_percent').get_parameter_value().integer_value
        self.accel_rate = self.get_parameter('acceleration_rate').get_parameter_value().double_value
        self.decel_rate = self.get_parameter('deceleration_rate').get_parameter_value().double_value
        self.servo_slew_rate = self.get_parameter('servo_slew_rate').get_parameter_value().double_value
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').get_parameter_value().double_value)
        self.throttle_expo = float(self.get_parameter('throttle_expo').get_parameter_value().double_value)
        self.min_moving_speed_percent = int(self.get_parameter('min_moving_speed_percent').get_parameter_value().integer_value)
        self.startup_kick_enabled = bool(self.get_parameter('startup_kick_enabled').get_parameter_value().bool_value)
        self.startup_kick_percent = int(self.get_parameter('startup_kick_percent').get_parameter_value().integer_value)
        self.startup_kick_duration = float(self.get_parameter('startup_kick_duration').get_parameter_value().double_value)
        self.servo_center = int(self.get_parameter('servo_center_angle').get_parameter_value().integer_value)
        self.servo_min = int(self.get_parameter('servo_min_angle').get_parameter_value().integer_value)
        self.servo_max = int(self.get_parameter('servo_max_angle').get_parameter_value().integer_value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').get_parameter_value().double_value)
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.velocity_smoothing = self.get_parameter('velocity_smoothing').get_parameter_value().double_value
        self.i2c_keepalive_rate = self.get_parameter('i2c_keepalive_rate').get_parameter_value().double_value
        if self.i2c_keepalive_rate <= 0:
            self.i2c_keepalive_rate = 5.0
        
        # Motion state
        self.target_speed = 0.0  # Desired speed from commands
        self.current_speed = 0.0  # Actual ramped speed
        # Validate steering params (keep safe, avoid weird inversion)
        self.servo_center = max(0, min(180, self.servo_center))
        self.servo_min = max(0, min(180, self.servo_min))
        self.servo_max = max(0, min(180, self.servo_max))
        if self.servo_min >= self.servo_max:
            self.get_logger().warn(
                f'Invalid steering limits (min={self.servo_min} >= max={self.servo_max}); falling back to 45..135'
            )
            self.servo_min = 45
            self.servo_max = 135
        if not (self.servo_min <= self.servo_center <= self.servo_max):
            self.get_logger().warn(
                f'Steering center {self.servo_center} not within [{self.servo_min},{self.servo_max}]; clamping'
            )
            self.servo_center = max(self.servo_min, min(self.servo_max, self.servo_center))
        if self.max_angular_speed <= 1e-6:
            self.get_logger().warn(f'max_angular_speed too small ({self.max_angular_speed}); using 1.0')
            self.max_angular_speed = 1.0

        # Validate throttle mapping params
        if self.max_linear_speed <= 1e-6:
            self.get_logger().warn(f'max_linear_speed too small ({self.max_linear_speed}); using 0.5')
            self.max_linear_speed = 0.5
        if not math.isfinite(self.throttle_expo) or self.throttle_expo <= 0.05:
            self.get_logger().warn(f'Invalid throttle_expo ({self.throttle_expo}); using 0.75')
            self.throttle_expo = 0.75
        self.min_moving_speed_percent = max(0, min(100, self.min_moving_speed_percent))
        self.startup_kick_percent = max(0, min(100, self.startup_kick_percent))
        if not math.isfinite(self.startup_kick_duration) or self.startup_kick_duration < 0:
            self.startup_kick_duration = 0.0

        self.target_servo = self.servo_center  # Desired servo angle
        self.current_servo = float(self.servo_center)  # Actual servo angle (float for smooth ramping)
        self.smoothed_linear = 0.0  # Smoothed input velocity
        self.smoothed_angular = 0.0
        self.last_target_speed = 0.0
        self.kick_until_time = 0.0
        self.kick_sign = 1.0
        
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
        self.get_logger().info(
            f'  Throttle: max_linear_speed={self.max_linear_speed} m/s, expo={self.throttle_expo:.2f}, '
            f'min_moving={self.min_moving_speed_percent}%'
        )
        if self.startup_kick_enabled and self.startup_kick_duration > 0 and self.startup_kick_percent > 0:
            self.get_logger().info(
                f'  Startup kick: {self.startup_kick_percent}% for {self.startup_kick_duration:.2f}s'
            )
        self.get_logger().info(
            f'  Steering: center={self.servo_center}°, min={self.servo_min}°, max={self.servo_max}° '
            f'(max_angular_speed={self.max_angular_speed})'
        )
        self.get_logger().info(f'  Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'  I2C keepalive: {self.i2c_keepalive_rate:.1f} Hz')

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
        # max_linear_speed maps to max_speed_percent
        speed_ratio = min(1.0, abs(self.smoothed_linear) / float(self.max_linear_speed))
        # Expo curve for better low-speed authority (helps overcome stiction)
        speed_ratio = math.pow(speed_ratio, self.throttle_expo) if speed_ratio > 0 else 0.0
        self.target_speed = speed_ratio * float(self.max_speed_percent)
        if self.smoothed_linear < 0:
            self.target_speed = -self.target_speed

        # Enforce a minimum moving command when non-zero (reduces PWM whine below stall torque)
        if self.min_moving_speed_percent > 0 and abs(self.target_speed) > 0.001:
            if abs(self.target_speed) < float(self.min_moving_speed_percent):
                self.target_speed = float(self.min_moving_speed_percent) * (1.0 if self.target_speed > 0 else -1.0)
        
        # Convert angular to servo angle
        # ESP32 firmware maps: 0°=1300us, 90°=1600us, 180°=1900us (1600 +/- 300)
        # We need to use full 0-180 range to get full 1300-1900 PWM range
        # angular_ratio in [-1, 1] where +/-1 is full steering deflection.
        angular_ratio = max(-1.0, min(1.0, self.smoothed_angular / float(self.max_angular_speed)))
        # Map to full servo range: 0-180 degrees
        # Simple linear mapping for predictable control
        if angular_ratio < 0:
            # Left: map [-1, 0] to [servo_min, servo_center]
            self.target_servo = int(round(self.servo_center + angular_ratio * (self.servo_center - self.servo_min)))
        else:
            # Right: map [0, 1] to [servo_center, servo_max]
            self.target_servo = int(round(self.servo_center + angular_ratio * (self.servo_max - self.servo_center)))

        # Startup kick: when going from stopped -> moving, apply a brief stronger initial command
        if self.startup_kick_enabled and self.startup_kick_duration > 0 and self.startup_kick_percent > 0:
            now = time.time()
            was_stopped = abs(self.last_target_speed) < 0.001 and abs(self.current_speed) < 0.5
            now_moving_cmd = abs(self.target_speed) >= 0.001
            if was_stopped and now_moving_cmd:
                self.kick_until_time = now + float(self.startup_kick_duration)
                self.kick_sign = 1.0 if self.target_speed > 0 else -1.0
            if not now_moving_cmd:
                self.kick_until_time = 0.0

        self.last_target_speed = float(self.target_speed)

    def control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        now = time.time()
        dt = now - self.last_control_time
        self.last_control_time = now
        
        # Check for command timeout - ramp down smoothly
        if now - self.last_cmd_time > self.command_timeout:
            self.target_speed = 0.0
            self.target_servo = self.servo_center
        
        # Apply acceleration/deceleration ramping to speed
        self.current_speed = self._ramp_speed(
            self.current_speed, 
            self.target_speed, 
            dt
        )
        
        # Ensure motors fully stop (PWM = 0) when target is zero to prevent buzzing
        # Use a small deadband to avoid PWM whine from very low values
        if abs(self.target_speed) < 0.5:
            self.current_speed = 0.0

        # If we're trying to move, don't linger below stall torque: apply a brief kick.
        if (
            self.startup_kick_enabled
            and now < self.kick_until_time
            and abs(self.target_speed) > 0.001
            and self.startup_kick_percent > 0
        ):
            kick = float(self.startup_kick_percent) * float(self.kick_sign)
            if abs(self.current_speed) < abs(kick):
                self.current_speed = kick
        
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
        # Ensure exactly 0 when stopped to prevent motor buzzing
        if abs(speed_int) < 1:
            speed_int = 0
        servo_int = max(0, min(180, servo_int))
        
        # ESP32 firmware watchdog requires periodic commands even if unchanged.
        unchanged = (speed_int == self.last_sent_speed and servo_int == self.last_sent_servo)
        
        # Rate limit I2C to max 10 Hz
        now = time.time()
        if now - self.last_i2c_time < 0.1:
            return

        # If unchanged, only send at keepalive rate (<= 10 Hz).
        keepalive_period = 1.0 / max(0.1, float(self.i2c_keepalive_rate))
        if unchanged and (now - self.last_i2c_time) < keepalive_period:
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
                    self.esp32_address, 0x01, [0, 0, int(self.servo_center)]
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
