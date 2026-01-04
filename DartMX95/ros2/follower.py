#!/usr/bin/env python3
"""
Industry-Standard Human Following Controller for WYZECAR

Based on visual servoing principles used in commercial following robots:
- Area-based distance estimation (robust to pose changes)
- Velocity matching (follows speed, not just position)  
- Exponential smoothing for jitter reduction
- Speed-coupled steering (safer turns)
- Proper state machine with hysteresis

Topics:
    Subscribed: /target_person (PointStamped)
    Published: /cmd_vel (Twist)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import time
import math


class TargetState:
    """
    Maintains smoothed target state with velocity estimation.
    Uses exponential moving average (EMA) for noise rejection.
    """
    
    def __init__(self, position_alpha=0.4, velocity_alpha=0.3):
        self.position_alpha = position_alpha  # Higher = more responsive
        self.velocity_alpha = velocity_alpha
        
        # Position state
        self.x = 0.0           # Horizontal position (-1 to 1)
        self.distance = 0.5   # Normalized distance (0=close, 1=far)
        
        # Velocity state
        self.vx = 0.0         # Horizontal velocity
        self.vd = 0.0         # Distance velocity (approach rate)
        
        # Timing
        self.last_update = 0.0
        self.valid = False
        self.update_count = 0
    
    def update(self, x: float, distance: float) -> None:
        """Update state with new measurement"""
        now = time.time()
        dt = now - self.last_update if self.last_update > 0 else 0.05
        dt = max(0.01, min(0.5, dt))  # Clamp dt
        
        if self.valid:
            # Estimate velocities
            raw_vx = (x - self.x) / dt
            raw_vd = (distance - self.distance) / dt
            
            # Smooth velocities
            self.vx = self.vx * (1 - self.velocity_alpha) + raw_vx * self.velocity_alpha
            self.vd = self.vd * (1 - self.velocity_alpha) + raw_vd * self.velocity_alpha
            
            # Smooth positions
            self.x = self.x * (1 - self.position_alpha) + x * self.position_alpha
            self.distance = self.distance * (1 - self.position_alpha) + distance * self.position_alpha
        else:
            # First update - initialize directly
            self.x = x
            self.distance = distance
            self.vx = 0.0
            self.vd = 0.0
        
        self.last_update = now
        self.valid = True
        self.update_count += 1
    
    def predict(self, dt: float) -> tuple:
        """Predict future position for lead compensation"""
        pred_x = self.x + self.vx * dt
        pred_d = self.distance + self.vd * dt
        return pred_x, max(0, min(1, pred_d))
    
    def age(self) -> float:
        """Time since last update"""
        if self.last_update <= 0:
            return float('inf')
        return time.time() - self.last_update
    
    def reset(self) -> None:
        """Reset state when target lost"""
        self.valid = False
        self.vx = 0.0
        self.vd = 0.0
        self.update_count = 0


class FollowerController(Node):
    """
    Visual servoing controller for human following.
    
    Control Strategy:
    1. Angular: Proportional control on horizontal error with velocity feedforward
    2. Linear: Proportional control on distance error with velocity matching
    3. Coupling: Reduce linear speed when turning (prevents overshooting)
    """
    
    # States
    IDLE = 0
    TRACKING = 1
    SEARCHING = 2
    STOPPED = 3  # Close enough, just track orientation
    
    STATE_NAMES = ['IDLE', 'TRACKING', 'SEARCHING', 'STOPPED']
    
    def __init__(self):
        super().__init__('follower')
        
        # === Distance Parameters ===
        # distance: 0 = very close (large bbox), 1 = far (small bbox)
        self.declare_parameter('stop_distance', 0.15)      # Stop forward motion
        self.declare_parameter('target_distance', 0.35)    # Ideal following distance
        self.declare_parameter('max_distance', 0.85)       # Beyond this, full speed
        
        # === Speed Limits ===
        self.declare_parameter('max_linear_speed', 0.7)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.25)   # Higher minimum to overcome friction
        
        # === Control Gains ===
        self.declare_parameter('angular_gain', 1.5)        # Horizontal error gain
        self.declare_parameter('angular_velocity_gain', 0.3)  # Velocity feedforward
        self.declare_parameter('linear_gain', 1.2)         # Distance error gain (higher for response)
        self.declare_parameter('linear_velocity_gain', 0.5)   # Velocity matching
        
        # === Behavior Tuning ===
        self.declare_parameter('turn_speed_coupling', 0.5) # How much turning slows forward
        self.declare_parameter('center_deadzone', 0.08)    # Ignore small horizontal errors
        self.declare_parameter('distance_deadzone', 0.05)  # Ignore small distance errors
        self.declare_parameter('prediction_time', 0.1)     # Lookahead for lead compensation
        
        # === Timeouts ===
        self.declare_parameter('lost_timeout', 1.2)        # Start searching
        self.declare_parameter('search_timeout', 6.0)      # Give up searching
        
        # Load all parameters
        self.stop_dist = self.get_parameter('stop_distance').value
        self.target_dist = self.get_parameter('target_distance').value
        self.max_dist = self.get_parameter('max_distance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_linear = self.get_parameter('min_linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.angular_vel_gain = self.get_parameter('angular_velocity_gain').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.linear_vel_gain = self.get_parameter('linear_velocity_gain').value
        self.turn_coupling = self.get_parameter('turn_speed_coupling').value
        self.center_deadzone = self.get_parameter('center_deadzone').value
        self.dist_deadzone = self.get_parameter('distance_deadzone').value
        self.pred_time = self.get_parameter('prediction_time').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_timeout = self.get_parameter('search_timeout').value
        
        # State
        self.target = TargetState()
        self.state = self.IDLE
        self.search_direction = 1
        self.search_start = 0.0
        self.last_x_direction = 0  # Remember which way target went
        
        # ROS interfaces
        self.target_sub = self.create_subscription(
            PointStamped, '/target_person', self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 25 Hz
        self.create_timer(0.04, self.control_loop)
        self.create_timer(2.0, self.log_status)
        
        self.cmd_count = 0
        
        self.get_logger().info('=== Human Following Controller ===')
        self.get_logger().info(f'Distance: stop={self.stop_dist:.2f}, target={self.target_dist:.2f}, max={self.max_dist:.2f}')
        self.get_logger().info(f'Speed: linear={self.max_linear:.2f}, angular={self.max_angular:.2f}')
        self.get_logger().info(f'Gains: angular={self.angular_gain:.2f}, linear={self.linear_gain:.2f}')
    
    def log_status(self):
        if self.target.valid:
            self.get_logger().info(
                f'[{self.STATE_NAMES[self.state]}] '
                f'x={self.target.x:+.2f} vx={self.target.vx:+.2f} | '
                f'd={self.target.distance:.2f} vd={self.target.vd:+.2f} | '
                f'cmds={self.cmd_count}'
            )
        else:
            self.get_logger().info(f'[{self.STATE_NAMES[self.state]}] No target | cmds={self.cmd_count}')
        self.cmd_count = 0
    
    def target_callback(self, msg: PointStamped):
        """Process new target detection"""
        self.target.update(msg.point.x, msg.point.z)
        
        # Remember direction for search behavior
        if abs(msg.point.x) > 0.15:
            self.last_x_direction = 1 if msg.point.x > 0 else -1
    
    def control_loop(self):
        """Main control loop - runs at 25 Hz"""
        cmd = Twist()
        age = self.target.age()
        
        # === State Machine ===
        if age < self.lost_timeout and self.target.valid:
            # Have target - check if stopped or tracking
            if self.target.distance <= self.stop_dist:
                self._transition_to(self.STOPPED)
            else:
                self._transition_to(self.TRACKING)
        elif age < self.search_timeout:
            self._transition_to(self.SEARCHING)
        else:
            self._transition_to(self.IDLE)
        
        # === Execute Behavior ===
        if self.state == self.TRACKING:
            cmd = self._compute_tracking_command()
        elif self.state == self.STOPPED:
            cmd = self._compute_stopped_command()
        elif self.state == self.SEARCHING:
            cmd = self._compute_search_command()
        # IDLE: cmd stays zero
        
        self.cmd_count += 1
        self.cmd_pub.publish(cmd)
    
    def _transition_to(self, new_state: int):
        """Handle state transitions with logging"""
        if new_state != self.state:
            self.get_logger().info(f'State: {self.STATE_NAMES[self.state]} → {self.STATE_NAMES[new_state]}')
            
            if new_state == self.SEARCHING:
                self.search_start = time.time()
                self.search_direction = self.last_x_direction if self.last_x_direction != 0 else 1
            elif new_state == self.IDLE:
                self.target.reset()
            
            self.state = new_state
    
    def _compute_tracking_command(self) -> Twist:
        """
        Compute velocity command for active tracking.
        Uses visual servoing with velocity feedforward.
        """
        cmd = Twist()
        
        # Get predicted position for lead compensation
        pred_x, pred_d = self.target.predict(self.pred_time)
        
        # === Angular Control ===
        # Error: negative x = target on left, we should turn left (positive angular)
        x_error = -pred_x
        
        # Apply deadzone
        if abs(x_error) < self.center_deadzone:
            x_error = 0.0
        
        # Proportional + velocity feedforward
        angular_cmd = (self.angular_gain * x_error + 
                      self.angular_vel_gain * (-self.target.vx))
        
        # === Linear Control ===
        # Error: positive when target is far (need to move forward)
        d_error = self.target.distance - self.target_dist
        
        # Apply deadzone
        if abs(d_error) < self.dist_deadzone:
            d_error = 0.0
        
        # Proportional + velocity matching
        # If target moving away (positive vd), speed up
        linear_cmd = (self.linear_gain * d_error + 
                     self.linear_vel_gain * self.target.vd)
        
        # === Speed Coupling ===
        # Reduce forward speed when turning to prevent overshooting
        turn_factor = 1.0 - self.turn_coupling * abs(angular_cmd / self.max_angular)
        turn_factor = max(0.3, turn_factor)  # Never reduce below 30%
        linear_cmd *= turn_factor
        
        # === Centering Priority ===
        # If target is off-center, slow down to center first
        if abs(self.target.x) > 0.4:
            center_urgency = (abs(self.target.x) - 0.4) / 0.6
            linear_cmd *= (1.0 - 0.7 * center_urgency)
        
        # === Clamp Outputs ===
        # Ensure minimum speed if we need to move (overcome static friction)
        if abs(linear_cmd) > 0.01 and abs(linear_cmd) < self.min_linear:
            linear_cmd = self.min_linear * (1 if linear_cmd > 0 else -1)
        
        cmd.linear.x = float(max(-self.max_linear, min(self.max_linear, linear_cmd)))
        cmd.angular.z = float(max(-self.max_angular, min(self.max_angular, angular_cmd)))
        
        return cmd
    
    def _compute_stopped_command(self) -> Twist:
        """
        Close to target - stop forward motion, just maintain centering.
        """
        cmd = Twist()
        
        # Still track horizontally
        x_error = -self.target.x
        if abs(x_error) > self.center_deadzone:
            angular_cmd = self.angular_gain * 0.5 * x_error  # Gentler when stopped
            cmd.angular.z = float(max(-self.max_angular * 0.5, min(self.max_angular * 0.5, angular_cmd)))
        
        return cmd
    
    def _compute_search_command(self) -> Twist:
        """
        Target lost - rotate to search in last known direction.
        """
        cmd = Twist()
        
        search_time = time.time() - self.search_start
        search_speed = 0.35  # Gentle rotation
        
        # Rotate in last known direction, flip every 2.5 seconds
        direction = self.search_direction
        if int(search_time / 2.5) % 2 == 1:
            direction = -direction
        
        cmd.angular.z = float(direction * search_speed)
        return cmd


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowerController()
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
