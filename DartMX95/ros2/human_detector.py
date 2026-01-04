#!/usr/bin/env python3
"""
Motion-Based Detector for WYZECAR

Ultra-lightweight motion detection - no neural networks!
Follows the largest moving object in the frame.

Topics:
    Subscribed:
        /image_raw (sensor_msgs/Image): Camera frames
    Published:
        /target_person (geometry_msgs/PointStamped): Target position
        /debug_image (sensor_msgs/Image): Annotated debug image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class MotionDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')
        
        # Parameters
        self.declare_parameter('motion_threshold', 25)  # Pixel difference threshold
        self.declare_parameter('min_area', 1500)  # Minimum motion area to track
        self.declare_parameter('blur_size', 21)  # Gaussian blur size
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('disable_detection', False)
        
        self.motion_threshold = self.get_parameter('motion_threshold').get_parameter_value().integer_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.blur_size = self.get_parameter('blur_size').get_parameter_value().integer_value
        self.process_every_n = self.get_parameter('process_every_n_frames').get_parameter_value().integer_value
        self.disable_detection = self.get_parameter('disable_detection').get_parameter_value().bool_value
        
        # Make blur size odd
        if self.blur_size % 2 == 0:
            self.blur_size += 1
        
        self.bridge = CvBridge()
        
        # Motion detection state
        self.prev_gray = None
        self.target_box = None  # (x, y, w, h) of current target
        self.target_lost_time = 0.0
        self.smoothed_center = None  # Smoothed target center
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        
        # Publishers
        self.target_pub = self.create_publisher(PointStamped, '/target_person', 10)
        self.debug_image_pub = self.create_publisher(Image, '/debug_image', 10)
        
        # Stats
        self.frame_count = 0
        self.detection_count = 0
        self.last_image_time = 0.0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0
        
        # Timer for status logging
        self.create_timer(2.0, self.log_status)
        
        self.get_logger().info('Motion Detector started (lightweight, no AI)')
        self.get_logger().info(f'  Motion threshold: {self.motion_threshold}')
        self.get_logger().info(f'  Min area: {self.min_area}')
        self.get_logger().info(f'  Process every {self.process_every_n} frames')

    def image_callback(self, msg):
        """Process incoming camera frames."""
        self.frame_count += 1
        self.last_image_time = time.time()
        
        # Frame skipping
        if self.frame_count % self.process_every_n != 0:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        height, width = frame.shape[:2]
        
        # Convert to grayscale and blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur_size, self.blur_size), 0)
        
        # Initialize previous frame
        if self.prev_gray is None:
            self.prev_gray = gray
            return
        
        # Detect motion
        target = None
        if not self.disable_detection:
            target = self._detect_motion(gray, width, height)
        
        # Update previous frame (use running average for stability)
        self.prev_gray = cv2.addWeighted(self.prev_gray, 0.7, gray, 0.3, 0)
        
        # Publish target if found
        if target:
            self.detection_count += 1
            self.target_box = target['bbox']
            self.target_lost_time = 0.0
            
            # Smooth the center position
            new_center = (target['center_x'], target['center_y'])
            if self.smoothed_center is None:
                self.smoothed_center = new_center
            else:
                alpha = 0.4  # Smoothing factor
                self.smoothed_center = (
                    alpha * new_center[0] + (1 - alpha) * self.smoothed_center[0],
                    alpha * new_center[1] + (1 - alpha) * self.smoothed_center[1]
                )
            
            # Calculate normalized position
            norm_x = (self.smoothed_center[0] / width) * 2 - 1
            norm_y = (self.smoothed_center[1] / height) * 2 - 1
            
            # Estimate distance based on bounding box height
            bbox_height = target['bbox'][3]
            distance = 1.0 - (bbox_height / height)
            
            # Publish
            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.point.x = float(norm_x)
            point_msg.point.y = float(norm_y)
            point_msg.point.z = float(distance)
            self.target_pub.publish(point_msg)
        else:
            self.target_lost_time += 0.1  # Approximate
            if self.target_lost_time > 1.0:
                self.target_box = None
                self.smoothed_center = None
        
        # Calculate FPS
        self.fps_frame_count += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.fps_frame_count / (now - self.last_fps_time)
            self.fps_frame_count = 0
            self.last_fps_time = now
        
        # Publish debug image
        self._publish_debug(frame, target, msg.header)

    def _detect_motion(self, gray: np.ndarray, width: int, height: int) -> dict:
        """Detect the largest moving region in the frame."""
        # Frame difference
        frame_diff = cv2.absdiff(self.prev_gray, gray)
        
        # Threshold
        _, thresh = cv2.threshold(frame_diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
        
        # Dilate to fill gaps
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find the largest contour above minimum area
        best_contour = None
        best_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area and area > best_area:
                best_area = area
                best_contour = contour
        
        if best_contour is None:
            return None
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(best_contour)
        center_x = x + w // 2
        center_y = y + h // 2
        
        return {
            'bbox': (x, y, w, h),
            'center_x': center_x,
            'center_y': center_y,
            'area': best_area
        }

    def _publish_debug(self, frame: np.ndarray, target: dict, header):
        """Publish annotated debug image."""
        height, width = frame.shape[:2]
        
        # Draw target box
        if target:
            x, y, w, h = target['bbox']
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(frame, "TARGET", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw center point
            if self.smoothed_center:
                cx, cy = int(self.smoothed_center[0]), int(self.smoothed_center[1])
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
        elif self.target_box and self.target_lost_time < 1.0:
            # Show last known position fading
            x, y, w, h = self.target_box
            alpha = 1.0 - self.target_lost_time
            color = (0, int(255 * alpha), int(255 * (1 - alpha)))
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, "LOST?", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Draw crosshair
        cv2.line(frame, (width//2, 0), (width//2, height), (100, 100, 100), 1)
        cv2.line(frame, (0, height//2), (width, height//2), (100, 100, 100), 1)
        
        # Status overlay
        status = f"Motion: {self.fps:.1f} fps"
        if target:
            status += f" | TRACKING (area: {target['area']:.0f})"
        else:
            status += " | No motion"
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Publish
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception:
            pass

    def log_status(self):
        """Log status periodically."""
        now = time.time()
        img_age = (now - self.last_image_time) if self.last_image_time > 0 else -1.0
        
        status = "TRACKING" if self.target_box else "SEARCHING"
        self.get_logger().info(
            f'[MOTION] Frames:{self.frame_count} | FPS:{self.fps:.1f} | '
            f'Status:{status} | ImgAge:{img_age:.1f}s'
        )
        
        if img_age > 2.0:
            self.get_logger().warn(f'[MOTION] Camera stream stale!')
        
        self.detection_count = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotionDetectorNode()
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
