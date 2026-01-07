#!/usr/bin/env python3
"""
Image Preprocessor Node for WYZECAR

Handles camera frame orientation correction at the source.
This ensures all downstream nodes receive correctly oriented frames.

Industry-standard approach: separate preprocessing from processing logic.

Topics:
    Subscribed:
        /image_raw (sensor_msgs/Image): Raw camera frames
    Published:
        /image_corrected (sensor_msgs/Image): Correctly oriented frames
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePreprocessorNode(Node):
    def __init__(self):
        super().__init__('image_preprocessor')
        
        # Parameters
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)
        self.declare_parameter('rotate_180', True)  # Default: flip both axes (180° rotation)
        
        self.flip_h = self.get_parameter('flip_horizontal').get_parameter_value().bool_value
        self.flip_v = self.get_parameter('flip_vertical').get_parameter_value().bool_value
        self.rotate_180 = self.get_parameter('rotate_180').get_parameter_value().bool_value
        
        self.bridge = CvBridge()
        
        # Subscriber - raw camera feed (topic name can be remapped)
        # Default: /image_raw, but can be remapped to /camera/image_raw via --ros-args
        self.image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        
        # Publisher - corrected feed (publishes to /image_raw for downstream nodes)
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        
        flip_mode = "180° rotation" if self.rotate_180 else f"H:{self.flip_h} V:{self.flip_v}"
        self.get_logger().info(f'Image Preprocessor started - Orientation: {flip_mode}')
    
    def image_callback(self, msg):
        """Process incoming frame and apply orientation correction."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Apply orientation correction
            if self.rotate_180:
                # Flip both axes (equivalent to 180° rotation)
                cv_image = cv2.flip(cv_image, -1)
            else:
                # Apply individual flips if specified
                if self.flip_h:
                    cv_image = cv2.flip(cv_image, 1)
                if self.flip_v:
                    cv_image = cv2.flip(cv_image, 0)
            
            # Convert back to ROS image
            corrected_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            corrected_msg.header = msg.header  # Preserve timestamp and frame_id
            
            # Publish corrected image
            self.image_pub.publish(corrected_msg)
            # Also republish to /image_raw for backward compatibility
            self.image_raw_pub.publish(corrected_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image preprocessing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePreprocessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

