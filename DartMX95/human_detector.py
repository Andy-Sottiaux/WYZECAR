#!/usr/bin/env python3
"""
Human Detector Node for WYZECAR Vision-Based Following System

Uses YOLOv8 to detect humans in camera frames and publishes the target
person's position for the follower node to track.

Topics:
    Subscribed:
        /image_raw (sensor_msgs/Image): Camera frames
    Published:
        /target_person (geometry_msgs/PointStamped): Target person position
        /detections (vision_msgs/Detection2DArray): All detections
        /debug_image (sensor_msgs/Image): Annotated debug image

Usage:
    ros2 run wyzecar_vision human_detector
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class HumanDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')
        
        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed. Install with: pip3 install ultralytics')
            return
        
        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')  # nano model for speed
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 0)  # 0 = person in COCO
        self.declare_parameter('min_box_area', 5000)  # Minimum detection size
        self.declare_parameter('publish_debug_image', True)
        
        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().integer_value
        self.min_box_area = self.get_parameter('min_box_area').get_parameter_value().integer_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {self.model_name}')
        self.model = YOLO(self.model_name)
        self.get_logger().info('YOLO model loaded successfully')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.target_pub = self.create_publisher(
            PointStamped,
            '/target_person',
            10
        )
        
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/debug_image',
                10
            )
        
        # State
        self.last_target = None
        self.frame_count = 0
        
        self.get_logger().info('Human Detector Node started')
        self.get_logger().info(f'  Model: {self.model_name}')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  Min box area: {self.min_box_area}')

    def image_callback(self, msg):
        """Process incoming camera frame"""
        self.frame_count += 1
        
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        height, width = cv_image.shape[:2]
        
        # Run YOLO detection
        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
        
        # Process detections
        detections = []
        persons = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                
                # Only process person class
                if cls != self.target_class:
                    continue
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                box_width = x2 - x1
                box_height = y2 - y1
                box_area = box_width * box_height
                
                # Filter small detections
                if box_area < self.min_box_area:
                    continue
                
                # Calculate center position (normalized -1 to 1)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                norm_x = (center_x / width) * 2 - 1  # -1 (left) to 1 (right)
                norm_y = (center_y / height) * 2 - 1  # -1 (top) to 1 (bottom)
                
                # Estimate relative distance (larger box = closer)
                # Normalized so full-height person = 0.2 (very close)
                relative_distance = 1.0 - (box_height / height)
                
                persons.append({
                    'bbox': (x1, y1, x2, y2),
                    'confidence': conf,
                    'center': (center_x, center_y),
                    'norm_pos': (norm_x, norm_y),
                    'area': box_area,
                    'distance': relative_distance
                })
                
                # Create Detection2D message
                det = Detection2D()
                det.bbox.center.position.x = center_x
                det.bbox.center.position.y = center_y
                det.bbox.size_x = float(box_width)
                det.bbox.size_y = float(box_height)
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls)
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                
                detections.append(det)
        
        # Publish all detections
        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections = detections
        self.detections_pub.publish(det_array)
        
        # Select target person (largest/closest)
        target = None
        if persons:
            # Sort by area (largest first = closest)
            persons.sort(key=lambda p: p['area'], reverse=True)
            target = persons[0]
            
            # Publish target position
            target_msg = PointStamped()
            target_msg.header = msg.header
            target_msg.point.x = target['norm_pos'][0]  # -1 to 1 (left to right)
            target_msg.point.y = target['norm_pos'][1]  # -1 to 1 (top to bottom)
            target_msg.point.z = target['distance']      # 0 to 1 (close to far)
            self.target_pub.publish(target_msg)
            
            self.last_target = target
        
        # Publish debug image
        if self.publish_debug:
            debug_image = cv_image.copy()
            
            # Draw all person detections
            for person in persons:
                x1, y1, x2, y2 = [int(v) for v in person['bbox']]
                color = (0, 255, 0) if person == target else (0, 255, 255)
                thickness = 3 if person == target else 1
                
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, thickness)
                
                label = f"Person {person['confidence']:.2f}"
                if person == target:
                    label = f"TARGET {person['confidence']:.2f}"
                
                cv2.putText(debug_image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw center crosshair
            cv2.line(debug_image, (width//2, 0), (width//2, height), (128, 128, 128), 1)
            cv2.line(debug_image, (0, height//2), (width, height//2), (128, 128, 128), 1)
            
            # Status text
            status = f"Targets: {len(persons)} | Frame: {self.frame_count}"
            cv2.putText(debug_image, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HumanDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

