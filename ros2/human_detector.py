#!/usr/bin/env python3
"""
Human Detector Node for WYZECAR Vision-Based Following System

Uses YOLOv8 for person detection with threaded inference.

Topics:
    Subscribed:
        /image_raw (sensor_msgs/Image): Camera frames
    Published:
        /target_person (geometry_msgs/PointStamped): Target person position
        /debug_image (sensor_msgs/Image): Annotated debug image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class HumanDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')
        
        # Parameters - tuned for better tracking performance
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.45)  # Slightly lower for better detection
        self.declare_parameter('min_box_area', 1500)  # Detect people further away
        self.declare_parameter('input_size', 416)  # Higher resolution for better accuracy
        self.declare_parameter('process_every_n_frames', 2)  # Process more frames
        self.declare_parameter('disable_detection', False)
        
        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.min_box_area = self.get_parameter('min_box_area').get_parameter_value().integer_value
        self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').get_parameter_value().integer_value
        self.disable_detection = self.get_parameter('disable_detection').get_parameter_value().bool_value
        
        if self.process_every_n_frames < 1:
            self.process_every_n_frames = 1
        
        self.bridge = CvBridge()
        self.model = None
        
        # Load YOLO model (unless disabled)
        if self.disable_detection:
            self.get_logger().warn('*** DETECTION DISABLED (passthrough mode) ***')
        elif not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed! Running in passthrough mode.')
            self.disable_detection = True
        else:
            self.get_logger().info(f'Loading YOLO model: {self.model_name}')
            try:
                self.model = YOLO(self.model_name)
                self.get_logger().info('YOLO model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.disable_detection = True
        
        # Shared state (protected by lock)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.latest_frame_id = 0
        self.latest_detections = []
        self.inference_fps = 0.0
        
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
        self.last_inference_time = 0.0
        
        # Start detection worker thread
        self.running = True
        self.worker_thread = None
        if not self.disable_detection:
            self.worker_thread = threading.Thread(target=self.detection_worker, daemon=True)
            self.worker_thread.start()
        
        # Timer to publish debug images at steady rate
        self.create_timer(0.1, self.publish_debug)  # 10 Hz
        self.create_timer(2.0, self.log_status)
        
        if self.disable_detection:
            self.get_logger().info('Human Detector started (PASSTHROUGH MODE)')
        else:
            self.get_logger().info('Human Detector started (YOLOv8)')
            self.get_logger().info(f'  Model: {self.model_name}')
            self.get_logger().info(f'  Input size: {self.input_size}')
            self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
            self.get_logger().info(f'  Process every {self.process_every_n_frames} frames')

    def image_callback(self, msg):
        """Save the latest frame - don't process here!"""
        self.frame_count += 1
        self.last_image_time = time.time()
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = cv_image
                self.latest_header = msg.header
                self.latest_frame_id = self.frame_count
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def detection_worker(self):
        """Background thread that runs YOLO on latest frame."""
        self.get_logger().info('YOLO detection worker thread started')
        
        last_processed_id = 0
        while self.running:
            # Grab latest frame
            with self.lock:
                if self.latest_frame is None:
                    time.sleep(0.01)
                    continue
                frame_id = self.latest_frame_id
                if frame_id == last_processed_id:
                    time.sleep(0.01)
                    continue
                
                # Frame skipping
                if (frame_id % self.process_every_n_frames) != 0:
                    last_processed_id = frame_id
                    time.sleep(0.001)
                    continue
                
                frame = self.latest_frame.copy()
                header = self.latest_header
            
            start_time = time.time()
            
            # Run YOLO inference
            try:
                results = self.model(frame, verbose=False, conf=self.conf_threshold, imgsz=self.input_size)
                persons = self._parse_results(results, frame.shape[:2])
            except Exception as e:
                self.get_logger().error(f'YOLO error: {e}')
                time.sleep(0.1)
                continue
            
            # Save detections
            with self.lock:
                self.latest_detections = persons
            
            # Publish target if found
            if persons:
                target = persons[0]
                msg = PointStamped()
                msg.header = header
                msg.point.x = float(target['norm_x'])
                msg.point.y = float(target['norm_y'])
                msg.point.z = float(target['distance'])
                self.target_pub.publish(msg)
                self.detection_count += 1
            
            # Calculate FPS
            elapsed = time.time() - start_time
            self.inference_fps = 1.0 / elapsed if elapsed > 0 else 0
            self.last_inference_time = time.time()
            last_processed_id = frame_id

    def _parse_results(self, results, frame_shape) -> list:
        """Parse YOLO results and return list of person detections."""
        height, width = frame_shape
        persons = []
        
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                if cls != 0:  # 0 = person in COCO
                    continue
                
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                box_area = (x2 - x1) * (y2 - y1)
                if box_area < self.min_box_area:
                    continue
                
                # Calculate normalized position (-1 to 1)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                norm_x = (center_x / width) * 2 - 1
                norm_y = (center_y / height) * 2 - 1
                
                # Estimate distance based on bounding box height
                distance = 1.0 - ((y2 - y1) / height)
                
                persons.append({
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'confidence': conf,
                    'norm_x': norm_x,
                    'norm_y': norm_y,
                    'distance': distance,
                    'area': box_area
                })
        
        # Sort by area (largest = closest = target)
        persons.sort(key=lambda p: p['area'], reverse=True)
        
        return persons

    def publish_debug(self):
        """Publish annotated debug image at steady rate."""
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            detections = self.latest_detections.copy()
            header = self.latest_header
        
        height, width = frame.shape[:2]
        
        # Draw detections
        if not self.disable_detection:
            for i, person in enumerate(detections):
                x1, y1, x2, y2 = person['bbox']
                is_target = (i == 0)
                color = (0, 255, 0) if is_target else (0, 255, 255)
                thickness = 3 if is_target else 1
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                label = f"TARGET {person['confidence']:.2f}" if is_target else f"Person {person['confidence']:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw crosshair
        cv2.line(frame, (width//2, 0), (width//2, height), (100, 100, 100), 1)
        cv2.line(frame, (0, height//2), (width, height//2), (100, 100, 100), 1)
        
        # Note: YOLO overlay is now drawn in web_viewer.py after frame flip
        # to ensure correct orientation. Detection boxes and crosshair remain here.
        
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
        inf_age = (now - self.last_inference_time) if self.last_inference_time > 0 else -1.0

        if self.disable_detection:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | DETECTION:DISABLED | ImgAge:{img_age:.1f}s'
            )
        else:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | YOLO:{self.inference_fps:.1f}fps | '
                f'Detections:{self.detection_count} | ImgAge:{img_age:.1f}s | InfAge:{inf_age:.1f}s'
            )
        
        if img_age > 2.0:
            self.get_logger().warn(f'[DETECTOR] Camera stream stale!')
        if not self.disable_detection and inf_age > 5.0 and self.frame_count > 0:
            self.get_logger().warn(f'[DETECTOR] Detection worker stalled!')
        
        self.detection_count = 0

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HumanDetectorNode()
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
