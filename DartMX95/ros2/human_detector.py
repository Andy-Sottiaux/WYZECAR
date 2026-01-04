#!/usr/bin/env python3
"""
Human Detector Node for WYZECAR Vision-Based Following System

Uses YOLOv8 in a SEPARATE THREAD to prevent blocking the main ROS2 loop.
Camera keeps streaming, YOLO processes in background.

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
        
        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('min_box_area', 3000)
        self.declare_parameter('input_size', 256)
        # Run YOLO only every N frames to keep the pipeline responsive on ARM
        self.declare_parameter('process_every_n_frames', 5)
        # Set to True to disable YOLO entirely (passthrough mode for debugging)
        self.declare_parameter('disable_yolo', False)
        
        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.min_box_area = self.get_parameter('min_box_area').get_parameter_value().integer_value
        self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').get_parameter_value().integer_value
        self.disable_yolo = self.get_parameter('disable_yolo').get_parameter_value().bool_value
        if self.process_every_n_frames < 1:
            self.process_every_n_frames = 1
        
        # Load YOLO model (unless disabled)
        self.model = None
        if self.disable_yolo:
            self.get_logger().warn('*** YOLO DISABLED (passthrough mode) ***')
        elif not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed! Running in passthrough mode.')
            self.disable_yolo = True
        else:
            self.get_logger().info(f'Loading YOLO model: {self.model_name}')
            self.model = self._load_yolo_model_with_recovery(self.model_name)
            self.get_logger().info('YOLO model loaded successfully')
        
        self.bridge = CvBridge()
        
        # Shared state (protected by lock)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.latest_frame_id = 0
        self.latest_detections = []  # List of person detections
        self.yolo_fps = 0.0
        
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
        self.last_yolo_time = 0.0
        
        # Start YOLO worker thread (only if YOLO is enabled)
        self.running = True
        self.yolo_thread = None
        if not self.disable_yolo:
            self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
            self.yolo_thread.start()
        
        # Timer to publish debug images at steady rate (not blocking on YOLO)
        self.create_timer(0.1, self.publish_debug)  # 10 Hz
        self.create_timer(2.0, self.log_status)
        
        if self.disable_yolo:
            self.get_logger().info('Human Detector Node started (PASSTHROUGH MODE - no YOLO)')
        else:
            self.get_logger().info('Human Detector Node started (THREADED)')
            self.get_logger().info(f'  Model: {self.model_name} @ {self.input_size}px')
            self.get_logger().info(f'  Process every {self.process_every_n_frames} frames')

    def _load_yolo_model_with_recovery(self, model_name: str):
        """
        Load YOLO model, and if the weights file is corrupted (common symptom: EOFError / 'Ran out of input'),
        delete the local weights and retry once.
        """
        try:
            return YOLO(model_name)
        except Exception as e:
            msg = str(e)
            self.get_logger().error(f'YOLO model load failed: {msg}')

            # Only attempt recovery for common corrupted-file signatures
            recoverable = ('Ran out of input' in msg) or ('EOFError' in msg)
            if not recoverable:
                raise

            # If model_name is a local file, try deleting it and retry
            if os.path.exists(model_name) and os.path.isfile(model_name):
                try:
                    os.remove(model_name)
                    self.get_logger().warn(f'Deleted corrupted model file: {model_name}. Retrying download/load...')
                except Exception as rm_e:
                    self.get_logger().error(f'Failed to delete corrupted model file {model_name}: {rm_e}')
                    raise
            else:
                self.get_logger().warn('Model appears to be cached elsewhere; retrying load once anyway...')

            return YOLO(model_name)

    def image_callback(self, msg):
        """Just save the latest frame - don't process here!"""
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

    def yolo_worker(self):
        """Background thread that runs YOLO on latest frame"""
        self.get_logger().info('YOLO worker thread started')
        
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

                # Frame skipping: only process every N frames
                if (frame_id % self.process_every_n_frames) != 0:
                    last_processed_id = frame_id
                    time.sleep(0.001)
                    continue

                frame = self.latest_frame.copy()
                header = self.latest_header
            
            start_time = time.time()
            
            # Run YOLO
            try:
                results = self.model(frame, verbose=False, conf=self.conf_threshold, imgsz=self.input_size)
            except Exception as e:
                self.get_logger().error(f'YOLO error: {e}')
                time.sleep(0.1)
                continue
            
            # Process results
            height, width = frame.shape[:2]
            persons = []
            
            for result in results:
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls != 0:  # 0 = person
                        continue
                    
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    box_area = (x2 - x1) * (y2 - y1)
                    
                    if box_area < self.min_box_area:
                        continue
                    
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    norm_x = (center_x / width) * 2 - 1
                    norm_y = (center_y / height) * 2 - 1
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
            self.yolo_fps = 1.0 / elapsed if elapsed > 0 else 0
            self.last_yolo_time = time.time()
            last_processed_id = frame_id

    def publish_debug(self):
        """Publish annotated debug image at steady rate"""
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            detections = self.latest_detections.copy()
            header = self.latest_header
        
        height, width = frame.shape[:2]
        
        # Draw detections (only if YOLO is enabled)
        if not self.disable_yolo:
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
        
        # Status overlay
        if self.disable_yolo:
            status = "YOLO: DISABLED (passthrough mode)"
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            status = f"YOLO: {self.yolo_fps:.1f} fps | Persons: {len(detections)}"
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Publish
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            pass

    def log_status(self):
        """Log status every 2 seconds"""
        now = time.time()
        img_age = (now - self.last_image_time) if self.last_image_time > 0 else -1.0
        yolo_age = (now - self.last_yolo_time) if self.last_yolo_time > 0 else -1.0

        if self.disable_yolo:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | YOLO:DISABLED | ImgAge:{img_age:.1f}s'
            )
        else:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | YOLO:{self.yolo_fps:.1f}fps | '
                f'Detections:{self.detection_count} | ImgAge:{img_age:.1f}s | YoloAge:{yolo_age:.1f}s'
            )
        if img_age > 2.0:
            self.get_logger().warn(f'[DETECTOR] Camera stream stale (no /image_raw for {img_age:.1f}s)')
        if not self.disable_yolo and yolo_age > 5.0 and self.frame_count > 0:
            self.get_logger().warn(f'[DETECTOR] YOLO worker stalled (no inference for {yolo_age:.1f}s)')
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
