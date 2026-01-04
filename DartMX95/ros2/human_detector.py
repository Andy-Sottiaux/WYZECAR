#!/usr/bin/env python3
"""
Human Detector Node for WYZECAR Vision-Based Following System

Uses OpenCV DNN with MobileNet-SSD for stable ARM-compatible person detection.
No YOLO, no PyTorch, no crashes!

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
import urllib.request


# Model files for MobileNet-SSD (VOC trained)
MODEL_DIR = "/tmp/wyzecar_models"
PROTOTXT_FILE = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.prototxt")
CAFFEMODEL_FILE = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.caffemodel")

# Multiple mirror URLs for reliability
PROTOTXT_URLS = [
    "https://raw.githubusercontent.com/opencv/opencv_extra/master/testdata/dnn/MobileNetSSD_deploy.prototxt",
    "https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.prototxt",
]
CAFFEMODEL_URLS = [
    "https://raw.githubusercontent.com/opencv/opencv_extra/master/testdata/dnn/MobileNetSSD_deploy.caffemodel",
    "https://github.com/djmv/MobilNet_SSD_opencv/raw/master/MobileNetSSD_deploy.caffemodel",
]

# MobileNet-SSD VOC classes (person is index 15)
VOC_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"
]
PERSON_CLASS_ID = 15  # "person" in VOC


class HumanDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_box_area', 2000)
        self.declare_parameter('input_size', 300)  # MobileNet-SSD uses 300x300
        self.declare_parameter('process_every_n_frames', 2)  # Can process more often since it's lighter
        self.declare_parameter('disable_detection', False)
        
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.min_box_area = self.get_parameter('min_box_area').get_parameter_value().integer_value
        self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').get_parameter_value().integer_value
        self.disable_detection = self.get_parameter('disable_detection').get_parameter_value().bool_value
        
        if self.process_every_n_frames < 1:
            self.process_every_n_frames = 1
        
        self.bridge = CvBridge()
        self.net = None
        
        # Load model (unless disabled)
        if self.disable_detection:
            self.get_logger().warn('*** DETECTION DISABLED (passthrough mode) ***')
        else:
            self._load_model()
        
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
            self.get_logger().info('Human Detector started (OpenCV DNN + MobileNet-SSD)')
            self.get_logger().info(f'  Input size: {self.input_size}x{self.input_size}')
            self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
            self.get_logger().info(f'  Process every {self.process_every_n_frames} frames')

    def _download_file(self, urls: list, dest: str) -> bool:
        """Download a file, trying multiple URLs until one works."""
        for url in urls:
            self.get_logger().info(f'Downloading: {url}')
            try:
                urllib.request.urlretrieve(url, dest)
                self.get_logger().info(f'  -> Saved to {dest}')
                return True
            except Exception as e:
                self.get_logger().warn(f'  -> Failed: {e}')
                continue
        return False

    def _load_model(self):
        """Load MobileNet-SSD model, downloading if necessary."""
        os.makedirs(MODEL_DIR, exist_ok=True)
        
        # Download prototxt if missing
        if not os.path.exists(PROTOTXT_FILE):
            if not self._download_file(PROTOTXT_URLS, PROTOTXT_FILE):
                self.get_logger().error('Failed to download prototxt from all mirrors!')
                self.disable_detection = True
                return
        
        # Download caffemodel if missing
        if not os.path.exists(CAFFEMODEL_FILE):
            if not self._download_file(CAFFEMODEL_URLS, CAFFEMODEL_FILE):
                self.get_logger().error('Failed to download caffemodel from all mirrors!')
                self.disable_detection = True
                return
        
        # Verify files exist
        if not os.path.exists(PROTOTXT_FILE) or not os.path.exists(CAFFEMODEL_FILE):
            self.get_logger().error('Model files not found! Detection disabled.')
            self.disable_detection = True
            return
        
        # Load the network
        self.get_logger().info('Loading MobileNet-SSD model...')
        try:
            self.net = cv2.dnn.readNetFromCaffe(PROTOTXT_FILE, CAFFEMODEL_FILE)
            # Use CPU backend (most compatible on ARM)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info('MobileNet-SSD loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.disable_detection = True

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
        """Background thread that runs MobileNet-SSD on latest frame."""
        self.get_logger().info('Detection worker thread started')
        
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
            
            # Run MobileNet-SSD inference
            try:
                persons = self._detect_persons(frame)
            except Exception as e:
                self.get_logger().error(f'Detection error: {e}')
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

    def _detect_persons(self, frame: np.ndarray) -> list:
        """Run MobileNet-SSD and return list of person detections."""
        height, width = frame.shape[:2]
        
        # Create blob from image
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=0.007843,  # 1/127.5
            size=(self.input_size, self.input_size),
            mean=(127.5, 127.5, 127.5),
            swapRB=False,
            crop=False
        )
        
        # Run forward pass
        self.net.setInput(blob)
        detections = self.net.forward()
        
        persons = []
        
        # Parse detections
        # Output shape: [1, 1, N, 7] where 7 = [batch_id, class_id, confidence, x1, y1, x2, y2]
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            class_id = int(detections[0, 0, i, 1])
            
            # Only keep person detections above threshold
            if class_id != PERSON_CLASS_ID or confidence < self.conf_threshold:
                continue
            
            # Get bounding box (normalized 0-1)
            x1 = int(detections[0, 0, i, 3] * width)
            y1 = int(detections[0, 0, i, 4] * height)
            x2 = int(detections[0, 0, i, 5] * width)
            y2 = int(detections[0, 0, i, 6] * height)
            
            # Clamp to frame bounds
            x1 = max(0, min(x1, width - 1))
            y1 = max(0, min(y1, height - 1))
            x2 = max(0, min(x2, width - 1))
            y2 = max(0, min(y2, height - 1))
            
            box_area = (x2 - x1) * (y2 - y1)
            if box_area < self.min_box_area:
                continue
            
            # Calculate normalized position (-1 to 1)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            norm_x = (center_x / width) * 2 - 1
            norm_y = (center_y / height) * 2 - 1
            
            # Estimate distance (larger box = closer)
            distance = 1.0 - ((y2 - y1) / height)
            
            persons.append({
                'bbox': (x1, y1, x2, y2),
                'confidence': float(confidence),
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
        
        # Status overlay
        if self.disable_detection:
            status = "DETECTION: DISABLED (passthrough)"
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            status = f"MobileNet-SSD: {self.inference_fps:.1f} fps | Persons: {len(detections)}"
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Publish
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception:
            pass

    def log_status(self):
        """Log status every 2 seconds."""
        now = time.time()
        img_age = (now - self.last_image_time) if self.last_image_time > 0 else -1.0
        inf_age = (now - self.last_inference_time) if self.last_inference_time > 0 else -1.0

        if self.disable_detection:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | DETECTION:DISABLED | ImgAge:{img_age:.1f}s'
            )
        else:
            self.get_logger().info(
                f'[DETECTOR] Frames:{self.frame_count} | FPS:{self.inference_fps:.1f} | '
                f'Detections:{self.detection_count} | ImgAge:{img_age:.1f}s | InfAge:{inf_age:.1f}s'
            )
        
        if img_age > 2.0:
            self.get_logger().warn(f'[DETECTOR] Camera stream stale (no /image_raw for {img_age:.1f}s)')
        if not self.disable_detection and inf_age > 5.0 and self.frame_count > 0:
            self.get_logger().warn(f'[DETECTOR] Detection worker stalled (no inference for {inf_age:.1f}s)')
        
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
