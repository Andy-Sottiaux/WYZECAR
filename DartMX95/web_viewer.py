#!/usr/bin/env python3
"""
Simple Web Viewer for WYZECAR Debug Image

Opens a web server on port 8080 that shows the camera feed with
YOLO detections overlaid. Access from your browser at:
    http://<DART-IP>:8080

Usage:
    python3 web_viewer.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np

# Global frame storage
latest_frame = None
frame_lock = threading.Lock()


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress HTTP logs
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = '''
            <!DOCTYPE html>
            <html>
            <head>
                <title>WYZECAR Vision</title>
                <style>
                    body { 
                        background: #1a1a2e; 
                        color: #eee; 
                        font-family: 'Segoe UI', sans-serif;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                        padding: 20px;
                    }
                    h1 { color: #00d4ff; }
                    img { 
                        border: 3px solid #00d4ff; 
                        border-radius: 10px;
                        max-width: 100%;
                    }
                    .status { 
                        margin-top: 10px; 
                        padding: 10px 20px;
                        background: #16213e;
                        border-radius: 5px;
                    }
                </style>
            </head>
            <body>
                <h1>🚗 WYZECAR Vision Feed</h1>
                <img src="/stream" />
                <div class="status">
                    Green box = TARGET (being followed)<br>
                    Yellow box = Other detected persons
                </div>
            </body>
            </html>
            '''
            self.wfile.write(html.encode())
        
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            while True:
                try:
                    with frame_lock:
                        if latest_frame is not None:
                            frame = latest_frame.copy()
                        else:
                            # Create placeholder
                            frame = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(frame, "Waiting for camera...", (150, 240),
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
                except:
                    break


class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer')
        self.bridge = CvBridge()
        
        # Try debug_image first (has annotations), fall back to raw
        self.debug_sub = self.create_subscription(
            Image, '/debug_image', self.debug_callback, 10)
        
        self.raw_sub = self.create_subscription(
            Image, '/image_raw', self.raw_callback, 10)
        
        self.has_debug = False
        self.get_logger().info('Web Viewer started - http://0.0.0.0:8080')

    def debug_callback(self, msg):
        global latest_frame
        self.has_debug = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with frame_lock:
                latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Debug image error: {e}')

    def raw_callback(self, msg):
        global latest_frame
        if self.has_debug:
            return  # Prefer debug image if available
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Add "No detector" text
            cv2.putText(frame, "Raw feed (no detector)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            with frame_lock:
                latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Raw image error: {e}')


def main():
    rclpy.init()
    node = WebViewerNode()
    
    # Start HTTP server in background
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print("\n" + "="*50)
    print("  WYZECAR Web Viewer")
    print("  Open in browser: http://<DART-IP>:8080")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

