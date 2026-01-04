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
import time

# Global frame storage
latest_frame = None
frame_lock = threading.Lock()
latest_frame_ts = 0.0
latest_frame_source = "none"  # debug_image | image_raw | none


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
                    now = time.time()
                    with frame_lock:
                        if latest_frame is not None:
                            frame = latest_frame.copy()
                            age_s = now - latest_frame_ts if latest_frame_ts > 0 else 0.0
                            source = latest_frame_source
                        else:
                            # Create placeholder
                            frame = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(frame, "Waiting for camera...", (150, 240),
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                            age_s = 0.0
                            source = "none"

                    # If frames are stale, overlay a warning (helps diagnose freezes)
                    if latest_frame is not None and age_s > 1.0:
                        cv2.putText(
                            frame,
                            f"STALE FRAME ({age_s:.1f}s) source={source}",
                            (10, frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 0, 255),
                            2,
                        )
                    
                    # Lower quality and add sleep to reduce CPU load
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
                    time.sleep(0.1)  # 10 FPS max to reduce load
                except Exception as e:
                    # Client disconnected or encoding error.
                    # Don't crash the server; just end this stream connection.
                    try:
                        self.server._wyzecar_last_stream_error = str(e)
                    except Exception:
                        pass
                    break


class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer')
        self.bridge = CvBridge()
        self.last_log_ts = 0.0
        
        # Try debug_image first (has annotations), fall back to raw
        self.debug_sub = self.create_subscription(
            Image, '/debug_image', self.debug_callback, 10)
        
        self.raw_sub = self.create_subscription(
            Image, '/image_raw', self.raw_callback, 10)
        
        self.has_debug = False
        self.get_logger().info('Web Viewer started - http://0.0.0.0:8080')
        self.create_timer(2.0, self.log_status)

    def debug_callback(self, msg):
        global latest_frame, latest_frame_ts, latest_frame_source
        self.has_debug = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with frame_lock:
                latest_frame = frame
                latest_frame_ts = time.time()
                latest_frame_source = "debug_image"
        except Exception as e:
            self.get_logger().error(f'Debug image error: {e}')

    def raw_callback(self, msg):
        global latest_frame, latest_frame_ts, latest_frame_source
        if self.has_debug:
            return  # Prefer debug image if available
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Add "No detector" text
            cv2.putText(frame, "Raw feed (no detector)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            with frame_lock:
                latest_frame = frame
                latest_frame_ts = time.time()
                latest_frame_source = "image_raw"
        except Exception as e:
            self.get_logger().error(f'Raw image error: {e}')

    def log_status(self):
        global latest_frame_ts, latest_frame_source
        now = time.time()
        with frame_lock:
            ts = latest_frame_ts
            src = latest_frame_source
            has = latest_frame is not None

        if not has:
            self.get_logger().warn('[WEB] No frames received yet')
            return

        age = now - ts if ts > 0 else 0.0
        if age > 2.0:
            self.get_logger().warn(f'[WEB] Frame stream stale: age={age:.1f}s source={src}')
        else:
            self.get_logger().info(f'[WEB] OK: age={age:.1f}s source={src}')


def main():
    rclpy.init()
    node = WebViewerNode()
    
    # Start HTTP server in background
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    server._wyzecar_last_stream_error = ""
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

