#!/usr/bin/env python3
"""
Enhanced Web Viewer for WYZECAR

Features:
- Live video stream with YOLO detections
- Real-time status panel (target, motors, FPS)
- Auto-refreshing telemetry via JSON API

Access at: http://<DART-IP>:8080
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np
import time
import json

# Global state
state_lock = threading.Lock()
state = {
    'frame': None,
    'frame_ts': 0.0,
    'frame_source': 'none',
    'target': None,  # {'x': float, 'y': float, 'distance': float}
    'target_ts': 0.0,
    'cmd_vel': None,  # {'linear': float, 'angular': float}
    'cmd_vel_ts': 0.0,
    'detection_fps': 0.0,
    'frame_count': 0,
}


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(self._get_html().encode())
        
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            self._stream_video()
        
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(self._get_status_json().encode())
        
        else:
            self.send_error(404)

    def _get_html(self):
        return '''<!DOCTYPE html>
<html>
<head>
    <title>WYZECAR Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            background: linear-gradient(135deg, #0f0f1a 0%, #1a1a2e 100%);
            color: #e0e0e0;
            font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, sans-serif;
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        header {
            text-align: center;
            margin-bottom: 20px;
        }
        h1 {
            font-size: 2em;
            font-weight: 300;
            color: #00d4ff;
            text-shadow: 0 0 20px rgba(0,212,255,0.3);
        }
        .main {
            display: grid;
            grid-template-columns: 1fr 300px;
            gap: 20px;
        }
        @media (max-width: 800px) {
            .main { grid-template-columns: 1fr; }
        }
        .video-panel {
            background: #16213e;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .video-panel img {
            width: 100%;
            display: block;
        }
        .status-panel {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .card {
            background: #16213e;
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .card h3 {
            font-size: 0.85em;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #888;
            margin-bottom: 10px;
            border-bottom: 1px solid #2a2a4a;
            padding-bottom: 8px;
        }
        .stat {
            display: flex;
            justify-content: space-between;
            padding: 6px 0;
            border-bottom: 1px solid #1a1a3a;
        }
        .stat:last-child { border-bottom: none; }
        .stat-label { color: #888; }
        .stat-value { 
            font-family: 'SF Mono', monospace;
            font-weight: 500;
        }
        .stat-value.good { color: #00ff88; }
        .stat-value.warn { color: #ffaa00; }
        .stat-value.bad { color: #ff4444; }
        .stat-value.neutral { color: #00d4ff; }
        .target-viz {
            height: 120px;
            background: #0a0a15;
            border-radius: 8px;
            position: relative;
            overflow: hidden;
        }
        .target-dot {
            width: 16px;
            height: 16px;
            background: #00ff88;
            border-radius: 50%;
            position: absolute;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 10px #00ff88;
            transition: all 0.1s ease;
        }
        .target-crosshair {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            width: 100%;
            height: 100%;
        }
        .target-crosshair::before,
        .target-crosshair::after {
            content: '';
            position: absolute;
            background: #333;
        }
        .target-crosshair::before {
            width: 1px;
            height: 100%;
            left: 50%;
        }
        .target-crosshair::after {
            width: 100%;
            height: 1px;
            top: 50%;
        }
        .no-target {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: #666;
            font-size: 0.9em;
        }
        .legend {
            margin-top: 15px;
            padding: 10px;
            background: #0a0a15;
            border-radius: 8px;
            font-size: 0.85em;
        }
        .legend-item {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 4px 0;
        }
        .legend-color {
            width: 12px;
            height: 12px;
            border-radius: 3px;
        }
        .legend-color.green { background: #00ff88; }
        .legend-color.yellow { background: #ffaa00; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🚗 WYZECAR Control</h1>
        </header>
        <div class="main">
            <div class="video-panel">
                <img src="/stream" alt="Camera Feed" />
            </div>
            <div class="status-panel">
                <div class="card">
                    <h3>Target Position</h3>
                    <div class="target-viz">
                        <div class="target-crosshair"></div>
                        <div id="target-dot" class="target-dot" style="display:none;"></div>
                        <div id="no-target" class="no-target">No target</div>
                    </div>
                    <div class="stat">
                        <span class="stat-label">X Position</span>
                        <span id="target-x" class="stat-value neutral">--</span>
                    </div>
                    <div class="stat">
                        <span class="stat-label">Distance</span>
                        <span id="target-dist" class="stat-value neutral">--</span>
                    </div>
                </div>
                <div class="card">
                    <h3>Motor Commands</h3>
                    <div class="stat">
                        <span class="stat-label">Linear</span>
                        <span id="cmd-linear" class="stat-value neutral">0.00</span>
                    </div>
                    <div class="stat">
                        <span class="stat-label">Angular</span>
                        <span id="cmd-angular" class="stat-value neutral">0.00</span>
                    </div>
                </div>
                <div class="card">
                    <h3>System Status</h3>
                    <div class="stat">
                        <span class="stat-label">Detection FPS</span>
                        <span id="det-fps" class="stat-value good">--</span>
                    </div>
                    <div class="stat">
                        <span class="stat-label">Frame Age</span>
                        <span id="frame-age" class="stat-value good">--</span>
                    </div>
                    <div class="stat">
                        <span class="stat-label">Status</span>
                        <span id="status" class="stat-value good">OK</span>
                    </div>
                </div>
                <div class="legend">
                    <div class="legend-item">
                        <div class="legend-color green"></div>
                        <span>TARGET (being followed)</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color yellow"></div>
                        <span>Other detected persons</span>
                    </div>
                </div>
            </div>
        </div>
    </div>
    <script>
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    // Target
                    const dot = document.getElementById('target-dot');
                    const noTarget = document.getElementById('no-target');
                    if (data.target) {
                        const viz = document.querySelector('.target-viz');
                        const x = (data.target.x + 1) / 2 * viz.offsetWidth;
                        const y = (data.target.distance) * viz.offsetHeight;
                        dot.style.left = x + 'px';
                        dot.style.top = y + 'px';
                        dot.style.display = 'block';
                        noTarget.style.display = 'none';
                        document.getElementById('target-x').textContent = data.target.x.toFixed(2);
                        document.getElementById('target-dist').textContent = data.target.distance.toFixed(2);
                    } else {
                        dot.style.display = 'none';
                        noTarget.style.display = 'block';
                        document.getElementById('target-x').textContent = '--';
                        document.getElementById('target-dist').textContent = '--';
                    }
                    
                    // Motor commands
                    if (data.cmd_vel) {
                        document.getElementById('cmd-linear').textContent = data.cmd_vel.linear.toFixed(2);
                        document.getElementById('cmd-angular').textContent = data.cmd_vel.angular.toFixed(2);
                    }
                    
                    // System status
                    document.getElementById('det-fps').textContent = data.detection_fps.toFixed(1);
                    document.getElementById('frame-age').textContent = data.frame_age.toFixed(1) + 's';
                    
                    const statusEl = document.getElementById('status');
                    if (data.frame_age > 2) {
                        statusEl.textContent = 'STALE';
                        statusEl.className = 'stat-value bad';
                    } else if (data.target) {
                        statusEl.textContent = 'TRACKING';
                        statusEl.className = 'stat-value good';
                    } else {
                        statusEl.textContent = 'SEARCHING';
                        statusEl.className = 'stat-value warn';
                    }
                })
                .catch(e => {
                    document.getElementById('status').textContent = 'ERROR';
                    document.getElementById('status').className = 'stat-value bad';
                });
        }
        setInterval(updateStatus, 200);
        updateStatus();
    </script>
</body>
</html>'''

    def _stream_video(self):
        while True:
            try:
                with state_lock:
                    if state['frame'] is not None:
                        frame = state['frame'].copy()
                    else:
                        frame = np.zeros((240, 320, 3), dtype=np.uint8)
                        cv2.putText(frame, "Waiting for camera...", (40, 120),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
                time.sleep(0.1)
            except Exception:
                break

    def _get_status_json(self):
        with state_lock:
            now = time.time()
            return json.dumps({
                'target': state['target'],
                'target_age': now - state['target_ts'] if state['target_ts'] > 0 else 999,
                'cmd_vel': state['cmd_vel'],
                'detection_fps': state['detection_fps'],
                'frame_age': now - state['frame_ts'] if state['frame_ts'] > 0 else 999,
                'frame_count': state['frame_count'],
            })


class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.debug_sub = self.create_subscription(
            Image, '/debug_image', self.debug_callback, 10)
        self.raw_sub = self.create_subscription(
            Image, '/image_raw', self.raw_callback, 10)
        self.target_sub = self.create_subscription(
            PointStamped, '/target_person', self.target_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.has_debug = False
        self.frame_times = []
        
        self.get_logger().info('Enhanced Web Viewer started - http://0.0.0.0:8080')
        self.create_timer(1.0, self.update_fps)

    def debug_callback(self, msg):
        self.has_debug = True
        self._update_frame(msg, 'debug_image')

    def raw_callback(self, msg):
        if self.has_debug:
            return
        self._update_frame(msg, 'image_raw')

    def _update_frame(self, msg, source):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with state_lock:
                state['frame'] = frame
                state['frame_ts'] = time.time()
                state['frame_source'] = source
                state['frame_count'] += 1
                self.frame_times.append(time.time())
        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

    def target_callback(self, msg):
        with state_lock:
            state['target'] = {
                'x': msg.point.x,
                'y': msg.point.y,
                'distance': msg.point.z
            }
            state['target_ts'] = time.time()

    def cmd_vel_callback(self, msg):
        with state_lock:
            state['cmd_vel'] = {
                'linear': msg.linear.x,
                'angular': msg.angular.z
            }
            state['cmd_vel_ts'] = time.time()

    def update_fps(self):
        now = time.time()
        # Clear target if stale
        with state_lock:
            if now - state['target_ts'] > 1.0:
                state['target'] = None
            if now - state['cmd_vel_ts'] > 1.0:
                state['cmd_vel'] = {'linear': 0, 'angular': 0}
            
            # Calculate FPS
            self.frame_times = [t for t in self.frame_times if now - t < 2.0]
            if len(self.frame_times) > 1:
                state['detection_fps'] = len(self.frame_times) / 2.0
            else:
                state['detection_fps'] = 0.0


def main():
    rclpy.init()
    node = WebViewerNode()
    
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print("\n" + "="*50)
    print("  WYZECAR Enhanced Web Viewer")
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
