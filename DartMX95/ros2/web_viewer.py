#!/usr/bin/env python3
"""
Advanced Web Dashboard for WYZECAR

Real-time telemetry display:
- Target position & velocity
- Motor commands with visual bars
- Controller state machine
- System performance metrics

Access at: http://<DART-IP>:8080
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np
import time
import json

# Global state with velocity tracking
state_lock = threading.Lock()
state = {
    'frame': None,
    'frame_ts': 0.0,
    'frame_source': 'none',
    # Target tracking
    'target': None,
    'target_ts': 0.0,
    'target_vx': 0.0,
    'target_vd': 0.0,
    'prev_target': None,
    'prev_target_ts': 0.0,
    # Commands
    'cmd_vel': {'linear': 0, 'angular': 0},
    'cmd_vel_ts': 0.0,
    # Metrics
    'detection_fps': 0.0,
    'cmd_rate': 0.0,
    'frame_count': 0,
    'cmd_count': 0,
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
    <title>WYZECAR Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            background: #0a0a12;
            color: #e0e0e0;
            font-family: 'SF Mono', 'Fira Code', monospace;
            min-height: 100vh;
            padding: 15px;
        }
        .dashboard {
            display: grid;
            grid-template-columns: 1fr 320px;
            gap: 15px;
            max-width: 1400px;
            margin: 0 auto;
        }
        @media (max-width: 900px) {
            .dashboard { grid-template-columns: 1fr; }
        }
        .video-section {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        .video-container {
            background: #12121a;
            border-radius: 8px;
            overflow: hidden;
            border: 1px solid #2a2a3a;
        }
        .video-container img {
            width: 100%;
            display: block;
        }
        .state-bar {
            display: flex;
            gap: 10px;
            padding: 10px;
            background: #12121a;
            border-radius: 8px;
            border: 1px solid #2a2a3a;
        }
        .state-indicator {
            padding: 8px 16px;
            border-radius: 6px;
            font-weight: bold;
            font-size: 0.9em;
        }
        .state-indicator.active {
            background: #00ff88;
            color: #000;
        }
        .state-indicator.inactive {
            background: #1a1a2a;
            color: #444;
        }
        .telemetry {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        .panel {
            background: #12121a;
            border-radius: 8px;
            padding: 12px;
            border: 1px solid #2a2a3a;
        }
        .panel-title {
            font-size: 0.75em;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: #666;
            margin-bottom: 10px;
            padding-bottom: 6px;
            border-bottom: 1px solid #1a1a2a;
        }
        .metric {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 4px 0;
        }
        .metric-label {
            color: #888;
            font-size: 0.85em;
        }
        .metric-value {
            font-weight: bold;
            font-size: 0.95em;
        }
        .metric-value.positive { color: #00ff88; }
        .metric-value.negative { color: #ff6b6b; }
        .metric-value.neutral { color: #00d4ff; }
        .metric-value.warn { color: #ffaa00; }
        
        /* Command bars */
        .cmd-bar-container {
            margin: 8px 0;
        }
        .cmd-bar-label {
            display: flex;
            justify-content: space-between;
            font-size: 0.8em;
            margin-bottom: 4px;
        }
        .cmd-bar {
            height: 20px;
            background: #1a1a2a;
            border-radius: 4px;
            position: relative;
            overflow: hidden;
        }
        .cmd-bar-fill {
            position: absolute;
            height: 100%;
            transition: all 0.1s ease;
        }
        .cmd-bar-fill.linear {
            background: linear-gradient(90deg, #00ff88, #00cc66);
        }
        .cmd-bar-fill.angular {
            background: linear-gradient(90deg, #00d4ff, #0099cc);
        }
        .cmd-bar-center {
            position: absolute;
            left: 50%;
            top: 0;
            bottom: 0;
            width: 2px;
            background: #444;
        }
        
        /* Target visualization */
        .target-viz {
            height: 140px;
            background: #0a0a12;
            border-radius: 6px;
            position: relative;
            margin: 8px 0;
        }
        .target-grid {
            position: absolute;
            inset: 0;
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, 1fr);
        }
        .target-grid > div {
            border: 1px solid #1a1a2a;
        }
        .target-dot {
            position: absolute;
            width: 20px;
            height: 20px;
            background: #00ff88;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 15px #00ff88;
            transition: all 0.1s ease;
        }
        .target-velocity {
            position: absolute;
            width: 3px;
            background: #ffaa00;
            transform-origin: bottom center;
        }
        .target-center {
            position: absolute;
            left: 50%;
            top: 50%;
            width: 30px;
            height: 30px;
            border: 2px solid #333;
            border-radius: 50%;
            transform: translate(-50%, -50%);
        }
        .no-target {
            position: absolute;
            inset: 0;
            display: flex;
            align-items: center;
            justify-content: center;
            color: #444;
            font-size: 0.9em;
        }
        
        /* Distance zones */
        .distance-bar {
            height: 12px;
            background: linear-gradient(90deg, #ff4444 0%, #ffaa00 30%, #00ff88 60%, #00d4ff 100%);
            border-radius: 4px;
            position: relative;
            margin: 8px 0;
        }
        .distance-marker {
            position: absolute;
            top: -4px;
            width: 4px;
            height: 20px;
            background: white;
            border-radius: 2px;
            transform: translateX(-50%);
            box-shadow: 0 0 5px white;
        }
        .distance-labels {
            display: flex;
            justify-content: space-between;
            font-size: 0.7em;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="video-section">
            <div class="state-bar">
                <div id="state-idle" class="state-indicator inactive">IDLE</div>
                <div id="state-tracking" class="state-indicator inactive">TRACKING</div>
                <div id="state-stopped" class="state-indicator inactive">STOPPED</div>
                <div id="state-searching" class="state-indicator inactive">SEARCHING</div>
            </div>
            <div class="video-container">
                <img src="/stream" alt="Camera Feed" />
            </div>
        </div>
        
        <div class="telemetry">
            <div class="panel">
                <div class="panel-title">Target Position</div>
                <div class="target-viz">
                    <div class="target-grid">
                        <div></div><div></div><div></div>
                        <div></div><div></div><div></div>
                        <div></div><div></div><div></div>
                    </div>
                    <div class="target-center"></div>
                    <div id="target-dot" class="target-dot" style="display:none;"></div>
                    <div id="velocity-arrow" class="target-velocity" style="display:none;"></div>
                    <div id="no-target" class="no-target">NO TARGET</div>
                </div>
                <div class="metric">
                    <span class="metric-label">X Position</span>
                    <span id="pos-x" class="metric-value neutral">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">X Velocity</span>
                    <span id="vel-x" class="metric-value neutral">--</span>
                </div>
            </div>
            
            <div class="panel">
                <div class="panel-title">Distance</div>
                <div class="distance-bar">
                    <div id="dist-marker" class="distance-marker" style="left:50%;"></div>
                </div>
                <div class="distance-labels">
                    <span>CLOSE</span>
                    <span>STOP</span>
                    <span>TARGET</span>
                    <span>FAR</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Distance</span>
                    <span id="distance" class="metric-value neutral">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Approach Rate</span>
                    <span id="vel-d" class="metric-value neutral">--</span>
                </div>
            </div>
            
            <div class="panel">
                <div class="panel-title">Motor Commands</div>
                <div class="cmd-bar-container">
                    <div class="cmd-bar-label">
                        <span>Linear (forward/back)</span>
                        <span id="cmd-linear-val">0.00</span>
                    </div>
                    <div class="cmd-bar">
                        <div class="cmd-bar-center"></div>
                        <div id="cmd-linear-bar" class="cmd-bar-fill linear"></div>
                    </div>
                </div>
                <div class="cmd-bar-container">
                    <div class="cmd-bar-label">
                        <span>Angular (left/right)</span>
                        <span id="cmd-angular-val">0.00</span>
                    </div>
                    <div class="cmd-bar">
                        <div class="cmd-bar-center"></div>
                        <div id="cmd-angular-bar" class="cmd-bar-fill angular"></div>
                    </div>
                </div>
            </div>
            
            <div class="panel">
                <div class="panel-title">System</div>
                <div class="metric">
                    <span class="metric-label">Detection FPS</span>
                    <span id="det-fps" class="metric-value positive">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Command Rate</span>
                    <span id="cmd-rate" class="metric-value positive">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Frame Age</span>
                    <span id="frame-age" class="metric-value positive">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Target Age</span>
                    <span id="target-age" class="metric-value positive">--</span>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const STOP_DIST = 0.15;
        const TARGET_DIST = 0.35;
        
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    // State indicators
                    const states = ['idle', 'tracking', 'stopped', 'searching'];
                    let activeState = 'idle';
                    
                    if (data.target && data.target_age < 1.2) {
                        if (data.target.distance <= STOP_DIST) {
                            activeState = 'stopped';
                        } else {
                            activeState = 'tracking';
                        }
                    } else if (data.target_age < 6) {
                        activeState = 'searching';
                    }
                    
                    states.forEach(s => {
                        const el = document.getElementById('state-' + s);
                        el.className = 'state-indicator ' + (s === activeState ? 'active' : 'inactive');
                    });
                    
                    // Target visualization
                    const dot = document.getElementById('target-dot');
                    const arrow = document.getElementById('velocity-arrow');
                    const noTarget = document.getElementById('no-target');
                    const viz = document.querySelector('.target-viz');
                    
                    if (data.target) {
                        const x = (data.target.x + 1) / 2 * viz.offsetWidth;
                        const y = data.target.distance * viz.offsetHeight;
                        
                        dot.style.left = x + 'px';
                        dot.style.top = y + 'px';
                        dot.style.display = 'block';
                        noTarget.style.display = 'none';
                        
                        // Velocity arrow
                        if (Math.abs(data.vx) > 0.05) {
                            const len = Math.min(40, Math.abs(data.vx) * 30);
                            const angle = data.vx > 0 ? 90 : -90;
                            arrow.style.left = x + 'px';
                            arrow.style.top = y + 'px';
                            arrow.style.height = len + 'px';
                            arrow.style.transform = 'rotate(' + angle + 'deg)';
                            arrow.style.display = 'block';
                        } else {
                            arrow.style.display = 'none';
                        }
                        
                        document.getElementById('pos-x').textContent = data.target.x.toFixed(2);
                        document.getElementById('vel-x').textContent = (data.vx >= 0 ? '+' : '') + data.vx.toFixed(2);
                        document.getElementById('distance').textContent = data.target.distance.toFixed(2);
                        document.getElementById('vel-d').textContent = (data.vd >= 0 ? '+' : '') + data.vd.toFixed(2);
                        
                        // Distance marker
                        document.getElementById('dist-marker').style.left = (data.target.distance * 100) + '%';
                    } else {
                        dot.style.display = 'none';
                        arrow.style.display = 'none';
                        noTarget.style.display = 'flex';
                        document.getElementById('pos-x').textContent = '--';
                        document.getElementById('vel-x').textContent = '--';
                        document.getElementById('distance').textContent = '--';
                        document.getElementById('vel-d').textContent = '--';
                    }
                    
                    // Motor command bars
                    const linearBar = document.getElementById('cmd-linear-bar');
                    const angularBar = document.getElementById('cmd-angular-bar');
                    const linear = data.cmd_vel ? data.cmd_vel.linear : 0;
                    const angular = data.cmd_vel ? data.cmd_vel.angular : 0;
                    
                    document.getElementById('cmd-linear-val').textContent = linear.toFixed(2);
                    document.getElementById('cmd-angular-val').textContent = angular.toFixed(2);
                    
                    // Linear bar (center = 0, left = back, right = forward)
                    const linearPct = Math.abs(linear) / 0.7 * 50;
                    if (linear >= 0) {
                        linearBar.style.left = '50%';
                        linearBar.style.width = linearPct + '%';
                    } else {
                        linearBar.style.left = (50 - linearPct) + '%';
                        linearBar.style.width = linearPct + '%';
                    }
                    
                    // Angular bar (center = 0, left = left turn, right = right turn)
                    const angularPct = Math.abs(angular) / 1.0 * 50;
                    if (angular >= 0) {
                        angularBar.style.left = '50%';
                        angularBar.style.width = angularPct + '%';
                    } else {
                        angularBar.style.left = (50 - angularPct) + '%';
                        angularBar.style.width = angularPct + '%';
                    }
                    
                    // System metrics
                    document.getElementById('det-fps').textContent = data.detection_fps.toFixed(1) + ' Hz';
                    document.getElementById('cmd-rate').textContent = data.cmd_rate.toFixed(1) + ' Hz';
                    document.getElementById('frame-age').textContent = data.frame_age.toFixed(2) + 's';
                    document.getElementById('target-age').textContent = data.target_age.toFixed(2) + 's';
                    
                    // Color code ages
                    const frameAgeEl = document.getElementById('frame-age');
                    frameAgeEl.className = 'metric-value ' + (data.frame_age > 1 ? 'negative' : 'positive');
                    
                    const targetAgeEl = document.getElementById('target-age');
                    targetAgeEl.className = 'metric-value ' + (data.target_age > 1.2 ? 'warn' : 'positive');
                })
                .catch(e => console.error(e));
        }
        
        setInterval(updateStatus, 100);
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
                
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
                time.sleep(0.08)
            except Exception:
                break

    def _get_status_json(self):
        with state_lock:
            now = time.time()
            return json.dumps({
                'target': state['target'],
                'target_age': now - state['target_ts'] if state['target_ts'] > 0 else 999,
                'vx': state['target_vx'],
                'vd': state['target_vd'],
                'cmd_vel': state['cmd_vel'],
                'detection_fps': state['detection_fps'],
                'cmd_rate': state['cmd_rate'],
                'frame_age': now - state['frame_ts'] if state['frame_ts'] > 0 else 999,
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
        
        self.get_logger().info('Subscribed to: /debug_image, /image_raw, /target_person, /cmd_vel')
        
        self.has_debug = False
        self.frame_times = []
        self.cmd_times = []
        
        self.get_logger().info('Advanced Web Dashboard - http://0.0.0.0:8080')
        self.create_timer(0.5, self.update_metrics)

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
        now = time.time()
        self.get_logger().info(f'TARGET: x={msg.point.x:.2f} d={msg.point.z:.2f}')
        with state_lock:
            # Calculate velocities from previous target
            if state['target'] is not None and state['target_ts'] > 0:
                dt = now - state['target_ts']
                if dt > 0.01 and dt < 0.5:
                    raw_vx = (msg.point.x - state['target']['x']) / dt
                    raw_vd = (msg.point.z - state['target']['distance']) / dt
                    # Smooth velocities
                    state['target_vx'] = state['target_vx'] * 0.6 + raw_vx * 0.4
                    state['target_vd'] = state['target_vd'] * 0.6 + raw_vd * 0.4
            
            state['target'] = {
                'x': msg.point.x,
                'y': msg.point.y,
                'distance': msg.point.z
            }
            state['target_ts'] = now

    def cmd_vel_callback(self, msg):
        with state_lock:
            state['cmd_vel'] = {
                'linear': msg.linear.x,
                'angular': msg.angular.z
            }
            state['cmd_vel_ts'] = time.time()
            state['cmd_count'] += 1
            self.cmd_times.append(time.time())

    def update_metrics(self):
        now = time.time()
        with state_lock:
            target_age = now - state['target_ts'] if state['target_ts'] > 0 else 999
            
            # Log subscription status periodically
            self.get_logger().info(
                f'[STATUS] frames={state["frame_count"]} cmds={state["cmd_count"]} '
                f'target_age={target_age:.1f}s'
            )
            
            # Clear stale data
            if target_age > 1.5:
                state['target'] = None
                state['target_vx'] = 0.0
                state['target_vd'] = 0.0
            
            if now - state['cmd_vel_ts'] > 1.0:
                state['cmd_vel'] = {'linear': 0, 'angular': 0}
            
            # Calculate rates
            self.frame_times = [t for t in self.frame_times if now - t < 2.0]
            self.cmd_times = [t for t in self.cmd_times if now - t < 2.0]
            
            state['detection_fps'] = len(self.frame_times) / 2.0 if self.frame_times else 0
            state['cmd_rate'] = len(self.cmd_times) / 2.0 if self.cmd_times else 0


def main():
    rclpy.init()
    node = WebViewerNode()
    
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print("\n" + "="*50)
    print("  WYZECAR Advanced Dashboard")
    print("  http://<DART-IP>:8080")
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
