#!/usr/bin/env python3
"""
WYZECAR Remote Control Dashboard

Features:
- WASD keyboard control for manual driving
- Live video stream with high FPS
- Human detection visualization (optional)
- Real-time telemetry display

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
from http.server import BaseHTTPRequestHandler
import numpy as np
import time
import json

try:
    # Python 3.7+
    from http.server import ThreadingHTTPServer as _ThreadingHTTPServer
except Exception:  # pragma: no cover
    _ThreadingHTTPServer = None

if _ThreadingHTTPServer is None:  # pragma: no cover
    import socketserver

    class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        daemon_threads = True
        allow_reuse_address = True
else:

    class ThreadingHTTPServer(_ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

# Global state
state_lock = threading.Lock()
state = {
    'frame': None,
    'frame_ts': 0.0,
    'frame_source': 'none',
    # Target tracking (from human detector)
    'target': None,
    'target_ts': 0.0,
    'target_vx': 0.0,
    'target_vd': 0.0,
    # Commands (from keyboard)
    'cmd_vel': {'linear': 0, 'angular': 0},
    'cmd_vel_ts': 0.0,
    # Keyboard control state
    'keys': {'w': False, 's': False, 'a': False, 'd': False},
    'manual_control': True,  # Manual mode by default
    # Metrics
    'detection_fps': 0.0,
    'cmd_rate': 0.0,
    'frame_count': 0,
    'cmd_count': 0,
}

# Global reference to ROS node for publishing
ros_node = None


class MJPEGHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        pass
    
    def do_GET(self):
        if self.path == '/':
            html = self._get_html().encode('utf-8')
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Private-Network', 'true')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.send_header('Content-Length', str(len(html)))
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(html)
            self.close_connection = True
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Private-Network', 'true')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            self.close_connection = False
            self._stream_video()
        elif self.path == '/status':
            try:
                json_data = self._get_status_json()
                resp = json_data.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Expires', '0')
                self.send_header('Content-Length', str(len(resp)))
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(resp)
                self.close_connection = True
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'text/plain')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(f'Error: {e}'.encode())
        else:
            self.send_error(404)
    
    def do_POST(self):
        """Handle keyboard control commands."""
        if self.path == '/cmd':
            try:
                content_length = int(self.headers['Content-Length'])
                body = self.rfile.read(content_length).decode('utf-8')
                data = json.loads(body)
                
                # Update key states
                with state_lock:
                    if 'keys' in data:
                        state['keys'] = data['keys']
                    
                    # Calculate velocity from keys
                    linear = 0.0
                    angular = 0.0
                    
                    if state['keys'].get('w'):
                        linear = 1.0  # Forward (full speed - matches max_linear_speed)
                    elif state['keys'].get('s'):
                        linear = -1.0  # Backward (full speed - matches max_linear_speed)
                    
                    if state['keys'].get('a'):
                        angular = -1.0  # Turn left (full deflection)
                    elif state['keys'].get('d'):
                        angular = 1.0  # Turn right (full deflection)
                    
                    state['cmd_vel'] = {'linear': linear, 'angular': angular}
                    state['cmd_vel_ts'] = time.time()
                
                # Publish to ROS
                if ros_node:
                    ros_node.publish_cmd_vel(linear, angular)
                
                resp = b'{"ok":true}'
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Private-Network', 'true')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Expires', '0')
                self.send_header('Content-Length', str(len(resp)))
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(resp)
            except Exception as e:
                resp = json.dumps({"error": str(e)}).encode("utf-8")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Content-Length', str(len(resp)))
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(resp)
        else:
            self.send_error(404)
    
    def do_HEAD(self):
        """Handle HEAD requests (browser preflight/checks)."""
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Private-Network', 'true')
            self.send_header('Connection', 'close')
            self.end_headers()
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Private-Network', 'true')
            self.send_header('Connection', 'close')
            self.end_headers()
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Private-Network', 'true')
            self.send_header('Connection', 'close')
            self.end_headers()
        else:
            self.send_error(404)
    
    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Private-Network', 'true')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, HEAD, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Length', '0')
        self.send_header('Connection', 'close')
        self.end_headers()

    def _get_html(self):
        return '''<!DOCTYPE html>
<html>
<head>
    <title>WYZECAR Remote Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            background: linear-gradient(135deg, #0a0a12 0%, #1a1a2e 100%);
            color: #e0e0e0;
            font-family: 'JetBrains Mono', 'SF Mono', monospace;
            min-height: 100vh;
            padding: 20px;
        }
        .header {
            text-align: center;
            margin-bottom: 20px;
        }
        .header h1 {
            font-size: 1.8em;
            background: linear-gradient(90deg, #00ff88, #00d4ff);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 5px;
        }
        .header .status {
            font-size: 0.9em;
            color: #666;
        }
        .header .status.connected { color: #00ff88; }
        .dashboard {
            display: grid;
            grid-template-columns: 1fr 280px;
            gap: 20px;
            max-width: 1200px;
            margin: 0 auto;
        }
        @media (max-width: 900px) {
            .dashboard { grid-template-columns: 1fr; }
        }
        .video-section {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .video-container {
            background: #000;
            border-radius: 12px;
            overflow: hidden;
            border: 2px solid #2a2a4a;
            box-shadow: 0 10px 40px rgba(0,0,0,0.5);
        }
        .video-container img {
            width: 100%;
            display: block;
        }
        
        /* WASD Control Panel */
        .controls {
            background: #12121a;
            border-radius: 12px;
            padding: 20px;
            border: 1px solid #2a2a4a;
            text-align: center;
        }
        .controls-title {
            font-size: 0.8em;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: #666;
            margin-bottom: 15px;
        }
        .wasd-grid {
            display: grid;
            grid-template-columns: repeat(3, 60px);
            grid-template-rows: repeat(2, 60px);
            gap: 8px;
            justify-content: center;
            margin-bottom: 15px;
        }
        .key {
            width: 60px;
            height: 60px;
            background: #1a1a2a;
            border: 2px solid #3a3a5a;
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 1.5em;
            font-weight: bold;
            color: #888;
            transition: all 0.1s ease;
            user-select: none;
        }
        .key.active {
            background: linear-gradient(135deg, #00ff88 0%, #00cc66 100%);
            border-color: #00ff88;
            color: #000;
            box-shadow: 0 0 20px rgba(0,255,136,0.5);
            transform: scale(0.95);
        }
        .key.empty { visibility: hidden; }
        .speed-display {
            font-size: 1.2em;
            margin-top: 10px;
        }
        .speed-display span { color: #00ff88; }
        
        /* Telemetry Panel */
        .telemetry {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .panel {
            background: #12121a;
            border-radius: 12px;
            padding: 15px;
            border: 1px solid #2a2a4a;
        }
        .panel-title {
            font-size: 0.75em;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: #666;
            margin-bottom: 12px;
            padding-bottom: 8px;
            border-bottom: 1px solid #2a2a4a;
        }
        .metric {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 6px 0;
        }
        .metric-label { color: #888; font-size: 0.85em; }
        .metric-value { font-weight: bold; }
        .metric-value.positive { color: #00ff88; }
        .metric-value.warn { color: #ffaa00; }
        
        /* Command bars */
        .cmd-bar-container { margin: 10px 0; }
        .cmd-bar-label {
            display: flex;
            justify-content: space-between;
            font-size: 0.8em;
            margin-bottom: 4px;
        }
        .cmd-bar {
            height: 24px;
            background: #1a1a2a;
            border-radius: 6px;
            position: relative;
            overflow: hidden;
        }
        .cmd-bar-fill {
            position: absolute;
            height: 100%;
            transition: all 0.05s ease;
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
        
        /* Instructions */
        .instructions {
            text-align: center;
            color: #666;
            font-size: 0.85em;
            padding: 10px;
        }
        .instructions kbd {
            background: #2a2a4a;
            padding: 4px 8px;
            border-radius: 4px;
            margin: 0 2px;
        }
    </style>
</head>
<body tabindex="0">
    <div class="header">
        <h1>WYZECAR</h1>
        <div id="conn-status" class="status">Connecting...</div>
    </div>
    
    <div class="dashboard">
        <div class="video-section">
            <div class="video-container">
                <img src="/stream" alt="Camera Feed" />
            </div>
            <div class="controls">
                <div class="controls-title">Keyboard Control</div>
                <div class="wasd-grid">
                    <div class="key empty"></div>
                    <div class="key" id="key-w">W</div>
                    <div class="key empty"></div>
                    <div class="key" id="key-a">A</div>
                    <div class="key" id="key-s">S</div>
                    <div class="key" id="key-d">D</div>
                </div>
                <div class="speed-display">
                    Speed: <span id="speed-val">0%</span> | Turn: <span id="turn-val">0°</span>
                </div>
            </div>
            <div class="instructions" id="kb-hint">
                Click anywhere on this page to enable driving, then use <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>
            </div>
        </div>
        
        <div class="telemetry">
            <div class="panel">
                <div class="panel-title">Motor Output</div>
                <div class="cmd-bar-container">
                    <div class="cmd-bar-label">
                        <span>Forward / Back</span>
                        <span id="cmd-linear-val">0.00</span>
                    </div>
                    <div class="cmd-bar">
                        <div class="cmd-bar-center"></div>
                        <div id="cmd-linear-bar" class="cmd-bar-fill linear"></div>
                    </div>
                </div>
                <div class="cmd-bar-container">
                    <div class="cmd-bar-label">
                        <span>Left / Right</span>
                        <span id="cmd-angular-val">0.00</span>
                    </div>
                    <div class="cmd-bar">
                        <div class="cmd-bar-center"></div>
                        <div id="cmd-angular-bar" class="cmd-bar-fill angular"></div>
                    </div>
                </div>
            </div>
            
            <div class="panel">
                <div class="panel-title">Human Detection</div>
                <div class="metric">
                    <span class="metric-label">Target</span>
                    <span id="target-status" class="metric-value warn">None</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Distance</span>
                    <span id="target-dist" class="metric-value">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Position X</span>
                    <span id="target-x" class="metric-value">--</span>
                </div>
            </div>
            
            <div class="panel">
                <div class="panel-title">System</div>
                <div class="metric">
                    <span class="metric-label">Video FPS</span>
                    <span id="det-fps" class="metric-value positive">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Cmd Rate</span>
                    <span id="cmd-rate" class="metric-value positive">--</span>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // Keyboard state
        const keys = { w: false, s: false, a: false, d: false };
        let connected = false;
        let hasFocus = false;
        let lastSendMs = 0;
        const SEND_MIN_INTERVAL_MS = 20; // 50Hz max for lower latency
        
        // Send commands to server
        function sendCommand() {
            const now = performance.now();
            if (now - lastSendMs < SEND_MIN_INTERVAL_MS) return;
            lastSendMs = now;
            fetch('/cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ keys })
            }).then(() => {
                connected = true;
                document.getElementById('conn-status').textContent = 'Connected';
                document.getElementById('conn-status').className = 'status connected';
            }).catch(() => {
                connected = false;
                document.getElementById('conn-status').textContent = 'Disconnected';
                document.getElementById('conn-status').className = 'status';
            });
        }
        
        // Key handlers
        function setHint(text) {
            const el = document.getElementById('kb-hint');
            if (el) el.textContent = text;
        }

        function focusForDriving() {
            // Chrome often needs a focused element for consistent key events.
            document.body.focus();
        }

        function keyToControl(k) {
            if (!k) return null;
            const key = k.toLowerCase();
            if (!Object.prototype.hasOwnProperty.call(keys, key)) return null;
            return key;
        }

        document.addEventListener('keydown', (e) => {
            // Don’t steal keys while typing somewhere else (future-proofing).
            const tag = (e.target && e.target.tagName) ? e.target.tagName.toLowerCase() : '';
            if (tag === 'input' || tag === 'textarea' || tag === 'select') return;

            const key = keyToControl(e.key);
            if (!key) return;

            // Prevent scroll/quickfind behaviors in some browsers.
            e.preventDefault();

            if (!keys[key]) {
                keys[key] = true;
                document.getElementById('key-' + key).classList.add('active');
                sendCommand();
                updateSpeedDisplay();
            }
        }, { passive: false });
        
        document.addEventListener('keyup', (e) => {
            const key = keyToControl(e.key);
            if (!key) return;
            e.preventDefault();
            keys[key] = false;
            document.getElementById('key-' + key).classList.remove('active');
            sendCommand();
            updateSpeedDisplay();
        }, { passive: false });

        // Click-to-focus (important for Chrome)
        document.addEventListener('pointerdown', () => {
            focusForDriving();
        });

        window.addEventListener('focus', () => {
            hasFocus = true;
            setHint('Driving enabled — use W A S D');
        });

        window.addEventListener('blur', () => {
            hasFocus = false;
            setHint('Click anywhere on this page to enable driving, then use W A S D');
        });
        
        // Lose all keys on blur
        window.addEventListener('blur', () => {
            Object.keys(keys).forEach(k => {
                keys[k] = false;
                document.getElementById('key-' + k).classList.remove('active');
            });
            sendCommand();
            updateSpeedDisplay();
        });
        
        function updateSpeedDisplay() {
            let speed = 0, turn = 0;
            if (keys.w) speed = 100;
            else if (keys.s) speed = -100;
            if (keys.a) turn = -45;
            else if (keys.d) turn = 45;
            document.getElementById('speed-val').textContent = speed + '%';
            document.getElementById('turn-val').textContent = turn + '°';
        }
        
        // Status polling
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    // Motor bars
                    const linear = data.cmd_vel ? data.cmd_vel.linear : 0;
                    const angular = data.cmd_vel ? data.cmd_vel.angular : 0;
                    
                    document.getElementById('cmd-linear-val').textContent = linear.toFixed(2);
                    document.getElementById('cmd-angular-val').textContent = angular.toFixed(2);
                    
                    const linearBar = document.getElementById('cmd-linear-bar');
                    const angularBar = document.getElementById('cmd-angular-bar');
                    
                    const linearPct = Math.abs(linear) / 1.0 * 50;
                    linearBar.style.left = linear >= 0 ? '50%' : (50 - linearPct) + '%';
                    linearBar.style.width = linearPct + '%';
                    
                    const angularPct = Math.abs(angular) / 1.0 * 50;
                    angularBar.style.left = angular >= 0 ? '50%' : (50 - angularPct) + '%';
                    angularBar.style.width = angularPct + '%';
                    
                    // Human detection
                    if (data.target && data.target_age < 2) {
                        document.getElementById('target-status').textContent = 'Detected';
                        document.getElementById('target-status').className = 'metric-value positive';
                        document.getElementById('target-dist').textContent = data.target.distance.toFixed(2) + 'm';
                        document.getElementById('target-x').textContent = data.target.x.toFixed(2);
                    } else {
                        document.getElementById('target-status').textContent = 'None';
                        document.getElementById('target-status').className = 'metric-value warn';
                        document.getElementById('target-dist').textContent = '--';
                        document.getElementById('target-x').textContent = '--';
                    }
                    
                    // System
                    document.getElementById('det-fps').textContent = data.detection_fps.toFixed(1) + ' Hz';
                    document.getElementById('cmd-rate').textContent = data.cmd_rate.toFixed(1) + ' Hz';
                })
                .catch(console.error);
        }
        
        setInterval(updateStatus, 50);  // 20Hz status updates
        setInterval(sendCommand, 50);  // Keep-alive commands (20Hz)
        updateStatus();
    </script>
</body>
</html>'''

    def _stream_video(self):
        """Stream video at ~30 FPS for low-latency remote control."""
        target_period_s = 1.0 / 30.0
        while True:
            try:
                loop_start = time.time()
                with state_lock:
                    if state['frame'] is not None:
                        frame = state['frame'].copy()
                    else:
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(frame, "Waiting for camera...", (200, 240),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                # Lower quality for faster encoding/transfer (reduces latency)
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
                try:
                    self.wfile.flush()
                except Exception:
                    pass

                # Target 30 FPS (account for encode + send time)
                elapsed = time.time() - loop_start
                sleep_s = target_period_s - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)
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
        
        # Publishers - for keyboard control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriptions
        self.debug_sub = self.create_subscription(
            Image, '/debug_image', self.debug_callback, 10)
        self.raw_sub = self.create_subscription(
            Image, '/image_raw', self.raw_callback, 10)
        self.target_sub = self.create_subscription(
            PointStamped, '/target_person', self.target_callback, 10)
        
        self.get_logger().info('Subscribed to: /debug_image, /image_raw, /target_person')
        self.get_logger().info('Publishing to: /cmd_vel')
        
        self.has_debug = False
        self.frame_times = []
        self.cmd_times = []
        
        self.get_logger().info('WYZECAR Remote Control - http://0.0.0.0:8080')
        self.get_logger().info('Use WASD keys to drive!')
        self.create_timer(0.5, self.update_metrics)
    
    def publish_cmd_vel(self, linear: float, angular: float):
        """Publish velocity command from keyboard input."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        
        # Update state for display
        with state_lock:
            state['cmd_vel'] = {'linear': linear, 'angular': angular}
            state['cmd_vel_ts'] = time.time()
            state['cmd_count'] += 1
            self.cmd_times.append(time.time())

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


    def update_metrics(self):
        now = time.time()
        should_publish_stop = False
        with state_lock:
            target_age = now - state['target_ts'] if state['target_ts'] > 0 else 999
            cmd_age = now - state['cmd_vel_ts'] if state['cmd_vel_ts'] > 0 else 999
            
            # Log status periodically
            linear = state['cmd_vel']['linear']
            angular = state['cmd_vel']['angular']
            self.get_logger().info(
                f'[CTRL] Speed={linear:.2f} Turn={angular:.2f} | '
                f'Frames={state["frame_count"]} Cmds={state["cmd_count"]}'
            )
            
            # Clear stale target data
            if target_age > 2.0:
                state['target'] = None
                state['target_vx'] = 0.0
                state['target_vd'] = 0.0
            
            # Stop motors if no commands for 0.5s
            if cmd_age > 0.5:
                # Avoid deadlock: do NOT publish while holding state_lock.
                if state['cmd_vel'].get('linear', 0) != 0 or state['cmd_vel'].get('angular', 0) != 0:
                    state['cmd_vel'] = {'linear': 0, 'angular': 0}
                    should_publish_stop = True
            
            # Calculate rates
            self.frame_times = [t for t in self.frame_times if now - t < 2.0]
            self.cmd_times = [t for t in self.cmd_times if now - t < 2.0]
            
            state['detection_fps'] = len(self.frame_times) / 2.0 if self.frame_times else 0
            state['cmd_rate'] = len(self.cmd_times) / 2.0 if self.cmd_times else 0

        if should_publish_stop:
            self.publish_cmd_vel(0.0, 0.0)


def main():
    global ros_node
    
    rclpy.init()
    node = WebViewerNode()
    ros_node = node  # Set global reference for HTTP handler
    
    # Threaded server is required: /stream is a long-lived response; without threading
    # it will block /cmd and /status for most browsers (notably Chrome).
    server = ThreadingHTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print("\n" + "="*60)
    print("  WYZECAR Remote Control")
    print("  http://<DART-IP>:8080")
    print("")
    print("  Controls:")
    print("    W - Forward")
    print("    S - Backward")
    print("    A - Turn Left")
    print("    D - Turn Right")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

