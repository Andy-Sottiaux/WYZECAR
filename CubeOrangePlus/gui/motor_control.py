#!/usr/bin/env python3
"""
WyzeCar Motor & Servo Control GUI
Interface to control L298N motors and steering servo via ESP32 WROOM

For manual testing and debugging. The ESP32 must be in MANUAL mode
(send MODE:MANUAL command) to accept serial motor/servo commands.
In AUTO mode, the ESP32 receives throttle/steering commands from Cube Orange Plus.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time


class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("WyzeCar Motor & Servo Control")
        self.root.geometry("520x700")
        self.root.resizable(False, False)
        
        # Configure styles
        self.setup_styles()

        self.serial_conn = None
        self.connected = False
        self.motor1_speed = tk.IntVar(value=0)
        self.motor2_speed = tk.IntVar(value=0)
        self.servo_angle = tk.IntVar(value=90)
        self.current_mode = tk.StringVar(value="Unknown")

        self.create_widgets()
        self.refresh_ports()

    def setup_styles(self):
        style = ttk.Style()
        style.configure("Danger.TButton", foreground="red")
        style.configure("Servo.TFrame", background="#e8f4e8")
        
    def create_widgets(self):
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=5
        )
        self.connect_btn = ttk.Button(
            conn_frame, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.grid(row=0, column=3, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)

        # Mode Frame
        mode_frame = ttk.LabelFrame(self.root, text="ESP32 Mode", padding=10)
        mode_frame.pack(fill="x", padx=10, pady=5)

        mode_btn_frame = ttk.Frame(mode_frame)
        mode_btn_frame.pack(fill="x")

        ttk.Button(mode_btn_frame, text="MANUAL Mode", command=self.set_manual_mode).pack(side="left", padx=5)
        ttk.Button(mode_btn_frame, text="AUTO Mode", command=self.set_auto_mode).pack(side="left", padx=5)
        ttk.Button(mode_btn_frame, text="Status", command=self.request_status).pack(side="left", padx=5)

        self.mode_label = ttk.Label(mode_btn_frame, textvariable=self.current_mode, font=("TkDefaultFont", 10, "bold"))
        self.mode_label.pack(side="right", padx=10)
        ttk.Label(mode_btn_frame, text="Current:").pack(side="right")

        mode_info = ttk.Label(mode_frame, text="MANUAL = GUI control | AUTO = Cube Orange Plus control", foreground="gray")
        mode_info.pack(pady=(5, 0))

        # Motor 1 Frame
        m1_frame = ttk.LabelFrame(self.root, text="Motor 1 (Left)", padding=10)
        m1_frame.pack(fill="x", padx=10, pady=5)

        self.m1_slider = ttk.Scale(
            m1_frame,
            from_=-255,
            to=255,
            orient="horizontal",
            variable=self.motor1_speed,
            command=lambda _: self.on_motor1_change(),
        )
        self.m1_slider.pack(fill="x", padx=10)

        m1_btn_frame = ttk.Frame(m1_frame)
        m1_btn_frame.pack(fill="x", pady=5)

        ttk.Button(m1_btn_frame, text="Full Rev", command=lambda: self.set_motor1(-255)).pack(side="left", padx=5)
        ttk.Button(m1_btn_frame, text="Half Rev", command=lambda: self.set_motor1(-128)).pack(side="left", padx=5)
        ttk.Button(m1_btn_frame, text="Stop", command=lambda: self.set_motor1(0)).pack(side="left", padx=5)
        ttk.Button(m1_btn_frame, text="Half Fwd", command=lambda: self.set_motor1(128)).pack(side="left", padx=5)
        ttk.Button(m1_btn_frame, text="Full Fwd", command=lambda: self.set_motor1(255)).pack(side="left", padx=5)

        self.m1_value_label = ttk.Label(m1_frame, text="Speed: 0")
        self.m1_value_label.pack()

        # Motor 2 Frame
        m2_frame = ttk.LabelFrame(self.root, text="Motor 2 (Right)", padding=10)
        m2_frame.pack(fill="x", padx=10, pady=5)

        self.m2_slider = ttk.Scale(
            m2_frame,
            from_=-255,
            to=255,
            orient="horizontal",
            variable=self.motor2_speed,
            command=lambda _: self.on_motor2_change(),
        )
        self.m2_slider.pack(fill="x", padx=10)

        m2_btn_frame = ttk.Frame(m2_frame)
        m2_btn_frame.pack(fill="x", pady=5)

        ttk.Button(m2_btn_frame, text="Full Rev", command=lambda: self.set_motor2(-255)).pack(side="left", padx=5)
        ttk.Button(m2_btn_frame, text="Half Rev", command=lambda: self.set_motor2(-128)).pack(side="left", padx=5)
        ttk.Button(m2_btn_frame, text="Stop", command=lambda: self.set_motor2(0)).pack(side="left", padx=5)
        ttk.Button(m2_btn_frame, text="Half Fwd", command=lambda: self.set_motor2(128)).pack(side="left", padx=5)
        ttk.Button(m2_btn_frame, text="Full Fwd", command=lambda: self.set_motor2(255)).pack(side="left", padx=5)

        self.m2_value_label = ttk.Label(m2_frame, text="Speed: 0")
        self.m2_value_label.pack()

        # Steering Servo Frame
        servo_frame = ttk.LabelFrame(self.root, text="Steering Servo (GPIO33)", padding=10)
        servo_frame.pack(fill="x", padx=10, pady=5)

        # Servo slider
        self.servo_slider = ttk.Scale(
            servo_frame,
            from_=0,
            to=180,
            orient="horizontal",
            variable=self.servo_angle,
            command=lambda _: self.on_servo_change(),
        )
        self.servo_slider.pack(fill="x", padx=10)

        # Servo buttons
        servo_btn_frame = ttk.Frame(servo_frame)
        servo_btn_frame.pack(fill="x", pady=5)

        ttk.Button(servo_btn_frame, text="Full Left (0°)", command=lambda: self.set_servo(0)).pack(side="left", padx=5)
        ttk.Button(servo_btn_frame, text="Left (45°)", command=lambda: self.set_servo(45)).pack(side="left", padx=5)
        ttk.Button(servo_btn_frame, text="Center (90°)", command=lambda: self.set_servo(90)).pack(side="left", padx=5)
        ttk.Button(servo_btn_frame, text="Right (135°)", command=lambda: self.set_servo(135)).pack(side="left", padx=5)
        ttk.Button(servo_btn_frame, text="Full Right (180°)", command=lambda: self.set_servo(180)).pack(side="left", padx=5)

        self.servo_value_label = ttk.Label(servo_frame, text="Angle: 90°")
        self.servo_value_label.pack()

        # Both Motors Frame
        both_frame = ttk.LabelFrame(self.root, text="Quick Controls", padding=10)
        both_frame.pack(fill="x", padx=10, pady=5)

        btn_frame = ttk.Frame(both_frame)
        btn_frame.pack()

        ttk.Button(btn_frame, text="⛔ STOP ALL", command=self.stop_all, style="Danger.TButton").pack(side="left", padx=10, pady=5)
        ttk.Button(btn_frame, text="⬆️ Forward", command=lambda: self.both_motors(150)).pack(side="left", padx=10, pady=5)
        ttk.Button(btn_frame, text="⬇️ Reverse", command=lambda: self.both_motors(-150)).pack(side="left", padx=10, pady=5)
        ttk.Button(btn_frame, text="🎯 Center Servo", command=lambda: self.set_servo(90)).pack(side="left", padx=10, pady=5)

        # Console Frame
        console_frame = ttk.LabelFrame(self.root, text="Console", padding=10)
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.console = tk.Text(console_frame, height=6, state="disabled", bg="#1e1e1e", fg="#00ff00", font=("Courier", 10))
        self.console.pack(fill="both", expand=True)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "No port selected")
            return

        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Wait for ESP32 reset
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            self.log(f"Connected to {port}")

            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

            # Auto-switch to MANUAL mode for GUI control
            self.root.after(500, self.set_manual_mode)

        except Exception as e:
            messagebox.showerror("Error", f"Connection failed: {e}")

    def disconnect(self):
        if self.serial_conn:
            self.serial_conn.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.current_mode.set("Unknown")
        self.log("Disconnected")

    def read_serial(self):
        while self.connected and self.serial_conn:
            try:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode("utf-8").strip()
                    if line:
                        self.root.after(0, lambda l=line: self.handle_response(l))
            except Exception:
                break
            time.sleep(0.01)

    def handle_response(self, line):
        self.log(f"< {line}")
        # Update mode indicator based on responses
        if "Mode: AUTO" in line or "AUTO (Cube PWM" in line:
            self.current_mode.set("AUTO")
            self.mode_label.config(foreground="blue")
        elif "Mode: MANUAL" in line or "MANUAL (Serial" in line:
            self.current_mode.set("MANUAL")
            self.mode_label.config(foreground="green")

    def send_command(self, cmd):
        if self.connected and self.serial_conn:
            try:
                self.serial_conn.write(f"{cmd}\n".encode())
                self.log(f"> {cmd}")
            except Exception as e:
                self.log(f"Error: {e}")

    def log(self, message):
        self.console.config(state="normal")
        self.console.insert("end", message + "\n")
        self.console.see("end")
        self.console.config(state="disabled")

    def set_manual_mode(self):
        self.send_command("MODE:MANUAL")

    def set_auto_mode(self):
        self.send_command("MODE:AUTO")

    def request_status(self):
        self.send_command("STATUS")

    # Motor 1 controls
    def set_motor1(self, speed):
        self.motor1_speed.set(speed)
        self.m1_value_label.config(text=f"Speed: {speed}")
        self.send_command(f"M1:{speed}")

    def on_motor1_change(self):
        speed = int(self.motor1_speed.get())
        self.m1_value_label.config(text=f"Speed: {speed}")
        self.send_command(f"M1:{speed}")

    # Motor 2 controls
    def set_motor2(self, speed):
        self.motor2_speed.set(speed)
        self.m2_value_label.config(text=f"Speed: {speed}")
        self.send_command(f"M2:{speed}")

    def on_motor2_change(self):
        speed = int(self.motor2_speed.get())
        self.m2_value_label.config(text=f"Speed: {speed}")
        self.send_command(f"M2:{speed}")

    # Servo controls
    def set_servo(self, angle):
        self.servo_angle.set(angle)
        self.servo_value_label.config(text=f"Angle: {angle}°")
        self.send_command(f"SERVO:{angle}")

    def on_servo_change(self):
        angle = int(self.servo_angle.get())
        self.servo_value_label.config(text=f"Angle: {angle}°")
        self.send_command(f"SERVO:{angle}")

    # Combined controls
    def stop_all(self):
        self.set_motor1(0)
        self.set_motor2(0)
        self.send_command("STOP")

    def both_motors(self, speed):
        self.set_motor1(speed)
        self.set_motor2(speed)


def main():
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
