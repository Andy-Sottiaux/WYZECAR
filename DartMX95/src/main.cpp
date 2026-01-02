/*
  ESP32 Motor Control Firmware for WYZECAR Vision-Based Human Following System
  
  This firmware runs on ESP32 WROOM and controls:
  - L298N dual motor driver for differential drive motors
  - Servo motor for steering
  - UART communication with DART-MX95 for high-level commands
  
  Hardware Connections:
  - Motor A PWM: GPIO25 (ENA)
  - Motor A Dir: GPIO26 (IN1), GPIO27 (IN2) 
  - Motor B PWM: GPIO14 (ENB)
  - Motor B Dir: GPIO12 (IN3), GPIO13 (IN4)
  - Servo PWM: GPIO33
  - UART: GPIO16 (RX), GPIO17 (TX)
  
  Communication Protocol:
  - Baud: 115200
  - Commands: "MOTOR A FORWARD 75\n", "SERVO 90\n", "STOP\n", "STATUS\n"
  - Responses: "OK\n", "ERROR <msg>\n", "STATUS motor_a:75 motor_b:50 servo:90\n"
*/

#include <WiFi.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

// Motor Control Pins
#define MOTOR_A_PWM    25   // ENA - Motor A speed control
#define MOTOR_A_IN1    26   // IN1 - Motor A direction
#define MOTOR_A_IN2    27   // IN2 - Motor A direction
#define MOTOR_B_PWM    14   // ENB - Motor B speed control  
#define MOTOR_B_IN3    12   // IN3 - Motor B direction
#define MOTOR_B_IN4    13   // IN4 - Motor B direction

// Servo Control Pin
#define SERVO_PIN      33   // Servo signal

// UART Communication
#define UART_RX        16   // RX from DART-MX95
#define UART_TX        17   // TX to DART-MX95
#define UART_BAUD      115200

// PWM Configuration
#define PWM_FREQ       1000  // 1kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0     // PWM channel for Motor A
#define PWM_CHANNEL_B  1     // PWM channel for Motor B

// Safety Configuration
#define WATCHDOG_TIMEOUT  1000  // 1 second timeout
#define MAX_SPEED         100   // Maximum speed percentage
#define MIN_SERVO_ANGLE   0     // Minimum servo angle
#define MAX_SERVO_ANGLE   180   // Maximum servo angle

// Global Variables
Servo steeringServo;
HardwareSerial dartSerial(1);  // Use UART1 for communication

struct MotorState {
  int speed;        // 0-100 percentage
  String direction; // "FORWARD", "BACKWARD", "STOP"
};

MotorState motorA = {0, "STOP"};
MotorState motorB = {0, "STOP"};
int servoAngle = 90;  // Center position

unsigned long lastCommandTime = 0;
bool systemEnabled = true;

// Function declarations
void processCommand(String command);
void handleMotorCommand(String command);
void handleServoCommand(String command);
void handleStopCommand();
void handleStatusCommand();
void setMotorA(String direction, int speed);
void setMotorB(String direction, int speed);
void stopAllMotors();

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("ESP32 Motor Controller Starting...");
  
  // Initialize UART communication with DART-MX95
  dartSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  
  // Configure Motor A pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM, PWM_CHANNEL_A);
  
  // Configure Motor B pins
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM, PWM_CHANNEL_B);
  
  // Configure Servo - Hitec HS-85MG requires 50Hz PWM, 1-2ms pulse width
  steeringServo.attach(SERVO_PIN, 1000, 2000);  // min 1ms, max 2ms pulse width
  steeringServo.write(servoAngle);
  
  // Initialize motors to stopped state
  stopAllMotors();
  
  Serial.println("ESP32 Motor Controller Ready");
  dartSerial.println("ESP32 MOTOR CONTROLLER READY");
  
  lastCommandTime = millis();
}

void loop() {
  // Check for incoming commands
  if (dartSerial.available()) {
    String command = dartSerial.readStringUntil('\n');
    command.trim();
    processCommand(command);
    lastCommandTime = millis();
  }
  
  // Safety watchdog - stop motors if no command received
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (systemEnabled) {
      Serial.println("Watchdog timeout - stopping motors");
      stopAllMotors();
      systemEnabled = false;
      dartSerial.println("ERROR WATCHDOG_TIMEOUT");
    }
  }
  
  // Send periodic heartbeat
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 500) {
    dartSerial.println("WATCHDOG");
    lastHeartbeat = millis();
  }
  
  delay(10);  // Small delay to prevent tight loop
}

void processCommand(String command) {
  Serial.println("Received: " + command);
  
  // Reset system if it was disabled by watchdog
  if (!systemEnabled) {
    systemEnabled = true;
  }
  
  if (command.startsWith("MOTOR")) {
    handleMotorCommand(command);
  }
  else if (command.startsWith("SERVO")) {
    handleServoCommand(command);
  }
  else if (command == "STOP") {
    handleStopCommand();
  }
  else if (command == "STATUS") {
    handleStatusCommand();
  }
  else {
    dartSerial.println("ERROR UNKNOWN_COMMAND");
    Serial.println("Unknown command: " + command);
  }
}

void handleMotorCommand(String command) {
  // Parse: "MOTOR A FORWARD 75" or "MOTOR B BACKWARD 50"
  int firstSpace = command.indexOf(' ', 6);  // After "MOTOR "
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  if (firstSpace == -1 || secondSpace == -1) {
    dartSerial.println("ERROR INVALID_MOTOR_COMMAND");
    return;
  }
  
  String motor = command.substring(6, firstSpace);
  String direction = command.substring(firstSpace + 1, secondSpace);
  int speed = command.substring(secondSpace + 1).toInt();
  
  // Validate inputs
  if (motor != "A" && motor != "B") {
    dartSerial.println("ERROR INVALID_MOTOR");
    return;
  }
  
  if (direction != "FORWARD" && direction != "BACKWARD" && direction != "STOP") {
    dartSerial.println("ERROR INVALID_DIRECTION");
    return;
  }
  
  if (speed < 0 || speed > MAX_SPEED) {
    dartSerial.println("ERROR INVALID_SPEED");
    return;
  }
  
  // Execute motor command
  if (motor == "A") {
    setMotorA(direction, speed);
    motorA.direction = direction;
    motorA.speed = speed;
  } else {
    setMotorB(direction, speed);
    motorB.direction = direction;
    motorB.speed = speed;
  }
  
  dartSerial.println("OK");
  Serial.println("Motor " + motor + " set to " + direction + " at " + String(speed) + "%");
}

void handleServoCommand(String command) {
  // Parse: "SERVO 90"
  int angle = command.substring(6).toInt();
  
  if (angle < MIN_SERVO_ANGLE || angle > MAX_SERVO_ANGLE) {
    dartSerial.println("ERROR INVALID_SERVO_ANGLE");
    return;
  }
  
  steeringServo.write(angle);
  servoAngle = angle;
  
  dartSerial.println("OK");
  Serial.println("Servo set to " + String(angle) + " degrees");
}

void handleStopCommand() {
  stopAllMotors();
  dartSerial.println("OK");
  Serial.println("Emergency stop executed");
}

void handleStatusCommand() {
  String status = "STATUS motor_a:" + String(motorA.speed) + 
                  " motor_b:" + String(motorB.speed) + 
                  " servo:" + String(servoAngle);
  dartSerial.println(status);
  Serial.println("Status sent: " + status);
}

void setMotorA(String direction, int speed) {
  int pwmValue = map(speed, 0, 100, 0, 255);
  
  if (direction == "FORWARD") {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, pwmValue);
  }
  else if (direction == "BACKWARD") {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_A, pwmValue);
  }
  else { // STOP
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
  }
}

void setMotorB(String direction, int speed) {
  int pwmValue = map(speed, 0, 100, 0, 255);
  
  if (direction == "FORWARD") {
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
    ledcWrite(PWM_CHANNEL_B, pwmValue);
  }
  else if (direction == "BACKWARD") {
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
    ledcWrite(PWM_CHANNEL_B, pwmValue);
  }
  else { // STOP
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
    ledcWrite(PWM_CHANNEL_B, 0);
  }
}

void stopAllMotors() {
  setMotorA("STOP", 0);
  setMotorB("STOP", 0);
  motorA = {0, "STOP"};
  motorB = {0, "STOP"};
}