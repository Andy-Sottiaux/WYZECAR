/*
 * WyzeCar Motor Control - ESP32 WROOM + L298N + Servo + Cube Orange Plus
 *
 * This firmware reads PWM throttle/steering signals from Cube Orange Plus and
 * controls the L298N motor driver and steering servo accordingly. It also 
 * accepts serial commands for manual testing.
 *
 * Operation Modes:
 *   1. AUTO: Cube Orange Plus PWM input controls motors/servo (ArduRover)
 *   2. MANUAL: Serial commands control motors/servo (for testing)
 *
 * Serial Protocol:
 *   M1:<speed>    - Set Motor 1 speed (-255 to 255)
 *   M2:<speed>    - Set Motor 2 speed (-255 to 255)
 *   SERVO:<angle> - Set servo angle (0 to 180)
 *   STOP          - Stop both motors
 *   STATUS        - Get current motor/servo status
 *   MODE:AUTO     - Switch to autopilot mode
 *   MODE:MANUAL   - Switch to manual mode
 *
 * PWM Input (from Cube):
 *   Throttle: 1000us = Full Reverse, 1500us = Stop, 2000us = Full Forward
 *   Steering: 1000us = Full Left, 1500us = Center, 2000us = Full Right
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============================================================================
// PIN DEFINITIONS - ESP32 WROOM
// ============================================================================

// L298N Motor Driver Pins
// Motor 1 (Left)
#define ENA_PIN  25   // PWM Enable for Motor 1
#define IN1_PIN  26   // Motor 1 Direction
#define IN2_PIN  27   // Motor 1 Direction

// Motor 2 (Right)
#define ENB_PIN  14   // PWM Enable for Motor 2
#define IN3_PIN  12   // Motor 2 Direction
#define IN4_PIN  13   // Motor 2 Direction

// Steering Servo Pin (Hitec HS-85MG)
#define SERVO_PIN  33  // GPIO33 for servo PWM

// Cube Orange Plus PWM Inputs
#define THROTTLE_PWM_PIN  34  // GPIO34 (input only) - Throttle channel
#define STEERING_PWM_PIN  35  // GPIO35 (input only) - Steering channel

// Status LED (built-in on most ESP32 boards)
#define LED_PIN  2

// ============================================================================
// LEDC PWM CONFIGURATION (for motors)
// ============================================================================

#define PWM_FREQ       5000   // 5 kHz PWM frequency
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0      // LEDC channel for Motor 1
#define PWM_CHANNEL_B  1      // LEDC channel for Motor 2

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

#define SERVO_MIN_ANGLE  0    // Minimum servo angle (full left)
#define SERVO_MAX_ANGLE  180  // Maximum servo angle (full right)
#define SERVO_CENTER     90   // Center position

// Servo PWM pulse widths (microseconds)
#define SERVO_PWM_MIN    1300  // Minimum pulse width (full left)
#define SERVO_PWM_CENTER 1500  // Center pulse width (neutral)
#define SERVO_PWM_MAX    1700  // Maximum pulse width (full right)

// ============================================================================
// PWM INPUT CONFIGURATION
// ============================================================================

#define PWM_MIN_US     1000   // Minimum pulse width (full reverse/left)
#define PWM_MID_US     1500   // Neutral (stop/center)
#define PWM_MAX_US     2000   // Maximum pulse width (full forward/right)
#define PWM_DEADBAND   50     // Deadband around neutral (+/- microseconds)
#define PWM_TIMEOUT_MS 500    // Signal lost timeout

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Motor state
volatile int motor1Speed = 0;
volatile int motor2Speed = 0;

// Servo state
Servo steeringServo;
volatile int servoAngle = SERVO_CENTER;

// Throttle PWM input measurement (using interrupts)
volatile unsigned long throttlePwmRiseTime = 0;
volatile unsigned long throttlePwmPulseWidth = 0;
volatile unsigned long lastThrottlePwmUpdate = 0;
volatile bool newThrottlePwmData = false;

// Steering PWM input measurement (using interrupts)
volatile unsigned long steeringPwmRiseTime = 0;
volatile unsigned long steeringPwmPulseWidth = 0;
volatile unsigned long lastSteeringPwmUpdate = 0;
volatile bool newSteeringPwmData = false;

// Operating mode
enum OperatingMode { MODE_MANUAL, MODE_AUTO };
volatile OperatingMode currentMode = MODE_AUTO;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupMotorPins();
void setupServoPins();
void setupPwmInput();
void setMotor1(int speed);
void setMotor2(int speed);
void setBothMotors(int speed);
void setServo(int angle);
void stopMotors();
void stopAll();
void processCommand(String cmd);
void sendStatus();
void processThrottlePwm();
void processSteeringPwm();
int pwmToSpeed(unsigned long pulseWidth);
int pwmToServoAngle(unsigned long pulseWidth);
void IRAM_ATTR throttlePwmISR();
void IRAM_ATTR steeringPwmISR();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(100);

    // Configure LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Setup motor control
    setupMotorPins();
    
    // Setup servo control
    setupServoPins();

    // Setup PWM input from Cube
    setupPwmInput();

    // Start with motors stopped and servo centered
    stopAll();

    Serial.println();
    Serial.println("========================================");
    Serial.println("  WyzeCar ESP32 Motor Controller v2.0");
    Serial.println("  Cube Orange Plus + L298N + Servo");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  M1:<speed>    - Motor 1 (-255 to 255)");
    Serial.println("  M2:<speed>    - Motor 2 (-255 to 255)");
    Serial.println("  SERVO:<angle> - Servo (0 to 180)");
    Serial.println("  STOP          - Stop all motors");
    Serial.println("  STATUS        - Show current status");
    Serial.println("  MODE:AUTO     - Autopilot mode (Cube PWM)");
    Serial.println("  MODE:MANUAL   - Manual serial control");
    Serial.println();
    Serial.println("Current mode: AUTO (Cube PWM control)");
    Serial.println("========================================");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Process serial commands (always available)
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            processCommand(command);
        }
    }

    // Process Cube PWM input in AUTO mode
    if (currentMode == MODE_AUTO) {
        processThrottlePwm();
        processSteeringPwm();
    }

    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    }
}

// ============================================================================
// MOTOR CONTROL SETUP
// ============================================================================

void setupMotorPins() {
    // Configure direction pins as outputs
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    // Configure LEDC PWM for motor speed control
    // Motor 1 PWM
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENA_PIN, PWM_CHANNEL_A);

    // Motor 2 PWM
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENB_PIN, PWM_CHANNEL_B);

    Serial.println("Motor pins configured (ENA=25, IN1=26, IN2=27, IN3=12, IN4=13, ENB=14)");
}

// ============================================================================
// SERVO CONTROL SETUP
// ============================================================================

void setupServoPins() {
    // Allow allocation of all timers for servo
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Attach servo to pin with standard 50Hz frequency
    steeringServo.setPeriodHertz(50);  // Standard 50Hz servo
    steeringServo.attach(SERVO_PIN, SERVO_PWM_MIN, SERVO_PWM_MAX);  // 1300-1700us range
    
    // Center the servo
    steeringServo.write(SERVO_CENTER);
    servoAngle = SERVO_CENTER;
    
    Serial.println("Servo configured on GPIO33 (50Hz, 1300-1700us, center=1500us)");
}

// ============================================================================
// PWM INPUT SETUP (FROM CUBE ORANGE PLUS)
// ============================================================================

void setupPwmInput() {
    // Throttle input
    pinMode(THROTTLE_PWM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PWM_PIN), throttlePwmISR, CHANGE);
    
    // Steering input
    pinMode(STEERING_PWM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STEERING_PWM_PIN), steeringPwmISR, CHANGE);
    
    Serial.println("PWM input configured: Throttle=GPIO34, Steering=GPIO35");
}

// Throttle PWM Input Interrupt Service Routine
void IRAM_ATTR throttlePwmISR() {
    unsigned long now = micros();

    if (digitalRead(THROTTLE_PWM_PIN) == HIGH) {
        // Rising edge - record start time
        throttlePwmRiseTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (throttlePwmRiseTime > 0) {
            throttlePwmPulseWidth = now - throttlePwmRiseTime;
            lastThrottlePwmUpdate = millis();
            newThrottlePwmData = true;
        }
    }
}

// Steering PWM Input Interrupt Service Routine
void IRAM_ATTR steeringPwmISR() {
    unsigned long now = micros();

    if (digitalRead(STEERING_PWM_PIN) == HIGH) {
        // Rising edge - record start time
        steeringPwmRiseTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (steeringPwmRiseTime > 0) {
            steeringPwmPulseWidth = now - steeringPwmRiseTime;
            lastSteeringPwmUpdate = millis();
            newSteeringPwmData = true;
        }
    }
}

// ============================================================================
// THROTTLE PWM PROCESSING
// ============================================================================

void processThrottlePwm() {
    // Check for signal timeout
    if (millis() - lastThrottlePwmUpdate > PWM_TIMEOUT_MS) {
        // No signal - stop motors for safety
        if (motor1Speed != 0 || motor2Speed != 0) {
            stopMotors();
            Serial.println("Throttle PWM signal lost - motors stopped");
        }
        return;
    }

    // Process new PWM data
    if (newThrottlePwmData) {
        newThrottlePwmData = false;

        // Get pulse width (disable interrupts briefly for atomic read)
        noInterrupts();
        unsigned long pulse = throttlePwmPulseWidth;
        interrupts();

        // Validate pulse width
        if (pulse >= 900 && pulse <= 2100) {
            int speed = pwmToSpeed(pulse);
            setBothMotors(speed);
        }
    }
}

// ============================================================================
// STEERING PWM PROCESSING
// ============================================================================

void processSteeringPwm() {
    // Check for signal timeout - keep last position if signal lost
    if (millis() - lastSteeringPwmUpdate > PWM_TIMEOUT_MS) {
        // No signal - center servo for safety (optional, could keep last position)
        // setServo(SERVO_CENTER);
        return;
    }

    // Process new PWM data
    if (newSteeringPwmData) {
        newSteeringPwmData = false;

        // Get pulse width (disable interrupts briefly for atomic read)
        noInterrupts();
        unsigned long pulse = steeringPwmPulseWidth;
        interrupts();

        // Validate pulse width
        if (pulse >= 900 && pulse <= 2100) {
            int angle = pwmToServoAngle(pulse);
            setServo(angle);
        }
    }
}

// Convert PWM pulse width to motor speed (-255 to 255)
int pwmToSpeed(unsigned long pulseWidth) {
    // Apply deadband around neutral
    if (pulseWidth > (PWM_MID_US - PWM_DEADBAND) &&
        pulseWidth < (PWM_MID_US + PWM_DEADBAND)) {
        return 0;
    }

    int speed;
    if (pulseWidth < PWM_MID_US) {
        // Reverse: map 1000-1450 to -255 to 0
        speed = map(pulseWidth, PWM_MIN_US, PWM_MID_US - PWM_DEADBAND, -255, 0);
    } else {
        // Forward: map 1550-2000 to 0 to 255
        speed = map(pulseWidth, PWM_MID_US + PWM_DEADBAND, PWM_MAX_US, 0, 255);
    }

    return constrain(speed, -255, 255);
}

// Convert PWM pulse width to servo angle (0 to 180)
int pwmToServoAngle(unsigned long pulseWidth) {
    // Map 1000-2000us to 0-180 degrees
    int angle = map(pulseWidth, PWM_MIN_US, PWM_MAX_US, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    return constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotor1(int speed) {
    motor1Speed = constrain(speed, -255, 255);

    if (motor1Speed > 0) {
        // Forward
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, motor1Speed);
    }
    else if (motor1Speed < 0) {
        // Reverse
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        ledcWrite(PWM_CHANNEL_A, -motor1Speed);
    }
    else {
        // Stop
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    }
}

void setMotor2(int speed) {
    motor2Speed = constrain(speed, -255, 255);

    if (motor2Speed > 0) {
        // Forward
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
        ledcWrite(PWM_CHANNEL_B, motor2Speed);
    }
    else if (motor2Speed < 0) {
        // Reverse
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
        ledcWrite(PWM_CHANNEL_B, -motor2Speed);
    }
    else {
        // Stop
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
        ledcWrite(PWM_CHANNEL_B, 0);
    }
}

void setBothMotors(int speed) {
    setMotor1(speed);
    setMotor2(speed);
}

// ============================================================================
// SERVO CONTROL FUNCTION
// ============================================================================

void setServo(int angle) {
    servoAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    steeringServo.write(servoAngle);
}

// ============================================================================
// STOP FUNCTIONS
// ============================================================================

void stopMotors() {
    setMotor1(0);
    setMotor2(0);
}

void stopAll() {
    stopMotors();
    setServo(SERVO_CENTER);
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

void processCommand(String cmd) {
    cmd.toUpperCase();

    if (cmd.startsWith("M1:")) {
        if (currentMode != MODE_MANUAL) {
            Serial.println("WARN: Switch to MANUAL mode first (MODE:MANUAL)");
            return;
        }
        int speed = cmd.substring(3).toInt();
        speed = constrain(speed, -255, 255);
        setMotor1(speed);
        Serial.print("OK M1:");
        Serial.println(speed);
    }
    else if (cmd.startsWith("M2:")) {
        if (currentMode != MODE_MANUAL) {
            Serial.println("WARN: Switch to MANUAL mode first (MODE:MANUAL)");
            return;
        }
        int speed = cmd.substring(3).toInt();
        speed = constrain(speed, -255, 255);
        setMotor2(speed);
        Serial.print("OK M2:");
        Serial.println(speed);
    }
    else if (cmd.startsWith("SERVO:")) {
        if (currentMode != MODE_MANUAL) {
            Serial.println("WARN: Switch to MANUAL mode first (MODE:MANUAL)");
            return;
        }
        int angle = cmd.substring(6).toInt();
        angle = constrain(angle, 0, 180);
        setServo(angle);
        Serial.print("OK SERVO:");
        Serial.println(angle);
    }
    else if (cmd == "STOP") {
        stopMotors();
        Serial.println("OK STOPPED");
    }
    else if (cmd == "CENTER") {
        setServo(SERVO_CENTER);
        Serial.println("OK SERVO CENTERED");
    }
    else if (cmd == "STATUS") {
        sendStatus();
    }
    else if (cmd == "MODE:AUTO") {
        currentMode = MODE_AUTO;
        stopAll();
        Serial.println("OK Mode: AUTO (Cube PWM control)");
    }
    else if (cmd == "MODE:MANUAL") {
        currentMode = MODE_MANUAL;
        stopMotors();
        Serial.println("OK Mode: MANUAL (Serial control)");
    }
    else {
        Serial.print("ERR Unknown: ");
        Serial.println(cmd);
    }
}

void sendStatus() {
    Serial.println("--- STATUS ---");
    Serial.print("Mode: ");
    Serial.println(currentMode == MODE_AUTO ? "AUTO" : "MANUAL");
    Serial.print("Motor 1: ");
    Serial.println(motor1Speed);
    Serial.print("Motor 2: ");
    Serial.println(motor2Speed);
    Serial.print("Servo: ");
    Serial.print(servoAngle);
    Serial.println(" deg");

    noInterrupts();
    unsigned long throttlePulse = throttlePwmPulseWidth;
    unsigned long throttleLastUpdate = lastThrottlePwmUpdate;
    unsigned long steeringPulse = steeringPwmPulseWidth;
    unsigned long steeringLastUpdate = lastSteeringPwmUpdate;
    interrupts();

    Serial.print("Throttle PWM: ");
    Serial.print(throttlePulse);
    Serial.print(" us (");
    Serial.print((millis() - throttleLastUpdate) < PWM_TIMEOUT_MS ? "OK" : "LOST");
    Serial.println(")");

    Serial.print("Steering PWM: ");
    Serial.print(steeringPulse);
    Serial.print(" us (");
    Serial.print((millis() - steeringLastUpdate) < PWM_TIMEOUT_MS ? "OK" : "LOST");
    Serial.println(")");

    Serial.println("--------------");
}
