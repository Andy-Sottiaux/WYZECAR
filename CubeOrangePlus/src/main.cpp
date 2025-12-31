/*
 * WyzeCar Motor Control - ESP32 WROOM + L298N + Cube Orange Plus
 *
 * This firmware reads PWM throttle signals from Cube Orange Plus and
 * controls the L298N motor driver accordingly. It also accepts serial
 * commands for manual testing.
 *
 * Operation Modes:
 *   1. AUTO: Cube Orange Plus PWM input controls motors (ArduRover)
 *   2. MANUAL: Serial commands control motors (for testing)
 *
 * Serial Protocol:
 *   M1:<speed>    - Set Motor 1 speed (-255 to 255)
 *   M2:<speed>    - Set Motor 2 speed (-255 to 255)
 *   STOP          - Stop both motors
 *   STATUS        - Get current motor status
 *   MODE:AUTO     - Switch to autopilot mode
 *   MODE:MANUAL   - Switch to manual mode
 *
 * PWM Input (from Cube):
 *   1000us = Full Reverse
 *   1500us = Stop (neutral)
 *   2000us = Full Forward
 */

#include <Arduino.h>

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

// Cube Orange Plus PWM Input
#define THROTTLE_PWM_PIN  34  // GPIO34 (input only, ADC1_CH6)

// Status LED (built-in on most ESP32 boards)
#define LED_PIN  2

// ============================================================================
// LEDC PWM CONFIGURATION
// ============================================================================

#define PWM_FREQ       5000   // 5 kHz PWM frequency
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)
#define PWM_CHANNEL_A  0      // LEDC channel for Motor 1
#define PWM_CHANNEL_B  1      // LEDC channel for Motor 2

// ============================================================================
// PWM INPUT CONFIGURATION
// ============================================================================

#define PWM_MIN_US     1000   // Minimum pulse width (full reverse)
#define PWM_MID_US     1500   // Neutral (stop)
#define PWM_MAX_US     2000   // Maximum pulse width (full forward)
#define PWM_DEADBAND   50     // Deadband around neutral (+/- microseconds)
#define PWM_TIMEOUT_MS 500    // Signal lost timeout

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Motor state
volatile int motor1Speed = 0;
volatile int motor2Speed = 0;

// PWM input measurement (using interrupts)
volatile unsigned long pwmRiseTime = 0;
volatile unsigned long pwmPulseWidth = 0;
volatile unsigned long lastPwmUpdate = 0;
volatile bool newPwmData = false;

// Operating mode
enum OperatingMode { MODE_MANUAL, MODE_AUTO };
volatile OperatingMode currentMode = MODE_AUTO;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupMotorPins();
void setupPwmInput();
void setMotor1(int speed);
void setMotor2(int speed);
void setBothMotors(int speed);
void stopMotors();
void processCommand(String cmd);
void sendStatus();
void processThrottlePwm();
int pwmToSpeed(unsigned long pulseWidth);
void IRAM_ATTR pwmISR();

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

    // Setup PWM input from Cube
    setupPwmInput();

    // Start with motors stopped
    stopMotors();

    Serial.println();
    Serial.println("========================================");
    Serial.println("  WyzeCar ESP32 Motor Controller");
    Serial.println("  Cube Orange Plus + L298N Interface");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  M1:<speed>   - Motor 1 (-255 to 255)");
    Serial.println("  M2:<speed>   - Motor 2 (-255 to 255)");
    Serial.println("  STOP         - Stop all motors");
    Serial.println("  STATUS       - Show current status");
    Serial.println("  MODE:AUTO    - Autopilot mode (Cube PWM)");
    Serial.println("  MODE:MANUAL  - Manual serial control");
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

    Serial.println("Motor pins configured");
}

// ============================================================================
// PWM INPUT SETUP (FROM CUBE ORANGE PLUS)
// ============================================================================

void setupPwmInput() {
    pinMode(THROTTLE_PWM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PWM_PIN), pwmISR, CHANGE);
    Serial.println("PWM input configured on GPIO34");
}

// PWM Input Interrupt Service Routine
void IRAM_ATTR pwmISR() {
    unsigned long now = micros();

    if (digitalRead(THROTTLE_PWM_PIN) == HIGH) {
        // Rising edge - record start time
        pwmRiseTime = now;
    } else {
        // Falling edge - calculate pulse width
        if (pwmRiseTime > 0) {
            pwmPulseWidth = now - pwmRiseTime;
            lastPwmUpdate = millis();
            newPwmData = true;
        }
    }
}

// ============================================================================
// THROTTLE PWM PROCESSING
// ============================================================================

void processThrottlePwm() {
    // Check for signal timeout
    if (millis() - lastPwmUpdate > PWM_TIMEOUT_MS) {
        // No signal - stop motors for safety
        if (motor1Speed != 0 || motor2Speed != 0) {
            stopMotors();
            Serial.println("PWM signal lost - motors stopped");
        }
        return;
    }

    // Process new PWM data
    if (newPwmData) {
        newPwmData = false;

        // Get pulse width (disable interrupts briefly for atomic read)
        noInterrupts();
        unsigned long pulse = pwmPulseWidth;
        interrupts();

        // Validate pulse width
        if (pulse >= 900 && pulse <= 2100) {
            int speed = pwmToSpeed(pulse);
            setBothMotors(speed);
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

void stopMotors() {
    setMotor1(0);
    setMotor2(0);
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
    else if (cmd == "STOP") {
        stopMotors();
        Serial.println("OK STOPPED");
    }
    else if (cmd == "STATUS") {
        sendStatus();
    }
    else if (cmd == "MODE:AUTO") {
        currentMode = MODE_AUTO;
        stopMotors();
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

    noInterrupts();
    unsigned long pulse = pwmPulseWidth;
    unsigned long lastUpdate = lastPwmUpdate;
    interrupts();

    Serial.print("PWM Pulse: ");
    Serial.print(pulse);
    Serial.println(" us");

    bool signalValid = (millis() - lastUpdate) < PWM_TIMEOUT_MS;
    Serial.print("PWM Signal: ");
    Serial.println(signalValid ? "OK" : "LOST");
    Serial.println("--------------");
}
