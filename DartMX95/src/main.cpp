/*
  ESP32 I2C Motor Controller for DART-MX95
  Using ESP-IDF I2C Slave API directly (most reliable method)
  
  I2C Wiring (J6 Header on DART-MX95):
    Pin 18 (I2C3_SCL) -> ESP32 GPIO22 (SCL)
    Pin 20 (I2C3_SDA) -> ESP32 GPIO21 (SDA)
    Pin 12 (GND)      -> ESP32 GND
    
  NOTE: I2C3 has 10k pull-ups on the DART-MX95 SOM
    
  ESP32 I2C Address: 0x42
  
  Commands (from DART-MX95):
    0x01 [left] [right] [servo]  - Set motor speeds and servo
    0x02                          - Emergency stop
    0x03                          - Request status
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/i2c.h"

// I2C Configuration
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_SLAVE_ADDR 0x42
#define I2C_PORT I2C_NUM_0
#define I2C_RX_BUF_LEN 128
#define I2C_TX_BUF_LEN 128

// L298N Motor Driver Pins
#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 12
#define IN4 13
#define ENB 14

// Servo Pin and PWM Configuration
#define SERVO_PIN 33
#define SERVO_MIN_US 1300   // Minimum pulse width (full left)
#define SERVO_MAX_US 1700   // Maximum pulse width (full right)
#define SERVO_CENTER_US 1500 // Center pulse width

// LED Pin
#define LED_PIN 2

// Command codes
#define CMD_SET_MOTORS     0x01
#define CMD_EMERGENCY_STOP 0x02
#define CMD_REQUEST_STATUS 0x03

// Motor control variables
volatile int8_t leftSpeed = 0;
volatile int8_t rightSpeed = 0;
volatile uint8_t servoAngle = 90;
volatile bool emergencyStop = false;
volatile bool newCommandReceived = false;
volatile unsigned long lastI2CActivity = 0;

// Watchdog
unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 2000;

Servo steeringServo;

// Function declarations
void setMotorSpeed(int8_t left, int8_t right);
void stopMotors();
bool initI2CSlave();
void processI2CData();

bool initI2CSlave() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_SLAVE;
  conf.sda_io_num = I2C_SDA_PIN;
  conf.scl_io_num = I2C_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.slave.addr_10bit_en = 0;
  conf.slave.slave_addr = I2C_SLAVE_ADDR;
  conf.slave.maximum_speed = 100000;
  conf.clk_flags = 0;
  
  esp_err_t err = i2c_param_config(I2C_PORT, &conf);
  if (err != ESP_OK) {
    Serial.printf("I2C param config failed: %d\n", err);
    return false;
  }
  
  err = i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE, I2C_RX_BUF_LEN, I2C_TX_BUF_LEN, 0);
  if (err != ESP_OK) {
    Serial.printf("I2C driver install failed: %d\n", err);
    return false;
  }
  
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  ESP32 I2C Motor Controller v3.0           ║");
  Serial.println("║  Using ESP-IDF I2C Slave API               ║");
  Serial.println("║  WYZECAR Vision-Based Following System     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Initialize servo with custom PWM range (1300-1700µs)
  // 0° = 1300µs (left), 90° = 1500µs (center), 180° = 1700µs (right)
  steeringServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  steeringServo.write(servoAngle);  // Start at center (90°)
  
  stopMotors();
  
  Serial.println("\n[1/3] Motor & Servo configured");
  
  // Initialize I2C Slave using ESP-IDF API
  Serial.println("\n[2/3] Initializing I2C Slave (ESP-IDF API)...");
  
  if (initI2CSlave()) {
    Serial.println("      ✓ I2C Slave initialized!");
    Serial.printf("      Address: 0x%02X\n", I2C_SLAVE_ADDR);
    Serial.printf("      SDA: GPIO%d, SCL: GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  } else {
    Serial.println("      ✗ I2C Slave FAILED!");
  }
  
  Serial.println("\n[3/3] System Ready!");
  Serial.println("══════════════════════════════════════════════");
  Serial.println("Test on DART-MX95:");
  Serial.println("  i2cdetect -y 3");
  Serial.println("  i2cset -y 3 0x42 0x01 50 50 90 i");
  Serial.println("══════════════════════════════════════════════\n");
  
  lastCommandTime = millis();
  
  // Flash LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  static unsigned long lastStatusPrint = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  // Check for I2C data
  processI2CData();
  
  // Status print every 5 seconds
  if (millis() - lastStatusPrint > 5000) {
    Serial.printf("[%lus] I2C 0x%02X | L=%d R=%d S=%d | Last: %lums ago\n", 
                  millis()/1000, I2C_SLAVE_ADDR,
                  leftSpeed, rightSpeed, servoAngle,
                  lastI2CActivity > 0 ? millis() - lastI2CActivity : 0);
    lastStatusPrint = millis();
  }
  
  // LED patterns
  if (emergencyStop) {
    if (millis() - lastBlink > 100) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
  } else if (lastI2CActivity > 0 && millis() - lastI2CActivity < 1000) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (millis() - lastBlink > 1000) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
  }
  
  // Apply motor commands
  if (newCommandReceived) {
    newCommandReceived = false;
    lastCommandTime = millis();
    
    if (!emergencyStop) {
      setMotorSpeed(leftSpeed, rightSpeed);
      steeringServo.write(servoAngle);
    }
  }
  
  // Watchdog
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
    if (!emergencyStop && (leftSpeed != 0 || rightSpeed != 0)) {
      Serial.println("⚠️  Watchdog timeout");
      emergencyStop = true;
      leftSpeed = 0;
      rightSpeed = 0;
      stopMotors();
    }
  }
  
  if (emergencyStop) {
    stopMotors();
  }
  
  delay(1);
}

void processI2CData() {
  uint8_t buffer[16];
  
  // Try to read data from I2C slave buffer (non-blocking with 0 timeout)
  int len = i2c_slave_read_buffer(I2C_PORT, buffer, sizeof(buffer), 0);
  
  if (len > 0) {
    lastI2CActivity = millis();
    
    uint8_t command = buffer[0];
    Serial.printf("📥 I2C: cmd=0x%02X len=%d\n", command, len);
    
    switch (command) {
      case CMD_SET_MOTORS:
        if (len >= 4) {
          leftSpeed = constrain((int8_t)buffer[1], -100, 100);
          rightSpeed = constrain((int8_t)buffer[2], -100, 100);
          servoAngle = constrain(buffer[3], 0, 180);
          emergencyStop = false;
          newCommandReceived = true;
          Serial.printf("    Motors: L=%d R=%d S=%d\n", leftSpeed, rightSpeed, servoAngle);
        }
        break;
        
      case CMD_EMERGENCY_STOP:
        emergencyStop = true;
        leftSpeed = 0;
        rightSpeed = 0;
        stopMotors();
        Serial.println("🛑 Emergency Stop!");
        break;
        
      case CMD_REQUEST_STATUS: {
        uint8_t response[3];
        response[0] = (uint8_t)leftSpeed;
        response[1] = (uint8_t)rightSpeed;
        uint8_t flags = 0;
        if (leftSpeed != 0) flags |= 0x01;
        if (rightSpeed != 0) flags |= 0x02;
        if (emergencyStop) flags |= 0x04;
        response[2] = flags;
        
        i2c_slave_write_buffer(I2C_PORT, response, 3, 100 / portTICK_PERIOD_MS);
        Serial.printf("📤 Status: L=%d R=%d F=0x%02X\n", leftSpeed, rightSpeed, flags);
        break;
      }
        
      default:
        Serial.printf("    Unknown: 0x%02X\n", command);
        break;
    }
  }
}

void setMotorSpeed(int8_t left, int8_t right) {
  // Left motor
  if (left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (left < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(left) * 255 / 100);
  
  // Right motor
  if (right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (right < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(right) * 255 / 100);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
