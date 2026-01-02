I2C Communication - DART-MX95 to ESP32
========================================

Overview:
---------
Using I2C communication between DART-MX95 Sonata board and ESP32 for motor control.
The ESP32 firmware uses the ESP-IDF I2C slave API directly (not Arduino Wire library)
for reliable operation.

IMPORTANT: The Arduino Wire library I2C slave mode is BROKEN on ESP32!
Use the ESP-IDF driver/i2c.h API directly for stable slave operation.

Available I2C Buses:
--------------------
/dev/i2c-2  - Used by carrier board devices (RTC, touch, USB-C, etc.)
/dev/i2c-3  - COMPLETELY FREE - Used for ESP32 communication ✓
/dev/i2c-7  - Used by audio codec and GPIO expanders

Recommended Bus: I2C-3 (cleanest, no conflicts)

Hardware Connections (J6 Header):
---------------------------------
Pin 18: I2C3_SCL  -> ESP32 GPIO22 (SCL)
Pin 20: I2C3_SDA  -> ESP32 GPIO21 (SDA)
Pin 12: GND       -> ESP32 GND

Note: I2C3 has 10k pull-ups on the SOM - no external pull-ups needed!

I2C Configuration:
------------------
- Bus: I2C3 (/dev/i2c-3)
- ESP32 Address: 0x42 (7-bit address)
- Speed: 100kHz (standard mode)
- Pull-ups: 10k ohms on SOM (built-in)

Verified Working:
-----------------
$ i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- 42 -- -- -- -- -- -- -- -- -- -- -- -- --   <-- ESP32 detected!
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         

Command Protocol:
-----------------
DART-MX95 -> ESP32 commands (write operations):

Cmd 0x01: Set motor speeds and servo
  Data: [0x01, left_speed, right_speed, servo_angle]
  left_speed/right_speed: -100 to +100 (signed 8-bit)
  servo_angle: 0-180 degrees
  Example: i2cset -y 3 0x42 0x01 50 50 90 i

Cmd 0x02: Emergency stop
  Data: [0x02]
  Example: i2cset -y 3 0x42 0x02

Cmd 0x03: Request status
  Data: [0x03]
  Example: i2cset -y 3 0x42 0x03

ESP32 -> DART-MX95 responses (read operations):
Format: [left_speed, right_speed, status_byte]

Status byte bits:
- Bit 0: Left motor enabled
- Bit 1: Right motor enabled  
- Bit 2: Emergency stop active
- Bit 3: Reserved
- Bits 4-7: Reserved

Linux I2C Testing:
------------------
# Verify ESP32 is detected
i2cdetect -y 3

# Set motors to 50% forward, servo centered
i2cset -y 3 0x42 0x01 50 50 90 i

# Set motors to 30% forward, servo left (45°)
i2cset -y 3 0x42 0x01 30 30 45 i

# Emergency stop
i2cset -y 3 0x42 0x02

# Request status
i2cset -y 3 0x42 0x03

ESP32 Firmware Requirements:
----------------------------
CRITICAL: Do NOT use Arduino Wire library for I2C slave mode - it's broken!

Use ESP-IDF I2C driver directly:
  #include "driver/i2c.h"
  
  i2c_config_t conf;
  conf.mode = I2C_MODE_SLAVE;
  conf.sda_io_num = GPIO_NUM_21;
  conf.scl_io_num = GPIO_NUM_22;
  conf.slave.slave_addr = 0x42;
  // ... configure and install driver
  
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE, RX_BUF, TX_BUF, 0);
  
  // Read/write with:
  i2c_slave_read_buffer(I2C_NUM_0, buffer, len, timeout);
  i2c_slave_write_buffer(I2C_NUM_0, response, len, timeout);

ROS2 Integration:
-----------------
The ros2_motor_controller_i2c.py uses smbus2 Python library:
  pip3 install smbus2

Advantages:
-----------
- Clean bus (I2C-3) with no existing devices
- Hardware already configured and tested
- Built-in error detection and acknowledgment  
- Multiple device support possible
- Standard protocol with excellent Linux support
- ESP-IDF API provides reliable slave operation

Troubleshooting:
----------------
1. ESP32 not detected (no 42 in i2cdetect):
   - Check wiring: SDA=GPIO21->Pin20, SCL=GPIO22->Pin18
   - Verify common ground connection
   - Check ESP32 is powered and running firmware
   - Monitor ESP32 serial output at 115200 baud

2. "i2c driver install error" on ESP32:
   - You're using Arduino Wire library - switch to ESP-IDF API!
   - See DartMX95/src/main.cpp for working implementation

3. Communication errors:
   - Ensure firmware uses i2c_slave_read_buffer() not Wire.read()
   - Check ESP32 serial monitor for activity
