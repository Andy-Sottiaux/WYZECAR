#!/bin/bash
# ESP32 Flashing Script for WYZECAR Motor Control Firmware
# Run this script to flash your ESP32 with the motor control firmware

echo "===========================================" 
echo "  WYZECAR ESP32 Motor Controller Flash"
echo "==========================================="
echo ""

# Check if ESP32 is connected
echo "Checking for connected ESP32..."
if ls /dev/cu.usbserial* 1> /dev/null 2>&1; then
    echo "✓ ESP32 device found: $(ls /dev/cu.usbserial*)"
elif ls /dev/cu.SLAB_USBtoUART* 1> /dev/null 2>&1; then
    echo "✓ ESP32 device found: $(ls /dev/cu.SLAB_USBtoUART*)"
elif ls /dev/cu.usbmodem* 1> /dev/null 2>&1; then
    echo "✓ ESP32 device found: $(ls /dev/cu.usbmodem*)"
else
    echo "❌ ESP32 not detected. Please:"
    echo "   1. Connect ESP32 via USB"
    echo "   2. Install USB-to-Serial drivers if needed"
    echo "   3. Try different USB cable"
    echo "   4. Hold BOOT button while connecting"
    exit 1
fi

echo ""
echo "Building firmware..."
pio run

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
    echo ""
    echo "Flashing ESP32..."
    echo "Note: If flashing fails, hold BOOT button and press EN button on ESP32"
    
    pio run --target upload
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 ESP32 flashed successfully!"
        echo ""
        echo "Next steps:"
        echo "1. Connect ESP32 to DART-MX95 via J6 header"
        echo "2. Connect L298N motor driver as per WIRING.txt"
        echo "3. Test communication with ROS2 motor controller"
    else
        echo ""
        echo "❌ Flash failed. Try:"
        echo "1. Hold BOOT button while flashing"
        echo "2. Press EN button to reset"
        echo "3. Check USB connection"
        echo "4. Use different USB port/cable"
    fi
else
    echo "❌ Build failed. Check source code for errors."
fi

echo ""
echo "To monitor ESP32 serial output:"
echo "pio device monitor"