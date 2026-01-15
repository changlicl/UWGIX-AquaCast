# AquaCast Quick Start Guide

Get up and running with AquaCast firmware in 15 minutes!

## 📋 Prerequisites

- ESP32-S3 development board
- USB cable (USB-C or Micro-USB depending on board)
- Computer with Arduino IDE or PlatformIO installed
- Internet connection for library downloads

## ⚡ 5-Minute Test (No Hardware)

Test the firmware compilation without hardware:

```bash
# Clone repository
git clone https://github.com/your-org/aquacast-firmware.git
cd aquacast-firmware

# Option 1: Arduino IDE
# Open firmware/main/main.ino
# Select Board: ESP32S3 Dev Module
# Click Verify (✓) button

# Option 2: PlatformIO
cd firmware/main
pio run
```

**Success**: No compilation errors ✓

## 🔌 Hardware Setup (15 minutes)

### Minimal Test Setup

You need only:
1. ESP32-S3 board
2. USB cable
3. Computer

**Steps:**
1. Connect ESP32 to computer via USB
2. Install USB drivers if needed (CP210x or CH340)
3. Upload firmware (see below)
4. Open Serial Monitor @ 115200 baud
5. You should see system startup messages!

### With Basic Sensors (30 minutes)

Add depth sensor for full testing:

```
ESP32 Pin 21 (SDA) ──── Depth Sensor SDA
ESP32 Pin 22 (SCL) ──── Depth Sensor SCL
ESP32 3.3V ─────────── Depth Sensor VCC
ESP32 GND ──────────── Depth Sensor GND

Add 4.7kΩ resistors:
- GPIO21 to 3.3V (pull-up)
- GPIO22 to 3.3V (pull-up)
```

## 📤 Upload Firmware

### Arduino IDE

1. **Install ESP32 Board Support:**
   - File → Preferences
   - Additional Board Manager URLs: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Install Libraries:**
   - Sketch → Include Library → Manage Libraries
   - Install: TinyGPS++, ArduinoJson
   
3. **Configure Board:**
   - Tools → Board → ESP32 Arduino → ESP32S3 Dev Module
   - Tools → Port → Select your COM port
   - Tools → Upload Speed → 921600

4. **Upload:**
   - Open `firmware/main/main.ino`
   - Click Upload (→) button
   - Wait for "Done uploading"

5. **Open Serial Monitor:**
   - Tools → Serial Monitor
   - Set baud rate to 115200
   - You should see boot messages!

### PlatformIO

```bash
cd firmware/main

# Upload firmware
pio run -t upload

# Open serial monitor
pio device monitor -b 115200
```

## 🎮 Test Commands

Once firmware is running, try these commands in Serial Monitor:

```
HELP      - Show available commands
STATUS    - Display system status
START     - Start mission simulation
TRIGGER1  - Manually trigger chamber 1
RESET     - Restart device
```

## 🧪 Run Tests

### I2C Scanner Test

```bash
# Upload test sketch
cd firmware/test/i2c_scanner
arduino-cli compile --fqbn esp32:esp32:esp32s3 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 .

# Open serial monitor
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Should display detected I2C devices.

## 🐛 Troubleshooting

### Upload Fails

**Error: "Serial port not found"**
- Check USB cable (must be data cable, not charge-only)
- Install drivers: [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [CH340](https://sparks.gogo.co.nz/ch340.html)
- Try different USB port
- Check Device Manager (Windows) or `ls /dev/tty*` (Mac/Linux)

**Error: "Timed out waiting for packet header"**
- Hold BOOT button on ESP32 while clicking Upload
- Check baud rate (try 115200 instead of 921600)
- Try different USB cable

### Compilation Fails

**Error: "Wire.h: No such file"**
- Reinstall ESP32 board support

**Error: "TinyGPS++ not found"**
- Install missing library via Library Manager

### No Serial Output

- Check baud rate is 115200
- Press RESET button on ESP32
- Check Serial Monitor is on correct port

## 📚 Next Steps

1. **Read Documentation:**
   - [Firmware README](firmware/README.md)
   - [Hardware Guide](hardware/README.md)
   - [API Documentation](docs/api/)

2. **Try Examples:**
   - GPS test: `firmware/test/test_gps/`
   - Sensor test: `firmware/test/test_sensors/`

3. **Configure Mission:**
   - Edit `config/mission-templates/shallow-survey.json`
   - Upload via Serial or SD card

4. **Join Development:**
   - Read [CONTRIBUTING.md](CONTRIBUTING.md)
   - Check GitHub Issues for tasks

## 🆘 Get Help

**Still stuck?** 

1. Check [GitHub Issues](https://github.com/your-org/aquacast-firmware/issues)
2. Review [Troubleshooting Guide](docs/troubleshooting.md)
3. Contact team:
   - **Firmware**: Victoria Yang
   - **Hardware**: Joyce Chou, Chang Li

## 📞 Support Channels

- 💬 GitHub Discussions: For questions
- 🐛 GitHub Issues: For bugs
- 📧 Email: Contact course instructor for urgent issues

---

**Estimated Time to First Boot**: ~15 minutes  
**Last Updated**: January 14, 2025

🎉 **Welcome to AquaCast!** Happy coding!
