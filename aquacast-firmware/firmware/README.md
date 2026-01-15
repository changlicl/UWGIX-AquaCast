# AquaCast Firmware

ESP32-S3 firmware for the AquaCast autonomous water sampling device.

## 📋 Overview

This firmware implements the complete mission control logic for AquaCast, including:
- Multi-sensor data acquisition (depth, GPS, temperature, turbidity, pH)
- Depth-triggered chamber actuation
- Real-time data logging with timestamps
- Mission configuration management
- System health monitoring

## 🔧 Hardware Requirements

### Microcontroller
- **Board**: ESP32-S3 NodeMCU-32S
- **Flash**: 4MB minimum
- **RAM**: 512KB SRAM
- **Clock**: 240MHz dual-core

### Sensors
- **Depth/Pressure**: BlueRobotics Bar30 High-Resolution Sensor (I2C, 0x76)
- **GPS**: NEO-6M GPS Module (UART, 9600 baud)
- **Temperature**: Integrated with depth sensor
- **Turbidity**: Analog Turbidity Sensor V1.0 (ADC)
- **pH**: Analog pH Sensor (ADC, optional)
- **Salinity**: Analog Salinity Sensor (ADC, future)

### Actuators
- **Chamber Mechanism**: Servo motor or magnetic actuation
- **Confirmation Sensor**: Hall effect sensor or limit switch

## 📦 Required Libraries

Install via Arduino Library Manager or PlatformIO:

```ini
# platformio.ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

lib_deps =
    Wire @ ^1.0                           # I2C communication
    TinyGPSPlus @ ^1.0.3                 # GPS parsing
    ArduinoJson @ ^6.21.0                # JSON config/logging
    ESP32Servo @ ^0.13.0                 # Servo control (if using servo actuation)
```

### Manual Installation
If using Arduino IDE, install these libraries:

1. **TinyGPS++**: `Sketch → Include Library → Manage Libraries → Search "TinyGPS++"`
2. **ArduinoJson**: `Sketch → Include Library → Manage Libraries → Search "ArduinoJson"`
3. **ESP32Servo** (if needed): `Sketch → Include Library → Manage Libraries → Search "ESP32Servo"`

## 🚀 Compilation & Upload

### Arduino IDE

1. Open `firmware/main/main.ino`
2. Select **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**
3. Configure board settings:
   - Upload Speed: 921600
   - USB Mode: Hardware CDC and JTAG
   - Flash Size: 4MB
4. Click **Upload** (Ctrl+U)

### PlatformIO

```bash
# Compile
pio run

# Upload to board
pio run -t upload

# Open serial monitor
pio device monitor -b 115200
```

### Command Line (arduino-cli)

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/main/

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/main/

# Monitor serial output
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

## 📝 Configuration

### Mission Configuration

Edit `config/mission-templates/shallow-survey.json`:

```json
{
  "mission_id": "AQ_20250114_001",
  "device_id": "ESP32_DEV_01",
  "sampling_depths": [0.5, 2.0],
  "sampling_rate_hz": 1.0,
  "sensors": {
    "gps": true,
    "depth": true,
    "temperature": true,
    "turbidity": false,
    "ph": false,
    "salinity": false
  },
  "actuation": {
    "type": "magnetic",
    "confirmation_timeout_ms": 2000,
    "retry_attempts": 3
  }
}
```

Upload via Serial Monitor or SD card.

### Pin Configuration

Edit `firmware/main/config.h`:

```cpp
// I2C Pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// UART GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Analog Sensors
#define TURBIDITY_PIN 34
#define PH_PIN 35

// Actuation
#define CHAMBER_1_PIN 25
#define CHAMBER_2_PIN 26
#define CHAMBER_3_PIN 27
#define CHAMBER_4_PIN 14

// Status LED
#define LED_PIN 2
```

## 🧪 Testing

### Unit Tests

```bash
# Test I2C sensor communication
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/test/test_sensors/
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/test/test_sensors/

# Test GPS parsing
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/test/test_gps/
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/test/test_gps/

# Test actuation mechanism
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/test/test_actuation/
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/test/test_actuation/
```

### Integration Test

Run full mission simulation:

```bash
# Upload main firmware
pio run -t upload

# Open serial monitor
pio device monitor

# Send test commands via serial:
# > START_MISSION
# > TRIGGER_CHAMBER_1
# > EXPORT_DATA
```

## 📊 Data Format

### Sensor Data Log (JSON)

```json
{
  "timestamp": "2025-01-14T14:23:45Z",
  "mission_time_ms": 45320,
  "depth_m": 0.52,
  "temperature_c": 12.3,
  "pressure_mbar": 1025.4,
  "gps": {
    "latitude": 47.606209,
    "longitude": -122.332071,
    "fix_quality": 1,
    "satellites": 8,
    "hdop": 1.2
  },
  "turbidity_ntu": 15.3,
  "health_flags": 0b11110000
}
```

### Health Flags (Byte)
- Bit 7: GPS lock status (1 = locked)
- Bit 6: I2C communication OK
- Bit 5: SD card write OK
- Bit 4: Battery voltage OK (>3.0V)
- Bits 0-3: Reserved

## 🐛 Debugging

### Enable Debug Output

In `config.h`:

```cpp
#define DEBUG_MODE 1
#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 115200
```

### Common Issues

**"I2C device not found at 0x76"**
- Check wiring: SDA → GPIO21, SCL → GPIO22
- Verify pull-up resistors (4.7kΩ)
- Run I2C scanner: `firmware/test/i2c_scanner/`

**"GPS timeout - no fix"**
- Must be outdoors with clear sky view
- Wait 30-60s for cold start
- Check UART connections: RX → GPIO16, TX → GPIO17
- Verify baud rate: 9600

**"SD card initialization failed"**
- Format as FAT32
- Check SPI connections
- Try lower SPI speed: `SD.begin(SD_CS_PIN, SPI, 4000000)`

## 📁 File Structure

```
firmware/
├── main/
│   ├── main.ino              # Entry point, setup() and loop()
│   ├── config.h              # Pin definitions and constants
│   ├── sensors.h/.cpp        # Sensor interface abstraction
│   ├── actuation.h/.cpp      # Chamber actuation control
│   ├── datalogger.h/.cpp     # Data logging and export
│   └── mission.h/.cpp        # Mission state machine
├── test/
│   ├── test_sensors/         # Sensor unit tests
│   ├── test_actuation/       # Actuation tests
│   └── i2c_scanner/          # I2C address scanner
└── lib/                      # External libraries (auto-installed)
```

## 🔄 State Machine

```
IDLE → MISSION_LOADED → DESCENT → SAMPLING → SURFACING → DATA_READY → IDLE
  ↑                                    ↓
  └──────────────── ERROR ─────────────┘
```

### State Descriptions
- **IDLE**: Waiting for mission configuration
- **MISSION_LOADED**: Config validated, ready to deploy
- **DESCENT**: Device submerging, monitoring depth
- **SAMPLING**: Triggering chambers at target depths
- **SURFACING**: Ascending, activating GPS beacon
- **DATA_READY**: Mission complete, ready for retrieval
- **ERROR**: Fault detected, logging diagnostics

## 📞 Support

- **Issues**: Report bugs via GitHub Issues
- **Hardware Questions**: Contact Joyce Chou / Chang Li
- **Firmware Questions**: Contact Victoria Yang

## 📄 Version History

- **v0.1.0** (2025-01-14): Gate 7 development - basic sensor integration
- **v0.2.0** (TBD): Multi-chamber actuation
- **v1.0.0** (TBD): Production release

---

**Last Updated**: January 14, 2025
