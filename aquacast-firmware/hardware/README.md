# AquaCast Hardware Documentation

## 📋 Overview

This directory contains all hardware documentation for the AquaCast water sampling device, including wiring diagrams, schematics, bill of materials, and datasheets.

## 📁 Directory Contents

```
hardware/
├── README.md (this file)
├── bom.csv                    # Bill of Materials
├── wiring-diagram.png         # Fritzing wiring diagram
├── schematics/                # Circuit schematics
│   └── esp32-sensor-connections.pdf
└── datasheets/                # Component datasheets
    ├── esp32-s3-nodemcu.pdf
    ├── bar30-depth-sensor.pdf
    └── neo6m-gps.pdf
```

## 🔌 Pin Connections

### ESP32-S3 NodeMCU Pin Mapping

| Function | Pin | Connection | Notes |
|----------|-----|------------|-------|
| **I2C Bus** |
| I2C SDA | GPIO 21 | Depth sensor data | 4.7kΩ pull-up to 3.3V |
| I2C SCL | GPIO 22 | Depth sensor clock | 4.7kΩ pull-up to 3.3V |
| **GPS (UART2)** |
| GPS RX | GPIO 16 | ESP32 RX ← GPS TX | 9600 baud |
| GPS TX | GPIO 17 | ESP32 TX → GPS RX | 9600 baud |
| **Analog Sensors** |
| Turbidity | GPIO 34 (ADC1_CH6) | Analog input | 0-3.3V |
| pH Sensor | GPIO 35 (ADC1_CH7) | Analog input | 0-3.3V |
| Battery Monitor | GPIO 39 (ADC1_CH3) | Voltage divider | 2:1 divider |
| **Actuation** |
| Chamber 1 | GPIO 25 | Servo/magnet driver | PWM capable |
| Chamber 2 | GPIO 26 | Servo/magnet driver | PWM capable |
| Chamber 3 | GPIO 27 | Reserved (future) | PWM capable |
| Chamber 4 | GPIO 14 | Reserved (future) | PWM capable |
| **Confirmation Sensors** |
| Chamber 1 Confirm | GPIO 32 | Hall sensor / limit switch | Active LOW, pull-up |
| Chamber 2 Confirm | GPIO 33 | Hall sensor / limit switch | Active LOW, pull-up |
| **Status Indicators** |
| Status LED | GPIO 2 | Built-in LED | Active HIGH |
| GPS LED | GPIO 4 | External LED | Active HIGH |
| Error LED | GPIO 5 | External LED | Active HIGH |
| **Storage (Optional)** |
| SD Card CS | GPIO 5 | SPI chip select | If using SD logging |

## 📦 Bill of Materials (BOM)

### Core Components

| Part | Description | Quantity | Source | Price (USD) |
|------|-------------|----------|--------|-------------|
| ESP32-S3 NodeMCU-32S | Microcontroller dev board | 1 | Amazon/AliExpress | $12 |
| Bar30 Pressure Sensor | High-res depth/pressure sensor | 1 | Blue Robotics | $68 |
| I2C Level Converter | 5V↔3.3V level shifter (if needed) | 1 | SparkFun | $3 |
| NEO-6M GPS Module | GPS receiver with antenna | 1 | Amazon | $15 |
| Turbidity Sensor V1.0 | Analog turbidity sensor | 1 | DFRobot | $10 |
| pH Sensor Kit | Analog pH sensor | 1 | Atlas Scientific | $60 |

### Actuation Components (Magnetic Option)

| Part | Description | Quantity | Price |
|------|-------------|----------|-------|
| N52 Neodymium Magnets | 10mm x 3mm disc magnets | 8 | $12 |
| Compression Springs | Return spring for lids | 4 | $5 |
| Hall Effect Sensors | Magnetic field detector | 2 | $2 |

### Actuation Components (Servo Option)

| Part | Description | Quantity | Price |
|------|-------------|----------|-------|
| Waterproof Servo Motors | Corrosion-resistant servos | 2 | $40 |
| Servo Linkages | Mechanical linkage hardware | 4 | $8 |

### Power & Regulation

| Part | Description | Quantity | Price |
|------|-------------|----------|-------|
| LiPo Battery 3.7V 2000mAh | Rechargeable battery | 1 | $15 |
| TP4056 Charger Module | Li-ion battery charger | 1 | $2 |
| Voltage Regulator 3.3V | LDO regulator for sensors | 1 | $3 |

### Enclosure & Waterproofing

| Part | Description | Quantity | Price |
|------|-------------|----------|-------|
| Waterproof Case | Polycarbonate housing | 1 | $25 |
| O-Rings | Multiple sizes | 10 | $10 |
| Silicone Sealant | Marine-grade sealant | 1 tube | $8 |
| Cable Glands | Waterproof cable entry | 4 | $12 |

### Miscellaneous

| Part | Description | Quantity | Price |
|------|-------------|----------|-------|
| Jumper Wires | Male-male, male-female | 1 set | $5 |
| Breadboard | Half-size breadboard | 1 | $4 |
| Resistors (4.7kΩ) | I2C pull-up resistors | 2 | $0.20 |
| Capacitors (470µF) | Power supply decoupling | 2 | $1 |
| Heat Shrink Tubing | Various sizes | 1 set | $8 |

**Total Estimated Cost**: ~$328 (Magnetic) / ~$356 (Servo)

## 🔧 Assembly Instructions

### 1. I2C Sensor Connection (Joyce & Chang)

**Depth/Pressure Sensor Setup:**

1. Solder I2C level converter module (if sensor is 5V)
2. Connect converter to depth sensor:
   - VCC → 5V (or 3.3V if sensor supports it)
   - GND → GND
   - SDA → Converter HV SDA
   - SCL → Converter HV SCL

3. Connect converter to ESP32:
   - Converter LV SDA → GPIO 21
   - Converter LV SCL → GPIO 22
   - LV VCC → 3.3V
   - LV GND → GND

4. Add 4.7kΩ pull-up resistors:
   - One resistor: GPIO 21 (SDA) → 3.3V
   - One resistor: GPIO 22 (SCL) → 3.3V

**Verification:**
- Upload `firmware/test/i2c_scanner/` sketch
- Open serial monitor @ 115200 baud
- Should detect device at address 0x76

### 2. GPS Module Connection (Joyce)

1. Connect GPS module to ESP32:
   - GPS VCC → 3.3V
   - GPS GND → GND
   - GPS TX → GPIO 16 (ESP32 RX)
   - GPS RX → GPIO 17 (ESP32 TX)

2. Ensure antenna has clear view of sky

**Verification:**
- Upload `firmware/test/test_gps/` sketch
- Take device outdoors
- Wait ~30 seconds for cold start
- Should see NMEA sentences in serial monitor

### 3. Actuation Mechanism (Shareef)

**Magnetic Actuation:**
- Mount external magnet on servo-driven slider
- Internal magnet attached to chamber lid
- Hall effect sensor detects when lid opens
- See `mechanical/assembly-guide.pdf` for details

**Servo Actuation:**
- Mount waterproof servo in external housing
- Mechanical linkage connects to chamber lid
- Limit switch confirms lid position

## 🧪 Testing Procedures

### Component-Level Testing

1. **I2C Bus Test**: Run I2C scanner, verify depth sensor address
2. **Depth Sensor Test**: Submerge sensor in water at known depths
3. **GPS Test**: Verify fix acquisition time and accuracy
4. **Actuation Test**: Trigger chambers 10 times, check reliability

### Integration Testing

1. **Waterproof Test**: Seal device, submerge in bathtub for 60 seconds
2. **Mission Simulation**: Run full mission profile in controlled environment
3. **Field Test**: Deploy in shallow water body (0.5-2m depth)

## 📸 Photos & Diagrams

*(To be added: Photos of actual hardware setup, wiring closeups)*

## ⚠️ Safety Notes

- **Battery**: Never short-circuit LiPo battery. Use fireproof charging bag.
- **Water Exposure**: Test waterproofing thoroughly before deploying in deep water
- **Depth Rating**: Current design rated for max 5m depth
- **Sensor Calibration**: Recalibrate depth sensor at deployment site

## 🔍 Troubleshooting

**I2C sensor not detected:**
- Check pull-up resistors are installed
- Verify wiring with multimeter (continuity test)
- Try lowering I2C clock speed to 100kHz

**GPS no fix:**
- Move outdoors with clear sky view
- Wait longer (cold start can take 60+ seconds)
- Check UART connections and baud rate

**Actuation failure:**
- Verify GPIO pin assignments in `config.h`
- Check power supply can provide sufficient current
- Test actuator independently with separate sketch

## 📞 Support

- **Hardware Questions**: Joyce Chou, Chang Li
- **Firmware Integration**: Victoria Yang
- **Mechanical Issues**: Shareef Jasim

---

**Last Updated**: January 14, 2025  
**Hardware Version**: v0.1.0
