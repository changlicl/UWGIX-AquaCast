# AquaCast Firmware

**Autonomous Drone-Deployable Water Sampling Device for Near-Shore Ocean Monitoring**

![Project Status](https://img.shields.io/badge/status-gate%207%20development-blue)
![License](https://img.shields.io/badge/license-MIT-green)
![Hardware](https://img.shields.io/badge/hardware-ESP32--S3-orange)

## 📋 Project Overview

AquaCast is a drone-deployable autonomous water sampling device designed for near-shore ocean monitoring. The system submerges to preset depths, independently triggers multiple sampling chambers, collects sensor data, and resurfaces for GPS-guided retrieval.

**Course**: TECHIN 540 AU 2025 - Integrated Launch Studio I  
**Team**: Shareef Jasim, Joyce Chou, Chang Li, Victoria Yang  
**Institution**: University of Washington - Global Innovation Exchange

### Key Features
- ✅ Multi-depth autonomous sampling (0.5m, 2m target depths)
- ✅ High-resolution depth/pressure sensing (±0.01m accuracy)
- ✅ GPS positioning with metadata logging
- ✅ Multi-sensor integration (temperature, turbidity, pH, salinity)
- ✅ Waterproof operation to 5m depth
- ✅ Mission configuration via dashboard interface

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    AquaCast System                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐      ┌──────────────┐     ┌────────────┐ │
│  │   Sensors    │─────▶│   ESP32-S3   │────▶│  Dashboard │ │
│  │              │      │   Firmware   │     │     UI     │ │
│  │ • Depth/Pres │      │              │     │            │ │
│  │ • GPS        │      │ • Data Log   │     │ • Config   │ │
│  │ • Turbidity  │      │ • Actuation  │     │ • Viz      │ │
│  │ • pH/Salin   │      │ • Mission Ctl│     │ • Export   │ │
│  └──────────────┘      └──────────────┘     └────────────┘ │
│         │                      │                    │       │
│         └──────────────────────┴────────────────────┘       │
│                          I2C / UART / GPIO                  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │          Actuation Mechanism (4 chambers)            │  │
│  │     Magnetic repulsion OR servo-based triggering     │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## 📁 Repository Structure

```
aquacast-firmware/
├── README.md                    # This file
├── LICENSE                      # MIT License
├── .gitignore                   # Git ignore rules
│
├── firmware/                    # ESP32 firmware source code
│   ├── main/                    # Main application
│   │   ├── main.ino            # Arduino sketch entry point
│   │   ├── config.h            # Configuration constants
│   │   ├── sensors.h/.cpp      # Sensor interface layer
│   │   ├── actuation.h/.cpp    # Chamber actuation control
│   │   ├── datalogger.h/.cpp   # Data logging and export
│   │   └── mission.h/.cpp      # Mission state machine
│   │
│   ├── lib/                     # External libraries (gitignored)
│   └── test/                    # Unit tests
│       ├── test_sensors.ino
│       └── test_actuation.ino
│
├── hardware/                    # Hardware documentation
│   ├── README.md               # Hardware overview
│   ├── bom.csv                 # Bill of Materials
│   ├── wiring-diagram.png      # Fritzing diagram
│   ├── schematics/             # Circuit schematics
│   │   └── esp32-sensor-connections.pdf
│   └── datasheets/             # Component datasheets
│       ├── esp32-s3-nodemcu.pdf
│       ├── bar30-depth-sensor.pdf
│       └── neo6m-gps.pdf
│
├── mechanical/                  # Mechanical design files
│   ├── README.md               # Mechanical overview
│   ├── cad/                    # Rhino 3D models
│   │   ├── enclosure-v1.3dm
│   │   └── actuation-mechanism.3dm
│   ├── stl/                    # 3D printable files
│   │   ├── lid-assembly.stl
│   │   └── mounting-bracket.stl
│   └── assembly-guide.pdf      # Assembly instructions
│
├── dashboard/                   # Mission control dashboard
│   ├── README.md               # Dashboard documentation
│   ├── index.html              # Web interface
│   ├── app.js                  # Dashboard logic
│   └── styles.css              # UI styling
│
├── docs/                        # Project documentation
│   ├── gate-07-submission.md   # Gate 7 deliverables
│   ├── system-architecture.md  # Detailed architecture
│   ├── integration-plan.md     # Integration specifications
│   ├── testing-strategy.md     # Testing procedures
│   └── api/                    # Interface specifications
│       ├── sensor-interface.md
│       └── data-format.md
│
├── config/                      # Configuration files
│   ├── mission-templates/      # Example mission configs
│   │   ├── shallow-survey.json
│   │   └── multi-depth-sample.json
│   └── sensor-calibration/     # Calibration data
│       └── depth-sensor-cal.json
│
├── data/                        # Example data (gitignored)
│   └── sample-mission-2025-01-09.json
│
└── scripts/                     # Utility scripts
    ├── setup-arduino-env.sh    # Development environment setup
    ├── flash-firmware.sh       # Firmware upload script
    └── parse-logs.py           # Data processing utilities
```

## 🚀 Quick Start

### Prerequisites
- **Hardware**: ESP32-S3 NodeMCU-32S development board
- **Software**: Arduino IDE 2.3+ or PlatformIO
- **Libraries**: See [firmware/README.md](firmware/README.md)

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/aquacast-firmware.git
cd aquacast-firmware

# Install Arduino libraries
# (See firmware/README.md for detailed library installation)

# Open firmware in Arduino IDE
arduino-ide firmware/main/main.ino

# Or use PlatformIO
pio run -t upload
```

### First-Time Setup

1. **Hardware Assembly**: Follow [hardware/README.md](hardware/README.md)
2. **Sensor Calibration**: Run calibration sketch in `firmware/test/`
3. **Mission Configuration**: Edit `config/mission-templates/shallow-survey.json`
4. **Deploy Firmware**: Upload to ESP32 via USB

## 📊 Current Development Status

### ✅ Completed (Gate 7 - Week 1)
- [x] Subsystem architecture design
- [x] I2C depth/pressure sensor integration
- [x] GPS module UART communication
- [x] Basic data logging framework
- [x] Magnetic actuation prototype

### 🚧 In Progress (Week 2-3)
- [ ] Multi-chamber actuation control
- [ ] Turbidity and pH sensor integration
- [ ] Mission state machine implementation
- [ ] Dashboard UI development

### 📅 Upcoming (Week 4+)
- [ ] Waterproof enclosure final design
- [ ] Field testing in controlled water environment
- [ ] Drone integration and retrieval mechanism
- [ ] Full end-to-end mission validation

## 🛠️ Development Workflow

### Team Responsibilities
- **Victoria Yang**: Firmware architecture, sensor integration, data logging
- **Joyce Chou**: GPS integration, sensor validation, system testing
- **Chang Li**: I2C hardware setup, GitHub documentation, hardware diagrams
- **Shareef Jasim**: Mechanical design, actuation mechanism, waterproofing

### Contributing Guidelines

1. **Branch naming**: `feature/sensor-gps`, `fix/i2c-timeout`, `docs/hardware-guide`
2. **Commit messages**: Use conventional commits (e.g., `feat: add GPS parsing`, `fix: I2C address conflict`)
3. **Pull requests**: Require review from at least one team member
4. **Testing**: All sensor code must pass unit tests before merging

## 📖 Documentation

- **[Hardware Documentation](hardware/README.md)**: Wiring diagrams, BOM, datasheets
- **[Firmware API](docs/api/)**: Sensor interfaces and data formats
- **[System Architecture](docs/system-architecture.md)**: Detailed subsystem breakdown
- **[Testing Guide](docs/testing-strategy.md)**: Testing procedures and validation

## 🔧 Troubleshooting

### Common Issues

**I2C sensor not detected**
```bash
# Run I2C scanner
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/test/i2c_scanner/
# Check pull-up resistors (4.7kΩ recommended)
```

**GPS no fix**
- Ensure outdoor testing with clear sky view
- Check UART baud rate (9600 for NEO-6M)
- Verify antenna connection

**Data logging fails**
- Check SD card formatting (FAT32)
- Verify SPI pin connections
- Fallback: Use serial logging

See [docs/troubleshooting.md](docs/troubleshooting.md) for detailed solutions.

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file.

## 🙏 Acknowledgments

- **Instructors**: TECHIN 540 teaching team
- **Partners**: UW Friday Harbor Labs, King County Environmental Monitoring
- **Libraries**: TinyGPS++, Adafruit Sensor Library, Wire.h

## 📞 Contact

- **Project Lead**: Victoria Yang - [GitHub](https://github.com/victoria-yang)
- **Hardware Lead**: Joyce Chou & Chang Li
- **Mechanical Lead**: Shareef Jasim
- **Course**: TECHIN 540, University of Washington GIX

---

**Last Updated**: January 14, 2025  
**Version**: 0.1.0 (Gate 7 Development)
