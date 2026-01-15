# Contributing to AquaCast Firmware

Thank you for contributing to the AquaCast project! This guide will help you get started with development and collaboration.

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/aquacast-firmware.git
cd aquacast-firmware
```

### 2. Set Up Development Environment

**Arduino IDE:**
```bash
# Install Arduino IDE 2.3+
# Add ESP32 board support:
# File → Preferences → Additional Board Manager URLs
https://espressif.github.io/arduino-esp32/package_esp32_index.json

# Install required libraries:
# Sketch → Include Library → Manage Libraries
# Search and install: TinyGPS++, ArduinoJson, ESP32Servo
```

**PlatformIO (Recommended):**
```bash
# Install PlatformIO CLI
pip install platformio

# Initialize project
cd firmware/main
pio lib install

# Build
pio run

# Upload
pio run -t upload
```

## 🏗️ Project Structure

```
aquacast-firmware/
├── firmware/          # ESP32 source code (Victoria)
├── hardware/          # Wiring diagrams, BOM (Joyce, Chang)
├── mechanical/        # CAD files, STLs (Shareef)
├── docs/              # Documentation
└── config/            # Mission configuration files
```

## 👥 Team Responsibilities

| Team Member | Role | Responsibilities |
|-------------|------|------------------|
| **Victoria Yang** | Firmware Lead | Core firmware, sensor integration, mission logic |
| **Joyce Chou** | Hardware Integration | GPS, sensor testing, system validation |
| **Chang Li** | Hardware Documentation | I2C setup, GitHub docs, wiring diagrams |
| **Shareef Jasim** | Mechanical Lead | Enclosure design, actuation mechanism |

## 🔄 Development Workflow

### Branching Strategy

```
main                    # Production-ready code
├── develop            # Integration branch
├── feature/gps-integration
├── feature/multi-chamber
├── fix/i2c-timeout
└── docs/hardware-guide
```

**Branch Naming Convention:**
- `feature/` - New features (e.g., `feature/ph-sensor`)
- `fix/` - Bug fixes (e.g., `fix/depth-calibration`)
- `docs/` - Documentation updates (e.g., `docs/api-reference`)
- `test/` - Testing improvements (e.g., `test/actuation-bench`)

### Commit Message Format

Use **Conventional Commits**:

```
type(scope): description

[optional body]

[optional footer]
```

**Examples:**
```
feat(sensors): add turbidity sensor support
fix(gps): resolve timeout during cold start
docs(hardware): update wiring diagram for I2C
test(actuation): add chamber trigger unit test
refactor(logging): simplify JSON export logic
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting (no code change)
- `refactor`: Code restructuring
- `test`: Adding tests
- `chore`: Maintenance tasks

### Making Changes

1. **Create a branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes:**
   - Write code following style guidelines
   - Add comments and documentation
   - Test thoroughly

3. **Commit your changes:**
   ```bash
   git add .
   git commit -m "feat(sensors): add pH sensor calibration"
   ```

4. **Push to GitHub:**
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Create a Pull Request:**
   - Go to GitHub repository
   - Click "New Pull Request"
   - Select your branch
   - Fill in PR template
   - Request review from team member

### Pull Request Guidelines

**Before submitting:**
- [ ] Code compiles without errors
- [ ] All tests pass
- [ ] Documentation updated
- [ ] Comments added for complex logic
- [ ] Tested on hardware (if applicable)

**PR Title Format:**
```
[Type] Brief description

Examples:
[Feature] Add multi-chamber actuation support
[Fix] Resolve GPS timeout during initialization
[Docs] Update hardware assembly guide
```

**PR Description Template:**
```markdown
## Description
Brief description of changes

## Changes Made
- Added X feature
- Fixed Y bug
- Updated Z documentation

## Testing
- [ ] Compiled successfully
- [ ] Tested on ESP32 hardware
- [ ] Unit tests pass
- [ ] Integration tests pass

## Related Issues
Closes #123

## Screenshots/Logs
(If applicable)
```

## 🧪 Testing Requirements

### Before Committing

1. **Compilation Test:**
   ```bash
   pio run  # or arduino-cli compile
   ```

2. **Unit Tests:**
   ```bash
   pio test
   ```

3. **Hardware Test** (if code affects hardware):
   - Upload to ESP32
   - Run serial monitor test
   - Verify sensor readings

### Test Coverage

All new features should include:
- Unit tests for functions
- Integration test sketch
- Hardware validation procedure

## 📝 Code Style Guidelines

### C++ Style (Firmware)

```cpp
// Header guard
#ifndef FILE_NAME_H
#define FILE_NAME_H

// Includes grouped
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

// Constants uppercase
#define MAX_DEPTH_M 5.0

// Functions: camelCase
bool readDepthSensor(DepthData* data);

// Classes: PascalCase
class SensorManager {
private:
  float depth_m_;  // Member variables with trailing underscore
  
public:
  bool begin();
};

// Comments: Explain WHY, not WHAT
// Use low-pass filter to reduce sensor noise
float filtered = (alpha * raw) + ((1 - alpha) * last);
```

### Documentation

- Add function documentation:
```cpp
/**
 * @brief Read depth sensor via I2C
 * @param data Pointer to DepthData structure to fill
 * @return true if read successful, false otherwise
 */
bool readDepthSensor(DepthData* data);
```

- Add file headers:
```cpp
/**
 * @file sensors.cpp
 * @brief Sensor interface implementation
 * @author Victoria Yang
 * @date 2025-01-14
 */
```

## 🐛 Debugging Tips

### Serial Debugging

```cpp
#if DEBUG_MODE
  DEBUG_PRINTLN("Sensor reading:");
  DEBUG_PRINTF("Depth: %.2fm\n", depth);
#endif
```

### Common Issues

**I2C not working:**
- Check wiring: SDA/SCL correct?
- Pull-up resistors installed?
- Run I2C scanner test

**GPS no fix:**
- Must be outdoors
- Wait 30-60 seconds
- Check baud rate (9600)

**Compilation errors:**
- Install missing libraries
- Check board selection
- Verify pin definitions

## 📞 Getting Help

**Before asking for help:**
1. Check existing documentation
2. Search GitHub issues
3. Run diagnostic tests

**How to ask:**
- **Describe the problem clearly**
- **Include error messages**
- **List what you've tried**
- **Provide code snippets**

**Contact:**
- **Firmware**: Victoria Yang
- **Hardware**: Joyce Chou, Chang Li
- **Mechanical**: Shareef Jasim
- **GitHub Issues**: Use for bugs/features

## 📅 Review Process

### Code Review Checklist

Reviewers should check:
- [ ] Code follows style guidelines
- [ ] Logic is clear and well-commented
- [ ] No obvious bugs or security issues
- [ ] Tests included and passing
- [ ] Documentation updated
- [ ] Hardware impact considered

### Review Timeline

- **Simple fixes**: 1 day
- **New features**: 2-3 days
- **Major changes**: 1 week

## 🎯 Project Milestones

### Gate 7 (Week 1) - Current
- [x] Basic sensor integration
- [x] I2C depth sensor working
- [x] GPS parsing functional
- [x] Simple actuation prototype

### Gate 8 (Week 2-3)
- [ ] Multi-chamber actuation
- [ ] Mission state machine
- [ ] Data logging to SD card
- [ ] Dashboard interface

### Final (March 2025)
- [ ] Field-tested device
- [ ] Complete sensor suite
- [ ] Drone integration
- [ ] Production-ready firmware

## 📄 License

This project is licensed under the MIT License - see LICENSE file.

---

**Questions?** Open a GitHub issue or contact the team!

**Last Updated**: January 14, 2025
