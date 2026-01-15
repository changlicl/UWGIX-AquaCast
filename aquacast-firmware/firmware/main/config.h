/**
 * @file config.h
 * @brief Configuration constants and pin definitions for AquaCast firmware
 * 
 * This file contains all hardware pin mappings, sensor parameters, and system
 * configuration constants. Modify these values to match your hardware setup.
 * 
 * @author Victoria Yang (Firmware Lead)
 * @date 2025-01-14
 * @version 0.1.0
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

#define FIRMWARE_VERSION "0.1.0"
#define DEVICE_ID "ESP32_DEV_01"          // Unique device identifier

// Debug mode (1 = enabled, 0 = disabled)
#define DEBUG_MODE 1

#if DEBUG_MODE
  #define DEBUG_SERIAL Serial
  #define DEBUG_PRINT(x) DEBUG_SERIAL.print(x)
  #define DEBUG_PRINTLN(x) DEBUG_SERIAL.println(x)
  #define DEBUG_PRINTF(fmt, ...) DEBUG_SERIAL.printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

#define SERIAL_BAUD 115200                // USB serial baud rate
#define GPS_BAUD 9600                     // GPS UART baud rate

// ============================================================================
// I2C CONFIGURATION
// ============================================================================

#define I2C_SDA_PIN 21                    // I2C data line
#define I2C_SCL_PIN 22                    // I2C clock line
#define I2C_FREQUENCY 100000              // 100kHz (use 400000 for fast mode)

// I2C Device Addresses
#define I2C_ADDR_DEPTH_SENSOR 0x76        // Bar30 depth/pressure sensor
#define I2C_ADDR_OLED_DISPLAY 0x3C        // Optional OLED display (future)

// ============================================================================
// GPS CONFIGURATION (UART)
// ============================================================================

#define GPS_RX_PIN 16                     // ESP32 RX ← GPS TX
#define GPS_TX_PIN 17                     // ESP32 TX → GPS RX
#define GPS_TIMEOUT_MS 30000              // GPS fix timeout (30 seconds)
#define GPS_MIN_SATELLITES 4              // Minimum satellites for valid fix
#define GPS_MAX_HDOP 5.0                  // Maximum HDOP for acceptable fix

// ============================================================================
// ANALOG SENSOR PINS (ADC)
// ============================================================================

#define TURBIDITY_PIN 34                  // Turbidity sensor analog input
#define PH_PIN 35                         // pH sensor analog input
#define SALINITY_PIN 36                   // Salinity sensor (future)
#define BATTERY_VOLTAGE_PIN 39            // Battery voltage monitor

// ADC Configuration
#define ADC_RESOLUTION 12                 // 12-bit ADC (0-4095)
#define ADC_VREF 3.3                      // Reference voltage

// ============================================================================
// ACTUATION PINS
// ============================================================================

#define CHAMBER_1_PIN 25                  // Chamber 1 servo/actuator
#define CHAMBER_2_PIN 26                  // Chamber 2 servo/actuator
#define CHAMBER_3_PIN 27                  // Chamber 3 servo/actuator (future)
#define CHAMBER_4_PIN 14                  // Chamber 4 servo/actuator (future)

// Actuation confirmation sensors (Hall effect or limit switches)
#define CHAMBER_1_CONFIRM_PIN 32
#define CHAMBER_2_CONFIRM_PIN 33

// Actuation timing
#define ACTUATION_TIMEOUT_MS 2000         // Max time to wait for confirmation
#define ACTUATION_RETRY_ATTEMPTS 3        // Number of retry attempts

// ============================================================================
// STATUS INDICATORS
// ============================================================================

#define LED_STATUS_PIN 2                  // Built-in LED
#define LED_GPS_PIN 4                     // GPS lock indicator (external LED)
#define LED_ERROR_PIN 5                   // Error indicator (external LED)

// ============================================================================
// DEPTH SENSOR CONFIGURATION
// ============================================================================

// Depth calculation constants
#define SEA_LEVEL_PRESSURE_MBAR 1013.25   // Standard sea level pressure
#define WATER_DENSITY_KG_M3 1025.0        // Seawater density
#define GRAVITY_M_S2 9.80665              // Gravitational acceleration

// Conversion: pressure (mbar) → depth (meters)
// depth_m = (pressure_mbar - sea_level_pressure) / (water_density * gravity / 100)
#define MBAR_PER_METER 100.0              // Approximate pressure change per meter

// Depth sensor calibration
#define DEPTH_OFFSET_M 0.0                // Calibration offset (adjust after testing)
#define DEPTH_FILTER_ALPHA 0.8            // Low-pass filter coefficient (0.0-1.0)

// ============================================================================
// SAMPLING CONFIGURATION
// ============================================================================

#define DEFAULT_SAMPLING_RATE_HZ 1.0      // Default sensor reading frequency
#define MAX_SAMPLING_RATE_HZ 10.0         // Maximum supported sampling rate
#define SAMPLING_INTERVAL_MS (1000 / DEFAULT_SAMPLING_RATE_HZ)

// Depth trigger thresholds
#define TARGET_DEPTH_1_M 0.5              // First sampling depth (meters)
#define TARGET_DEPTH_2_M 2.0              // Second sampling depth (meters)
#define DEPTH_TOLERANCE_M 0.05            // Acceptable depth error (±5cm)
#define DEPTH_STABLE_COUNT 3              // Consecutive readings required for trigger

// ============================================================================
// DATA LOGGING
// ============================================================================

#define SD_CS_PIN 5                       // SD card chip select (if using SD)
#define LOG_BUFFER_SIZE 512               // Bytes allocated for log buffer
#define MAX_LOG_ENTRIES 1000              // Maximum entries before forcing flush

// Data format
#define USE_JSON_FORMAT 1                 // 1 = JSON, 0 = CSV
#define TIMESTAMP_FORMAT "ISO8601"        // ISO8601 or UNIX

// ============================================================================
// MISSION TIMING
// ============================================================================

#define MISSION_TIMEOUT_MS 3600000        // Max mission duration (1 hour)
#define DESCENT_RATE_M_S 0.1              // Expected descent rate (for ETA)
#define ASCENT_RATE_M_S 0.2               // Expected ascent rate

// ============================================================================
// BATTERY MONITORING
// ============================================================================

#define BATTERY_MIN_VOLTAGE 3.0           // Minimum operating voltage
#define BATTERY_CRITICAL_VOLTAGE 2.8      // Critical voltage (abort mission)
#define BATTERY_VOLTAGE_DIVIDER 2.0       // Voltage divider ratio (if used)

// ============================================================================
// SYSTEM LIMITS
// ============================================================================

#define MAX_DEPTH_M 5.0                   // Maximum safe operating depth
#define MAX_TEMPERATURE_C 40.0            // Maximum operating temperature
#define MIN_TEMPERATURE_C -10.0           // Minimum operating temperature

// ============================================================================
// HEALTH CHECK INTERVALS
// ============================================================================

#define HEALTH_CHECK_INTERVAL_MS 5000     // System health check frequency
#define SENSOR_TIMEOUT_MS 1000            // Timeout for sensor reads
#define I2C_RETRY_ATTEMPTS 3              // I2C communication retry count

// ============================================================================
// HEALTH FLAGS (Byte Flags)
// ============================================================================

#define HEALTH_GPS_LOCK       (1 << 7)    // Bit 7: GPS has fix
#define HEALTH_I2C_OK         (1 << 6)    // Bit 6: I2C communication OK
#define HEALTH_SD_OK          (1 << 5)    // Bit 5: SD card write OK
#define HEALTH_BATTERY_OK     (1 << 4)    // Bit 4: Battery voltage adequate
#define HEALTH_DEPTH_OK       (1 << 3)    // Bit 3: Depth sensor operational
#define HEALTH_CHAMBER_1_OK   (1 << 2)    // Bit 2: Chamber 1 confirmed
#define HEALTH_CHAMBER_2_OK   (1 << 1)    // Bit 1: Chamber 2 confirmed
#define HEALTH_RESERVED       (1 << 0)    // Bit 0: Reserved

// ============================================================================
// WIFI CONFIGURATION (OPTIONAL - FOR FUTURE USE)
// ============================================================================

// #define WIFI_SSID "AquaCast_Device"
// #define WIFI_PASSWORD "SecurePassword123"
// #define WIFI_ENABLED 0                 // Enable WiFi for debugging/config

// ============================================================================
// ADVANCED SETTINGS
// ============================================================================

// Watchdog timer
#define WATCHDOG_TIMEOUT_S 30             // Reset if no activity for 30 seconds

// Memory management
#define STACK_SIZE 8192                   // Task stack size (bytes)

// Timing precision
#define MILLIS_PER_SECOND 1000
#define MICROS_PER_SECOND 1000000

#endif // CONFIG_H
