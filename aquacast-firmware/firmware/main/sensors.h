/**
 * @file sensors.h
 * @brief Sensor interface layer for AquaCast water sampling device
 * 
 * This module provides a unified interface for all sensors used in the system:
 * - I2C Depth/Pressure sensor (Bar30)
 * - UART GPS module (NEO-6M)
 * - Analog turbidity sensor
 * - Analog pH sensor (optional)
 * 
 * @date 2025-01-14
 * @version 0.1.0
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

#include "config.h"

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct DepthData
 * @brief Depth and pressure sensor readings
 */
struct DepthData {
  float depth_m;              // Depth in meters
  float pressure_mbar;        // Absolute pressure in millibars
  float temperature_c;        // Water temperature in Celsius
  bool valid;                 // Data validity flag
  uint32_t timestamp_ms;      // Timestamp of reading
};

/**
 * @struct GpsData
 * @brief GPS position and quality metrics
 */
struct GpsData {
  double latitude;            // Latitude in decimal degrees
  double longitude;           // Longitude in decimal degrees
  float altitude_m;           // Altitude above sea level (meters)
  uint8_t satellites;         // Number of satellites in use
  float hdop;                 // Horizontal Dilution of Precision
  uint8_t fix_quality;        // Fix quality (0=invalid, 1=GPS fix, 2=DGPS fix)
  bool has_fix;               // True if GPS has valid fix
  uint32_t timestamp_ms;      // Timestamp of reading
};

/**
 * @struct AnalogSensorData
 * @brief Generic analog sensor reading
 */
struct AnalogSensorData {
  float raw_value;            // Raw ADC reading (0-4095)
  float calibrated_value;     // Calibrated value in appropriate units
  bool valid;                 // Data validity flag
  uint32_t timestamp_ms;      // Timestamp of reading
};

/**
 * @struct SensorReading
 * @brief Complete sensor suite reading
 */
struct SensorReading {
  uint32_t mission_time_ms;   // Time since mission start
  DepthData depth;            // Depth sensor data
  GpsData gps;                // GPS data
  AnalogSensorData turbidity; // Turbidity (NTU)
  AnalogSensorData ph;        // pH value (0-14 scale)
  uint8_t health_flags;       // System health status byte
};

// ============================================================================
// SENSOR MANAGER CLASS
// ============================================================================

/**
 * @class SensorManager
 * @brief Main sensor interface and coordination class
 * 
 * Handles initialization, reading, and calibration of all sensors.
 * Implements error handling and health monitoring.
 */
class SensorManager {
public:
  /**
   * @brief Constructor
   */
  SensorManager();

  /**
   * @brief Initialize all sensors
   * @return true if initialization successful, false otherwise
   */
  bool begin();

  /**
   * @brief Get complete sensor reading
   * @param reading Pointer to SensorReading structure to fill
   * @return true if reading successful, false otherwise
   */
  bool getSensorReading(SensorReading* reading);

  /**
   * @brief Read depth sensor
   * @param data Pointer to DepthData structure to fill
   * @return true if reading successful, false otherwise
   */
  bool readDepthSensor(DepthData* data);

  /**
   * @brief Read GPS module
   * @param data Pointer to GpsData structure to fill
   * @return true if reading successful, false otherwise
   */
  bool readGPS(GpsData* data);

  /**
   * @brief Read turbidity sensor
   * @param data Pointer to AnalogSensorData structure to fill
   * @return true if reading successful, false otherwise
   */
  bool readTurbidity(AnalogSensorData* data);

  /**
   * @brief Read pH sensor
   * @param data Pointer to AnalogSensorData structure to fill
   * @return true if reading successful, false otherwise
   */
  bool readPH(AnalogSensorData* data);

  /**
   * @brief Check if GPS has valid fix
   * @return true if GPS has fix, false otherwise
   */
  bool hasGPSFix();

  /**
   * @brief Get system health status byte
   * @return Health status flags
   */
  uint8_t getHealthStatus();

  /**
   * @brief Calibrate depth sensor (set zero point)
   * @param surface_pressure_mbar Current surface pressure in millibars
   */
  void calibrateDepthSensor(float surface_pressure_mbar);

  /**
   * @brief Perform I2C bus scan for debugging
   * @return Number of devices found
   */
  uint8_t scanI2CBus();

  /**
   * @brief Check if sensor is within safe operating limits
   * @param reading Sensor reading to check
   * @return true if within limits, false otherwise
   */
  bool checkSafeLimits(const SensorReading& reading);

private:
  // Private sensor reading functions
  bool initDepthSensor();
  bool initGPS();
  bool readDepthSensorRaw(float* pressure, float* temperature);
  void updateGPS();
  float convertPressureToDepth(float pressure_mbar);
  float readAnalogSensor(uint8_t pin, uint8_t samples = 10);
  
  // Private member variables
  TinyGPSPlus gps_parser_;               // GPS parser object
  HardwareSerial gps_serial_;            // GPS serial port
  
  float depth_offset_m_;                 // Depth calibration offset
  float last_depth_m_;                   // Last depth reading (for filtering)
  float surface_pressure_mbar_;          // Reference surface pressure
  
  uint8_t health_flags_;                 // Current health status
  uint32_t last_gps_update_ms_;          // Last GPS update timestamp
  uint32_t mission_start_ms_;            // Mission start time
  
  bool depth_sensor_initialized_;
  bool gps_initialized_;
};

// ============================================================================
// GLOBAL HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Print sensor reading to serial (for debugging)
 * @param reading Sensor reading to print
 */
void printSensorReading(const SensorReading& reading);

/**
 * @brief Convert sensor reading to JSON string
 * @param reading Sensor reading
 * @param buffer Output buffer
 * @param buffer_size Size of output buffer
 * @return true if conversion successful
 */
bool sensorReadingToJSON(const SensorReading& reading, char* buffer, size_t buffer_size);

#endif // SENSORS_H
