/**
 * @file sensors.cpp
 * @brief Implementation of sensor interface layer
 * 
 * @author Victoria Yang (Firmware), Joyce Chou (Hardware Integration)
 * @date 2025-01-14
 */

#include "sensors.h"
#include <ArduinoJson.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

SensorManager::SensorManager() 
  : gps_serial_(2),  // Use UART2 for GPS
    depth_offset_m_(DEPTH_OFFSET_M),
    last_depth_m_(0.0),
    surface_pressure_mbar_(SEA_LEVEL_PRESSURE_MBAR),
    health_flags_(0),
    last_gps_update_ms_(0),
    mission_start_ms_(0),
    depth_sensor_initialized_(false),
    gps_initialized_(false)
{
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool SensorManager::begin() {
  DEBUG_PRINTLN("=== Initializing Sensor Manager ===");
  
  mission_start_ms_ = millis();
  
  // Initialize I2C bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQUENCY);
  delay(100);
  
  // Scan I2C bus for debugging
  uint8_t device_count = scanI2CBus();
  DEBUG_PRINTF("Found %d I2C devices\n", device_count);
  
  // Initialize depth sensor
  if (!initDepthSensor()) {
    DEBUG_PRINTLN("ERROR: Depth sensor initialization failed");
    health_flags_ &= ~HEALTH_DEPTH_OK;
  } else {
    DEBUG_PRINTLN("✓ Depth sensor initialized");
    health_flags_ |= HEALTH_DEPTH_OK;
  }
  
  // Initialize GPS
  if (!initGPS()) {
    DEBUG_PRINTLN("WARNING: GPS initialization failed");
    health_flags_ &= ~HEALTH_GPS_LOCK;
  } else {
    DEBUG_PRINTLN("✓ GPS initialized");
  }
  
  // Configure ADC for analog sensors
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_11db);  // Full scale 0-3.3V
  
  DEBUG_PRINTLN("=== Sensor Manager Ready ===\n");
  
  return depth_sensor_initialized_;  // At minimum, depth sensor must work
}

// ============================================================================
// DEPTH SENSOR (I2C BAR30)
// ============================================================================

bool SensorManager::initDepthSensor() {
  // Check if device responds at expected address
  Wire.beginTransmission(I2C_ADDR_DEPTH_SENSOR);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    depth_sensor_initialized_ = true;
    
    // Read initial surface pressure for calibration
    float pressure, temp;
    if (readDepthSensorRaw(&pressure, &temp)) {
      surface_pressure_mbar_ = pressure;
      DEBUG_PRINTF("Surface pressure: %.2f mbar\n", surface_pressure_mbar_);
    }
    
    return true;
  }
  
  DEBUG_PRINTF("I2C error %d for depth sensor at 0x%02X\n", error, I2C_ADDR_DEPTH_SENSOR);
  return false;
}

bool SensorManager::readDepthSensor(DepthData* data) {
  if (!depth_sensor_initialized_) {
    data->valid = false;
    return false;
  }
  
  float pressure, temperature;
  if (!readDepthSensorRaw(&pressure, &temperature)) {
    data->valid = false;
    health_flags_ &= ~HEALTH_DEPTH_OK;
    return false;
  }
  
  // Convert pressure to depth
  float raw_depth = convertPressureToDepth(pressure);
  
  // Apply low-pass filter to reduce noise
  float filtered_depth = (DEPTH_FILTER_ALPHA * raw_depth) + 
                         ((1.0 - DEPTH_FILTER_ALPHA) * last_depth_m_);
  last_depth_m_ = filtered_depth;
  
  // Fill data structure
  data->depth_m = filtered_depth + depth_offset_m_;
  data->pressure_mbar = pressure;
  data->temperature_c = temperature;
  data->valid = true;
  data->timestamp_ms = millis();
  
  health_flags_ |= HEALTH_DEPTH_OK;
  return true;
}

bool SensorManager::readDepthSensorRaw(float* pressure, float* temperature) {
  // This is a placeholder - actual implementation depends on sensor protocol
  // For Bar30, you would typically:
  // 1. Send read command
  // 2. Wait for conversion
  // 3. Read pressure and temperature registers
  
  // Example I2C read sequence (adapt to your sensor's datasheet)
  Wire.beginTransmission(I2C_ADDR_DEPTH_SENSOR);
  Wire.write(0x00);  // Register address for pressure (example)
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Request 4 bytes (pressure + temperature data)
  Wire.requestFrom(I2C_ADDR_DEPTH_SENSOR, (uint8_t)4);
  
  if (Wire.available() >= 4) {
    // Read pressure (16-bit value, example format)
    uint16_t pressure_raw = (Wire.read() << 8) | Wire.read();
    uint16_t temp_raw = (Wire.read() << 8) | Wire.read();
    
    // Convert to physical units (these formulas are sensor-specific)
    // Consult Bar30 datasheet for actual conversion
    *pressure = surface_pressure_mbar_ + (pressure_raw / 100.0);  // Example
    *temperature = (temp_raw / 100.0) - 40.0;  // Example
    
    return true;
  }
  
  return false;
}

float SensorManager::convertPressureToDepth(float pressure_mbar) {
  // Hydrostatic pressure formula: P = ρgh
  // depth (m) = (P - P0) / (ρ * g) * 100
  // where 100 converts mbar to Pascals
  
  float pressure_diff_mbar = pressure_mbar - surface_pressure_mbar_;
  float depth_m = pressure_diff_mbar / MBAR_PER_METER;
  
  return max(0.0f, depth_m);  // Depth cannot be negative
}

void SensorManager::calibrateDepthSensor(float surface_pressure_mbar) {
  surface_pressure_mbar_ = surface_pressure_mbar;
  depth_offset_m_ = 0.0;  // Reset offset
  DEBUG_PRINTF("Depth sensor calibrated to %.2f mbar\n", surface_pressure_mbar);
}

// ============================================================================
// GPS (UART NEO-6M)
// ============================================================================

bool SensorManager::initGPS() {
  gps_serial_.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  
  gps_initialized_ = true;
  DEBUG_PRINTLN("GPS UART initialized (waiting for fix...)");
  
  return true;
}

bool SensorManager::readGPS(GpsData* data) {
  if (!gps_initialized_) {
    data->has_fix = false;
    return false;
  }
  
  // Feed GPS parser with available data
  updateGPS();
  
  // Check if we have a valid fix
  if (gps_parser_.location.isValid() && 
      gps_parser_.location.age() < 2000) {  // Data less than 2 seconds old
    
    data->latitude = gps_parser_.location.lat();
    data->longitude = gps_parser_.location.lng();
    data->altitude_m = gps_parser_.altitude.meters();
    data->satellites = gps_parser_.satellites.value();
    data->hdop = gps_parser_.hdop.hdop();
    data->has_fix = true;
    data->timestamp_ms = millis();
    
    // Determine fix quality
    if (data->satellites >= GPS_MIN_SATELLITES && data->hdop <= GPS_MAX_HDOP) {
      data->fix_quality = 1;  // Good GPS fix
      health_flags_ |= HEALTH_GPS_LOCK;
    } else {
      data->fix_quality = 0;  // Poor quality fix
      health_flags_ &= ~HEALTH_GPS_LOCK;
    }
    
    last_gps_update_ms_ = millis();
    return true;
    
  } else {
    // No valid fix
    data->has_fix = false;
    data->fix_quality = 0;
    health_flags_ &= ~HEALTH_GPS_LOCK;
    
    // Timeout check
    if (millis() - mission_start_ms_ > GPS_TIMEOUT_MS) {
      DEBUG_PRINTLN("WARNING: GPS timeout - no fix acquired");
    }
    
    return false;
  }
}

void SensorManager::updateGPS() {
  // Read all available GPS data and feed to parser
  while (gps_serial_.available() > 0) {
    char c = gps_serial_.read();
    gps_parser_.encode(c);
    
    #if DEBUG_MODE >= 2
      DEBUG_PRINT(c);  // Echo GPS NMEA sentences for debugging
    #endif
  }
}

bool SensorManager::hasGPSFix() {
  updateGPS();
  return gps_parser_.location.isValid() && 
         gps_parser_.location.age() < 2000 &&
         gps_parser_.satellites.value() >= GPS_MIN_SATELLITES;
}

// ============================================================================
// ANALOG SENSORS
// ============================================================================

bool SensorManager::readTurbidity(AnalogSensorData* data) {
  float raw_value = readAnalogSensor(TURBIDITY_PIN);
  
  // Convert ADC value to turbidity (NTU)
  // Calibration formula depends on your specific sensor
  // Example: Linear conversion
  float voltage = (raw_value / 4095.0) * ADC_VREF;
  float turbidity_ntu = (voltage - 0.0) * 1000.0;  // Example calibration
  
  data->raw_value = raw_value;
  data->calibrated_value = turbidity_ntu;
  data->valid = true;
  data->timestamp_ms = millis();
  
  return true;
}

bool SensorManager::readPH(AnalogSensorData* data) {
  float raw_value = readAnalogSensor(PH_PIN);
  
  // Convert ADC value to pH (0-14 scale)
  // Calibration formula depends on your specific sensor
  float voltage = (raw_value / 4095.0) * ADC_VREF;
  float ph_value = 7.0 + (voltage - 1.65) * 3.0;  // Example calibration
  
  data->raw_value = raw_value;
  data->calibrated_value = constrain(ph_value, 0.0, 14.0);
  data->valid = true;
  data->timestamp_ms = millis();
  
  return true;
}

float SensorManager::readAnalogSensor(uint8_t pin, uint8_t samples) {
  // Oversample for noise reduction
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return (float)sum / samples;
}

// ============================================================================
// COMPLETE SENSOR READING
// ============================================================================

bool SensorManager::getSensorReading(SensorReading* reading) {
  reading->mission_time_ms = millis() - mission_start_ms_;
  
  // Read all sensors
  bool depth_ok = readDepthSensor(&reading->depth);
  bool gps_ok = readGPS(&reading->gps);
  bool turbidity_ok = readTurbidity(&reading->turbidity);
  bool ph_ok = readPH(&reading->ph);
  
  // Update health flags
  reading->health_flags = getHealthStatus();
  
  // Return true if at least depth sensor works
  return depth_ok;
}

// ============================================================================
// HEALTH MONITORING
// ============================================================================

uint8_t SensorManager::getHealthStatus() {
  // Health flags are updated by individual sensor read functions
  
  // Check battery voltage
  float battery_voltage = readAnalogSensor(BATTERY_VOLTAGE_PIN, 5) * 
                          BATTERY_VOLTAGE_DIVIDER / 4095.0 * ADC_VREF;
  
  if (battery_voltage > BATTERY_MIN_VOLTAGE) {
    health_flags_ |= HEALTH_BATTERY_OK;
  } else {
    health_flags_ &= ~HEALTH_BATTERY_OK;
    DEBUG_PRINTF("WARNING: Low battery voltage: %.2fV\n", battery_voltage);
  }
  
  return health_flags_;
}

bool SensorManager::checkSafeLimits(const SensorReading& reading) {
  bool safe = true;
  
  // Check depth limit
  if (reading.depth.depth_m > MAX_DEPTH_M) {
    DEBUG_PRINTF("ERROR: Exceeded max depth: %.2fm\n", reading.depth.depth_m);
    safe = false;
  }
  
  // Check temperature limits
  if (reading.depth.temperature_c > MAX_TEMPERATURE_C || 
      reading.depth.temperature_c < MIN_TEMPERATURE_C) {
    DEBUG_PRINTF("ERROR: Temperature out of range: %.1f°C\n", reading.depth.temperature_c);
    safe = false;
  }
  
  // Check battery
  if (!(reading.health_flags & HEALTH_BATTERY_OK)) {
    DEBUG_PRINTLN("ERROR: Battery voltage critical");
    safe = false;
  }
  
  return safe;
}

// ============================================================================
// I2C DEBUGGING
// ============================================================================

uint8_t SensorManager::scanI2CBus() {
  uint8_t count = 0;
  DEBUG_PRINTLN("Scanning I2C bus...");
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      DEBUG_PRINTF("  Device found at 0x%02X\n", addr);
      count++;
      
      if (addr == I2C_ADDR_DEPTH_SENSOR) {
        DEBUG_PRINTLN("    → Identified as depth sensor");
      }
    }
  }
  
  if (count == 0) {
    DEBUG_PRINTLN("  No I2C devices found!");
  }
  
  return count;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printSensorReading(const SensorReading& reading) {
  DEBUG_PRINTLN("========== Sensor Reading ==========");
  DEBUG_PRINTF("Mission Time: %lu ms\n", reading.mission_time_ms);
  DEBUG_PRINTLN("------------------------------------");
  
  if (reading.depth.valid) {
    DEBUG_PRINTF("Depth:        %.2f m\n", reading.depth.depth_m);
    DEBUG_PRINTF("Pressure:     %.2f mbar\n", reading.depth.pressure_mbar);
    DEBUG_PRINTF("Temperature:  %.1f °C\n", reading.depth.temperature_c);
  } else {
    DEBUG_PRINTLN("Depth:        INVALID");
  }
  
  DEBUG_PRINTLN("------------------------------------");
  
  if (reading.gps.has_fix) {
    DEBUG_PRINTF("GPS:          %.6f, %.6f\n", reading.gps.latitude, reading.gps.longitude);
    DEBUG_PRINTF("Satellites:   %d\n", reading.gps.satellites);
    DEBUG_PRINTF("HDOP:         %.1f\n", reading.gps.hdop);
  } else {
    DEBUG_PRINTLN("GPS:          NO FIX");
  }
  
  DEBUG_PRINTLN("------------------------------------");
  DEBUG_PRINTF("Turbidity:    %.1f NTU\n", reading.turbidity.calibrated_value);
  DEBUG_PRINTF("pH:           %.2f\n", reading.ph.calibrated_value);
  DEBUG_PRINTLN("------------------------------------");
  DEBUG_PRINTF("Health Flags: 0x%02X\n", reading.health_flags);
  DEBUG_PRINTLN("====================================\n");
}

bool sensorReadingToJSON(const SensorReading& reading, char* buffer, size_t buffer_size) {
  StaticJsonDocument<512> doc;
  
  doc["timestamp"] = reading.mission_time_ms;
  doc["depth_m"] = reading.depth.depth_m;
  doc["pressure_mbar"] = reading.depth.pressure_mbar;
  doc["temperature_c"] = reading.depth.temperature_c;
  
  JsonObject gps = doc.createNestedObject("gps");
  gps["latitude"] = reading.gps.latitude;
  gps["longitude"] = reading.gps.longitude;
  gps["fix"] = reading.gps.has_fix;
  gps["satellites"] = reading.gps.satellites;
  gps["hdop"] = reading.gps.hdop;
  
  doc["turbidity_ntu"] = reading.turbidity.calibrated_value;
  doc["ph"] = reading.ph.calibrated_value;
  doc["health_flags"] = reading.health_flags;
  
  size_t len = serializeJson(doc, buffer, buffer_size);
  return (len > 0 && len < buffer_size);
}
