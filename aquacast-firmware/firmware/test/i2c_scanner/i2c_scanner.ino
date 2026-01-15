/**
 * @file i2c_scanner.ino
 * @brief I2C bus scanner for debugging sensor connections
 * 
 * This sketch scans the I2C bus and reports all detected devices.
 * Useful for verifying sensor connections and identifying I2C addresses.
 * 
 * @author Joyce Chou, Chang Li (Hardware Team)
 * @date 2025-01-14
 */

#include <Wire.h>

// Pin definitions (adjust based on your ESP32 board)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQUENCY 100000  // 100kHz

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║     I2C Bus Scanner v1.0           ║");
  Serial.println("║     AquaCast Hardware Test         ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println();
  
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQUENCY);
  
  Serial.printf("I2C initialized on SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.printf("Clock frequency: %d Hz\n", I2C_FREQUENCY);
  Serial.println();
}

void loop() {
  Serial.println("Scanning I2C bus...");
  Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  
  uint8_t device_count = 0;
  
  for (uint8_t address = 0; address < 128; address++) {
    // Print row header
    if (address % 16 == 0) {
      Serial.printf("%02X: ", address & 0xF0);
    }
    
    // Test this address
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      // Device found
      Serial.printf("%02X ", address);
      device_count++;
    } else {
      // No device
      Serial.print("-- ");
    }
    
    // End of row
    if ((address + 1) % 16 == 0) {
      Serial.println();
    }
    
    delay(5);  // Small delay between tests
  }
  
  Serial.println();
  Serial.println("────────────────────────────────────────");
  Serial.printf("Scan complete. Found %d device(s).\n", device_count);
  
  // Identify known devices
  if (device_count > 0) {
    Serial.println("\nIdentified devices:");
    identifyDevices();
  }
  
  Serial.println("────────────────────────────────────────\n");
  Serial.println("Scanning again in 5 seconds...\n");
  
  delay(5000);
}

void identifyDevices() {
  // List of known I2C addresses for AquaCast components
  struct KnownDevice {
    uint8_t address;
    const char* name;
  };
  
  const KnownDevice known_devices[] = {
    {0x76, "Depth/Pressure Sensor (Bar30)"},
    {0x77, "Depth/Pressure Sensor (alternate address)"},
    {0x3C, "OLED Display (128x64)"},
    {0x48, "ADS1115 ADC"},
    {0x68, "MPU6050 IMU / RTC DS3231"},
    {0x50, "EEPROM AT24Cxx"}
  };
  
  const uint8_t num_known = sizeof(known_devices) / sizeof(KnownDevice);
  
  for (uint8_t i = 0; i < num_known; i++) {
    Wire.beginTransmission(known_devices[i].address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X: %s\n", 
        known_devices[i].address, 
        known_devices[i].name);
    }
  }
}
