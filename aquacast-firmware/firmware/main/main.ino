/**
 * @file main.ino
 * @brief AquaCast Firmware - Main application sketch
 * 
 * This is the entry point for the AquaCast autonomous water sampling device.
 * It initializes all subsystems and runs the main mission control loop.
 * 
 * @date 2025-01-14
 * @version 0.1.0
 */

#include "config.h"
#include "sensors.h"
#include "actuation.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

SensorManager sensor_manager;
ActuationManager actuation_manager(ACTUATION_MAGNETIC);
DepthTriggerController trigger_controller(&actuation_manager);

// ============================================================================
// MISSION STATE MACHINE
// ============================================================================

enum MissionState {
  STATE_IDLE,           // Waiting for mission configuration
  STATE_INIT,           // Initializing systems
  STATE_READY,          // Ready to deploy
  STATE_DESCENT,        // Descending to sample depths
  STATE_SAMPLING,       // Actively sampling
  STATE_SURFACING,      // Returning to surface
  STATE_COMPLETE,       // Mission complete
  STATE_ERROR           // Error state
};

MissionState current_state = STATE_IDLE;
uint32_t state_start_time = 0;
uint32_t mission_start_time = 0;
uint32_t last_sensor_read = 0;
uint32_t last_health_check = 0;

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  delay(1000);  // Give serial time to initialize
  
  DEBUG_PRINTLN("\n\n");
  DEBUG_PRINTLN("╔════════════════════════════════════════╗");
  DEBUG_PRINTLN("║         AquaCast Firmware v0.1.0       ║");
  DEBUG_PRINTLN("║   Autonomous Water Sampling Device     ║");
  DEBUG_PRINTLN("║        TECHIN 540 SP 2026              ║");
  DEBUG_PRINTLN("╚════════════════════════════════════════╝");
  DEBUG_PRINTLN();
  
  DEBUG_PRINTF("Device ID: %s\n", DEVICE_ID);
  DEBUG_PRINTF("Firmware: %s\n", FIRMWARE_VERSION);
  DEBUG_PRINTF("Build: %s %s\n", __DATE__, __TIME__);
  DEBUG_PRINTLN();
  
  // Configure status LED
  pinMode(LED_STATUS_PIN, OUTPUT);
  blinkLED(3, 200);  // Startup indicator
  
  // Transition to initialization state
  changeState(STATE_INIT);
}

// ============================================================================
// LOOP - RUNS CONTINUOUSLY
// ============================================================================

void loop() {
  uint32_t current_time = millis();
  
  // Process state machine
  switch (current_state) {
    case STATE_IDLE:
      handleIdleState();
      break;
      
    case STATE_INIT:
      handleInitState();
      break;
      
    case STATE_READY:
      handleReadyState();
      break;
      
    case STATE_DESCENT:
      handleDescentState();
      break;
      
    case STATE_SAMPLING:
      handleSamplingState();
      break;
      
    case STATE_SURFACING:
      handleSurfacingState();
      break;
      
    case STATE_COMPLETE:
      handleCompleteState();
      break;
      
    case STATE_ERROR:
      handleErrorState();
      break;
  }
  
  // Periodic health check
  if (current_time - last_health_check >= HEALTH_CHECK_INTERVAL_MS) {
    performHealthCheck();
    last_health_check = current_time;
  }
  
  // Process serial commands
  if (Serial.available()) {
    processSerialCommand();
  }
  
  // Small delay to prevent CPU thrashing
  delay(10);
}

// ============================================================================
// STATE HANDLERS
// ============================================================================

void handleIdleState() {
  // Waiting for user to start mission
  // In production, this would wait for configuration upload
  static bool prompt_shown = false;
  
  if (!prompt_shown) {
    DEBUG_PRINTLN("System IDLE - Send 'START' to begin mission");
    prompt_shown = true;
  }
  
  // Blink LED slowly
  static uint32_t last_blink = 0;
  if (millis() - last_blink > 1000) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    last_blink = millis();
  }
}

void handleInitState() {
  DEBUG_PRINTLN("\n>>> Initializing Systems <<<\n");
  
  // Initialize sensor manager
  if (!sensor_manager.begin()) {
    DEBUG_PRINTLN("CRITICAL ERROR: Sensor initialization failed");
    changeState(STATE_ERROR);
    return;
  }
  
  // Initialize actuation manager
  if (!actuation_manager.begin()) {
    DEBUG_PRINTLN("ERROR: Actuation initialization failed");
    changeState(STATE_ERROR);
    return;
  }
  
  // Configure depth triggers (example mission)
  trigger_controller.clearTriggers();
  trigger_controller.addTrigger(1, TARGET_DEPTH_1_M);  // Chamber 1 @ 0.5m
  trigger_controller.addTrigger(2, TARGET_DEPTH_2_M);  // Chamber 2 @ 2.0m
  
  DEBUG_PRINTLN("\n✓ All systems initialized");
  changeState(STATE_READY);
}

void handleReadyState() {
  DEBUG_PRINTLN("\n>>> System Ready for Deployment <<<");
  DEBUG_PRINTLN("Waiting for GPS fix before descent...\n");
  
  // Wait for GPS lock
  if (sensor_manager.hasGPSFix()) {
    DEBUG_PRINTLN("✓ GPS fix acquired");
    
    // Log initial position
    GpsData gps_data;
    if (sensor_manager.readGPS(&gps_data)) {
      DEBUG_PRINTF("Starting position: %.6f, %.6f\n", 
        gps_data.latitude, gps_data.longitude);
    }
    
    mission_start_time = millis();
    changeState(STATE_DESCENT);
  } else {
    // Keep waiting
    DEBUG_PRINT(".");
    delay(1000);
  }
}

void handleDescentState() {
  // Read sensors at configured rate
  uint32_t current_time = millis();
  
  if (current_time - last_sensor_read >= SAMPLING_INTERVAL_MS) {
    SensorReading reading;
    
    if (sensor_manager.getSensorReading(&reading)) {
      // Print reading to serial
      printSensorReading(reading);
      
      // Check safety limits
      if (!sensor_manager.checkSafeLimits(reading)) {
        DEBUG_PRINTLN("ERROR: Safety limits exceeded!");
        changeState(STATE_ERROR);
        return;
      }
      
      // Process depth for triggering
      uint8_t triggered = trigger_controller.processDepth(reading.depth.depth_m);
      
      if (triggered > 0) {
        DEBUG_PRINTF("\n>>> %d chamber(s) triggered! <<<\n\n", triggered);
        actuation_manager.printStatus();
      }
      
      // Check if all samples collected
      if (trigger_controller.allTriggersExecuted()) {
        DEBUG_PRINTLN("\n>>> All samples collected! <<<");
        changeState(STATE_SURFACING);
      }
    }
    
    last_sensor_read = current_time;
  }
  
  // Blink LED rapidly during sampling
  static uint32_t last_blink = 0;
  if (current_time - last_blink > 200) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    last_blink = current_time;
  }
}

void handleSamplingState() {
  // Currently combined with descent state
  handleDescentState();
}

void handleSurfacingState() {
  DEBUG_PRINTLN("\n>>> Surfacing for Retrieval <<<\n");
  
  // Monitor depth until surface reached
  DepthData depth_data;
  sensor_manager.readDepthSensor(&depth_data);
  
  DEBUG_PRINTF("Current depth: %.2fm\n", depth_data.depth_m);
  
  if (depth_data.depth_m < 0.2) {  // Near surface
    DEBUG_PRINTLN("✓ Surface reached!");
    
    // Activate GPS beacon
    DEBUG_PRINTLN("Activating GPS beacon...");
    digitalWrite(LED_GPS_PIN, HIGH);
    
    // Get final GPS position
    GpsData gps_data;
    if (sensor_manager.readGPS(&gps_data)) {
      DEBUG_PRINTF("Retrieval position: %.6f, %.6f\n", 
        gps_data.latitude, gps_data.longitude);
    }
    
    changeState(STATE_COMPLETE);
  }
  
  delay(2000);  // Check every 2 seconds
}

void handleCompleteState() {
  DEBUG_PRINTLN("\n╔════════════════════════════════════════╗");
  DEBUG_PRINTLN("║       MISSION COMPLETE                 ║");
  DEBUG_PRINTLN("╚════════════════════════════════════════╝\n");
  
  // Print mission summary
  uint32_t mission_duration = millis() - mission_start_time;
  DEBUG_PRINTF("Mission duration: %lu seconds\n", mission_duration / 1000);
  
  actuation_manager.printStatus();
  
  DEBUG_PRINTLN("\nReady for retrieval and data download.");
  DEBUG_PRINTLN("Send 'EXPORT' to download data or 'RESET' for new mission.\n");
  
  // Keep GPS LED on
  digitalWrite(LED_GPS_PIN, HIGH);
  
  // Slow blink
  static uint32_t last_blink = 0;
  if (millis() - last_blink > 2000) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    last_blink = millis();
  }
}

void handleErrorState() {
  DEBUG_PRINTLN("\n!!! SYSTEM ERROR !!!");
  DEBUG_PRINTLN("Send 'RESET' to attempt recovery\n");
  
  // Rapid blink error LED
  static uint32_t last_blink = 0;
  if (millis() - last_blink > 100) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    last_blink = millis();
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void changeState(MissionState new_state) {
  const char* state_names[] = {
    "IDLE", "INIT", "READY", "DESCENT", "SAMPLING", "SURFACING", "COMPLETE", "ERROR"
  };
  
  DEBUG_PRINTF("\n>>> State Transition: %s → %s <<<\n\n", 
    state_names[current_state], state_names[new_state]);
  
  current_state = new_state;
  state_start_time = millis();
}

void blinkLED(uint8_t count, uint16_t delay_ms) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(delay_ms);
  }
}

void performHealthCheck() {
  uint8_t health = sensor_manager.getHealthStatus();
  
  #if DEBUG_MODE
    DEBUG_PRINT("Health: ");
    for (int i = 7; i >= 0; i--) {
      DEBUG_PRINT((health >> i) & 1 ? "1" : "0");
    }
    DEBUG_PRINTF(" (0x%02X)\n", health);
  #endif
  
  // Check for critical failures
  if (!(health & HEALTH_BATTERY_OK)) {
    DEBUG_PRINTLN("WARNING: Battery voltage low!");
  }
  
  if (!(health & HEALTH_DEPTH_OK) && current_state == STATE_DESCENT) {
    DEBUG_PRINTLN("ERROR: Depth sensor failure during mission");
    changeState(STATE_ERROR);
  }
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  DEBUG_PRINTF("Command received: %s\n", command.c_str());
  
  if (command == "START") {
    if (current_state == STATE_IDLE) {
      changeState(STATE_INIT);
    }
  }
  else if (command == "RESET") {
    DEBUG_PRINTLN("Resetting system...");
    ESP.restart();
  }
  else if (command == "STATUS") {
    printSystemStatus();
  }
  else if (command == "TRIGGER1") {
    actuation_manager.triggerChamber(1);
  }
  else if (command == "TRIGGER2") {
    actuation_manager.triggerChamber(2);
  }
  else if (command == "EXPORT") {
    exportData();
  }
  else if (command == "HELP") {
    printHelp();
  }
  else {
    DEBUG_PRINTLN("Unknown command. Send 'HELP' for command list.");
  }
}

void printSystemStatus() {
  DEBUG_PRINTLN("\n╔════════════════════════════════════════╗");
  DEBUG_PRINTLN("║         SYSTEM STATUS                  ║");
  DEBUG_PRINTLN("╚════════════════════════════════════════╝");
  
  const char* state_names[] = {
    "IDLE", "INIT", "READY", "DESCENT", "SAMPLING", "SURFACING", "COMPLETE", "ERROR"
  };
  DEBUG_PRINTF("State: %s\n", state_names[current_state]);
  DEBUG_PRINTF("Uptime: %lu seconds\n", millis() / 1000);
  
  if (mission_start_time > 0) {
    DEBUG_PRINTF("Mission time: %lu seconds\n", (millis() - mission_start_time) / 1000);
  }
  
  DEBUG_PRINTLN();
  actuation_manager.printStatus();
  trigger_controller.printTriggers();
}

void exportData() {
  DEBUG_PRINTLN("\n=== DATA EXPORT (CSV) ===");
  DEBUG_PRINTLN("timestamp_ms,depth_m,temp_c,pressure_mbar,lat,lon,gps_fix,turbidity,ph");
  
  // In production, this would read from SD card or internal buffer
  DEBUG_PRINTLN("(Data export not yet implemented - placeholder)");
  DEBUG_PRINTLN("=========================\n");
}

void printHelp() {
  DEBUG_PRINTLN("\n=== AVAILABLE COMMANDS ===");
  DEBUG_PRINTLN("START    - Start mission");
  DEBUG_PRINTLN("RESET    - Reset system");
  DEBUG_PRINTLN("STATUS   - Print system status");
  DEBUG_PRINTLN("TRIGGER1 - Manually trigger chamber 1");
  DEBUG_PRINTLN("TRIGGER2 - Manually trigger chamber 2");
  DEBUG_PRINTLN("EXPORT   - Export collected data");
  DEBUG_PRINTLN("HELP     - Show this help");
  DEBUG_PRINTLN("==========================\n");
}
