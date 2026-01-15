/**
 * @file actuation.cpp
 * @brief Implementation of chamber actuation control
 * 
 * @author Victoria Yang (Firmware), Shareef Jasim (Mechanical)
 * @date 2025-01-14
 */

#include "actuation.h"

// ============================================================================
// ACTUATION MANAGER
// ============================================================================

ActuationManager::ActuationManager(ActuationType type)
  : actuation_type_(type),
    open_chamber_count_(0),
    initialized_(false)
{
  // Initialize chamber statuses
  for (uint8_t i = 0; i < 4; i++) {
    chamber_status_[i].chamber_id = i + 1;
    chamber_status_[i].is_open = false;
    chamber_status_[i].trigger_depth_m = 0.0;
    chamber_status_[i].trigger_time_ms = 0;
    chamber_status_[i].last_result = ACT_SUCCESS;
    chamber_status_[i].retry_count = 0;
  }
}

bool ActuationManager::begin() {
  DEBUG_PRINTLN("=== Initializing Actuation Manager ===");
  
  // Configure actuation pins as outputs
  pinMode(CHAMBER_1_PIN, OUTPUT);
  pinMode(CHAMBER_2_PIN, OUTPUT);
  digitalWrite(CHAMBER_1_PIN, LOW);
  digitalWrite(CHAMBER_2_PIN, LOW);
  
  // Configure confirmation pins as inputs with pull-up
  pinMode(CHAMBER_1_CONFIRM_PIN, INPUT_PULLUP);
  pinMode(CHAMBER_2_CONFIRM_PIN, INPUT_PULLUP);
  
  initialized_ = true;
  DEBUG_PRINTF("Actuation type: %s\n", 
    actuation_type_ == ACTUATION_MAGNETIC ? "Magnetic" : "Servo");
  DEBUG_PRINTLN("=== Actuation Manager Ready ===\n");
  
  return true;
}

ActuationResult ActuationManager::triggerChamber(uint8_t chamber_id) {
  return triggerChamber(chamber_id, -1.0);  // Depth unknown
}

ActuationResult ActuationManager::triggerChamber(uint8_t chamber_id, float depth_m) {
  if (!initialized_) {
    DEBUG_PRINTLN("ERROR: Actuation manager not initialized");
    return ACT_MECHANICAL_FAILURE;
  }
  
  if (chamber_id < 1 || chamber_id > 4) {
    DEBUG_PRINTF("ERROR: Invalid chamber ID: %d\n", chamber_id);
    return ACT_INVALID_CHAMBER;
  }
  
  ChamberStatus* status = &chamber_status_[chamber_id - 1];
  
  // Check if already open
  if (status->is_open) {
    DEBUG_PRINTF("Chamber %d already open\n", chamber_id);
    return ACT_ALREADY_OPEN;
  }
  
  DEBUG_PRINTF("Triggering chamber %d", chamber_id);
  if (depth_m >= 0) {
    DEBUG_PRINTF(" at depth %.2fm", depth_m);
  }
  DEBUG_PRINTLN();
  
  // Attempt actuation with retries
  for (uint8_t attempt = 0; attempt < ACTUATION_RETRY_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      DEBUG_PRINTF("  Retry attempt %d/%d\n", attempt + 1, ACTUATION_RETRY_ATTEMPTS);
      delay(500);
    }
    
    // Send actuation signal
    if (!sendActuationSignal(chamber_id)) {
      status->retry_count = attempt + 1;
      continue;
    }
    
    // Wait for confirmation
    if (waitForConfirmation(chamber_id)) {
      // Success!
      status->is_open = true;
      status->trigger_depth_m = depth_m;
      status->trigger_time_ms = millis();
      status->last_result = ACT_SUCCESS;
      status->retry_count = attempt;
      open_chamber_count_++;
      
      DEBUG_PRINTF("✓ Chamber %d actuated successfully\n", chamber_id);
      return ACT_SUCCESS;
    }
  }
  
  // All retries failed
  DEBUG_PRINTF("✗ Chamber %d actuation FAILED after %d attempts\n", 
    chamber_id, ACTUATION_RETRY_ATTEMPTS);
  status->last_result = ACT_TIMEOUT;
  return ACT_TIMEOUT;
}

bool ActuationManager::sendActuationSignal(uint8_t chamber_id) {
  uint8_t pin = getChamberPin(chamber_id);
  
  if (actuation_type_ == ACTUATION_MAGNETIC) {
    // Magnetic actuation: Pulse signal
    digitalWrite(pin, HIGH);
    delay(500);  // Hold for 500ms
    digitalWrite(pin, LOW);
  } else {
    // Servo actuation: Rotate servo
    // (This would use ESP32Servo library - simplified here)
    digitalWrite(pin, HIGH);
    delay(1000);
    digitalWrite(pin, LOW);
  }
  
  return true;
}

bool ActuationManager::waitForConfirmation(uint8_t chamber_id) {
  uint8_t confirm_pin = getConfirmationPin(chamber_id);
  uint32_t start_time = millis();
  
  while (millis() - start_time < ACTUATION_TIMEOUT_MS) {
    if (checkConfirmationSensor(chamber_id)) {
      delay(50);  // Debounce
      if (checkConfirmationSensor(chamber_id)) {
        return true;  // Confirmed!
      }
    }
    delay(10);
  }
  
  return false;  // Timeout
}

bool ActuationManager::checkConfirmationSensor(uint8_t chamber_id) {
  uint8_t pin = getConfirmationPin(chamber_id);
  // Active LOW (pulled to ground when chamber opens)
  return (digitalRead(pin) == LOW);
}

uint8_t ActuationManager::getChamberPin(uint8_t chamber_id) {
  switch (chamber_id) {
    case 1: return CHAMBER_1_PIN;
    case 2: return CHAMBER_2_PIN;
    case 3: return CHAMBER_3_PIN;
    case 4: return CHAMBER_4_PIN;
    default: return 0;
  }
}

uint8_t ActuationManager::getConfirmationPin(uint8_t chamber_id) {
  switch (chamber_id) {
    case 1: return CHAMBER_1_CONFIRM_PIN;
    case 2: return CHAMBER_2_CONFIRM_PIN;
    default: return 0;  // Chambers 3-4 not yet implemented
  }
}

bool ActuationManager::isChamberOpen(uint8_t chamber_id) {
  if (chamber_id < 1 || chamber_id > 4) return false;
  return chamber_status_[chamber_id - 1].is_open;
}

bool ActuationManager::getChamberStatus(uint8_t chamber_id, ChamberStatus* status) {
  if (chamber_id < 1 || chamber_id > 4) return false;
  *status = chamber_status_[chamber_id - 1];
  return true;
}

uint8_t ActuationManager::getOpenChamberCount() {
  return open_chamber_count_;
}

void ActuationManager::resetAllChambers() {
  for (uint8_t i = 0; i < 4; i++) {
    chamber_status_[i].is_open = false;
    chamber_status_[i].trigger_depth_m = 0.0;
    chamber_status_[i].trigger_time_ms = 0;
    chamber_status_[i].retry_count = 0;
  }
  open_chamber_count_ = 0;
  DEBUG_PRINTLN("All chambers reset");
}

void ActuationManager::printStatus() {
  DEBUG_PRINTLN("===== Chamber Status =====");
  for (uint8_t i = 0; i < 4; i++) {
    ChamberStatus* s = &chamber_status_[i];
    DEBUG_PRINTF("Chamber %d: %s", s->chamber_id, s->is_open ? "OPEN" : "CLOSED");
    if (s->is_open) {
      DEBUG_PRINTF(" (%.2fm @ %lums)", s->trigger_depth_m, s->trigger_time_ms);
    }
    DEBUG_PRINTLN();
  }
  DEBUG_PRINTF("Total open: %d/4\n", open_chamber_count_);
  DEBUG_PRINTLN("==========================\n");
}

// ============================================================================
// DEPTH TRIGGER CONTROLLER
// ============================================================================

DepthTriggerController::DepthTriggerController(ActuationManager* actuation_manager)
  : actuation_manager_(actuation_manager),
    trigger_count_(0),
    last_depth_m_(0.0)
{
  clearTriggers();
}

bool DepthTriggerController::addTrigger(uint8_t chamber_id, float target_depth_m) {
  if (trigger_count_ >= 4) {
    DEBUG_PRINTLN("ERROR: Maximum triggers reached");
    return false;
  }
  
  triggers_[trigger_count_].chamber_id = chamber_id;
  triggers_[trigger_count_].target_depth_m = target_depth_m;
  triggers_[trigger_count_].executed = false;
  triggers_[trigger_count_].stable_count = 0;
  trigger_count_++;
  
  DEBUG_PRINTF("Added trigger: Chamber %d @ %.2fm\n", chamber_id, target_depth_m);
  return true;
}

uint8_t DepthTriggerController::processDepth(float current_depth_m) {
  uint8_t triggered_count = 0;
  
  for (uint8_t i = 0; i < trigger_count_; i++) {
    DepthTrigger* t = &triggers_[i];
    
    if (t->executed) continue;  // Already triggered
    
    // Check if depth is within tolerance
    float depth_error = abs(current_depth_m - t->target_depth_m);
    
    if (depth_error <= DEPTH_TOLERANCE_M) {
      t->stable_count++;
      
      // Require DEPTH_STABLE_COUNT consecutive stable readings
      if (t->stable_count >= DEPTH_STABLE_COUNT) {
        DEBUG_PRINTF("Depth stable at %.2fm - triggering chamber %d\n", 
          current_depth_m, t->chamber_id);
        
        ActuationResult result = actuation_manager_->triggerChamber(
          t->chamber_id, current_depth_m);
        
        if (result == ACT_SUCCESS) {
          t->executed = true;
          triggered_count++;
        }
      }
    } else {
      t->stable_count = 0;  // Reset stability counter
    }
  }
  
  last_depth_m_ = current_depth_m;
  return triggered_count;
}

void DepthTriggerController::clearTriggers() {
  trigger_count_ = 0;
  for (uint8_t i = 0; i < 4; i++) {
    triggers_[i].executed = false;
    triggers_[i].stable_count = 0;
  }
}

bool DepthTriggerController::allTriggersExecuted() {
  for (uint8_t i = 0; i < trigger_count_; i++) {
    if (!triggers_[i].executed) return false;
  }
  return trigger_count_ > 0;
}

void DepthTriggerController::printTriggers() {
  DEBUG_PRINTLN("===== Depth Triggers =====");
  for (uint8_t i = 0; i < trigger_count_; i++) {
    DepthTrigger* t = &triggers_[i];
    DEBUG_PRINTF("Chamber %d @ %.2fm: %s (stable: %d/%d)\n",
      t->chamber_id, t->target_depth_m,
      t->executed ? "DONE" : "PENDING",
      t->stable_count, DEPTH_STABLE_COUNT);
  }
  DEBUG_PRINTLN("==========================\n");
}
