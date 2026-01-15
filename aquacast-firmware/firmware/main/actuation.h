/**
 * @file actuation.h
 * @brief Chamber actuation control for water sampling
 * 
 * This module handles the triggering of sampling chambers at specific depths.
 * Supports both magnetic actuation and servo-based mechanisms.
 * 
 * @author Victoria Yang (Firmware), Shareef Jasim (Mechanical Design)
 * @date 2025-01-14
 * @version 0.1.0
 */

#ifndef ACTUATION_H
#define ACTUATION_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @enum ActuationType
 * @brief Type of actuation mechanism used
 */
enum ActuationType {
  ACTUATION_MAGNETIC,      // Magnetic repulsion-based
  ACTUATION_SERVO,         // Servo motor-based
  ACTUATION_SOLENOID       // Solenoid valve (future)
};

/**
 * @enum ActuationResult
 * @brief Result codes for actuation operations
 */
enum ActuationResult {
  ACT_SUCCESS,             // Chamber actuated successfully
  ACT_TIMEOUT,             // No confirmation received within timeout
  ACT_ALREADY_OPEN,        // Chamber was already open
  ACT_INSUFFICIENT_POWER,  // Battery voltage too low
  ACT_MECHANICAL_FAILURE,  // Mechanism jammed or failed
  ACT_INVALID_CHAMBER      // Chamber ID out of range
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct ChamberStatus
 * @brief Status of a single sampling chamber
 */
struct ChamberStatus {
  uint8_t chamber_id;           // Chamber identifier (1-4)
  bool is_open;                 // True if chamber has been actuated
  float trigger_depth_m;        // Depth at which chamber was triggered
  uint32_t trigger_time_ms;     // Timestamp of actuation
  ActuationResult last_result;  // Result of last actuation attempt
  uint8_t retry_count;          // Number of retry attempts
};

// ============================================================================
// ACTUATION MANAGER CLASS
// ============================================================================

/**
 * @class ActuationManager
 * @brief Manages all chamber actuation operations
 * 
 * Handles triggering logic, confirmation checking, and retry mechanisms
 * for all sampling chambers.
 */
class ActuationManager {
public:
  /**
   * @brief Constructor
   * @param type Actuation mechanism type
   */
  ActuationManager(ActuationType type = ACTUATION_MAGNETIC);

  /**
   * @brief Initialize actuation system
   * @return true if initialization successful
   */
  bool begin();

  /**
   * @brief Trigger a specific chamber
   * @param chamber_id Chamber number (1-4)
   * @return ActuationResult status code
   */
  ActuationResult triggerChamber(uint8_t chamber_id);

  /**
   * @brief Trigger chamber with depth metadata
   * @param chamber_id Chamber number
   * @param depth_m Current depth in meters
   * @return ActuationResult status code
   */
  ActuationResult triggerChamber(uint8_t chamber_id, float depth_m);

  /**
   * @brief Check if chamber has been actuated
   * @param chamber_id Chamber number (1-4)
   * @return true if chamber is open
   */
  bool isChamberOpen(uint8_t chamber_id);

  /**
   * @brief Get status of a specific chamber
   * @param chamber_id Chamber number (1-4)
   * @param status Pointer to ChamberStatus to fill
   * @return true if chamber ID valid
   */
  bool getChamberStatus(uint8_t chamber_id, ChamberStatus* status);

  /**
   * @brief Get number of chambers currently open
   * @return Count of open chambers
   */
  uint8_t getOpenChamberCount();

  /**
   * @brief Reset all chamber states (for testing)
   */
  void resetAllChambers();

  /**
   * @brief Run self-test of actuation mechanism
   * @return true if self-test passed
   */
  bool runSelfTest();

  /**
   * @brief Print actuation status to serial
   */
  void printStatus();

private:
  // Private actuation functions
  bool sendActuationSignal(uint8_t chamber_id);
  bool waitForConfirmation(uint8_t chamber_id);
  bool checkConfirmationSensor(uint8_t chamber_id);
  uint8_t getChamberPin(uint8_t chamber_id);
  uint8_t getConfirmationPin(uint8_t chamber_id);
  
  // Private member variables
  ActuationType actuation_type_;
  ChamberStatus chamber_status_[4];  // Status for chambers 1-4
  uint8_t open_chamber_count_;
  bool initialized_;
};

// ============================================================================
// DEPTH TRIGGER CONTROLLER
// ============================================================================

/**
 * @class DepthTriggerController
 * @brief Manages depth-based chamber triggering logic
 * 
 * Monitors depth readings and triggers chambers when target depths
 * are reached, with stability checking to avoid false triggers.
 */
class DepthTriggerController {
public:
  /**
   * @brief Constructor
   * @param actuation_manager Pointer to ActuationManager
   */
  DepthTriggerController(ActuationManager* actuation_manager);

  /**
   * @brief Add a depth trigger point
   * @param chamber_id Chamber to trigger
   * @param target_depth_m Target depth in meters
   * @return true if trigger added successfully
   */
  bool addTrigger(uint8_t chamber_id, float target_depth_m);

  /**
   * @brief Process current depth reading
   * @param current_depth_m Current depth in meters
   * @return Number of chambers triggered this cycle
   */
  uint8_t processDepth(float current_depth_m);

  /**
   * @brief Clear all trigger points
   */
  void clearTriggers();

  /**
   * @brief Check if all triggers have been executed
   * @return true if all chambers triggered
   */
  bool allTriggersExecuted();

  /**
   * @brief Print trigger status
   */
  void printTriggers();

private:
  struct DepthTrigger {
    uint8_t chamber_id;
    float target_depth_m;
    bool executed;
    uint8_t stable_count;  // Consecutive readings within tolerance
  };

  ActuationManager* actuation_manager_;
  DepthTrigger triggers_[4];
  uint8_t trigger_count_;
  float last_depth_m_;
};

#endif // ACTUATION_H
