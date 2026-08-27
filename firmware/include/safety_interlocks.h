/**
 * @file safety_interlocks.h
 * @brief Advanced Hardware-In-The-Loop (HIL) Safety Interlock & Voting System
 * @repository t4rxnn-devel/hil-pneumatic-controller
 * @details Implements redundant pressure transducer cross-checking, continuous arming 
 *          loop continuity validation, and deterministic fail-safe abort triggers.
 */

#ifndef SAFETY_INTERLOCKS_H
#define SAFETY_INTERLOCKS_H

#include <stdint.h>
#include <stdbool.h>

// System Operating States
typedef enum {
    SYSTEM_STATE_INIT = 0x00,
    SYSTEM_STATE_STANDBY = 0x01,
    SYSTEM_STATE_ARMED_PRE_FLIGHT = 0x02,
    SYSTEM_STATE_ARMED_HOT = 0x03,
    SYSTEM_STATE_FIRING_SEQUENCE = 0x04,
    SYSTEM_STATE_ABORT_FAULT = 0xFF
} SystemState_t;

// Fault Code Bitmask Registry
typedef enum {
    FAULT_NONE                  = 0x0000,
    FAULT_DUAL_TRANSDUCER_MISMATCH = 0x0001,
    FAULT_LOOP_CONTINUITY_LOST  = 0x0002,
    FAULT_OVER_PRESSURE_SPIKE   = 0x0004,
    FAULT_UNDER_PRESSURE_DROP   = 0x0008,
    FAULT_WATCHDOG_TIMEOUT      = 0x0010,
    FAULT_MANUAL_ABORT_TRIGGERED = 0x0020
} FaultCode_t;

// Configuration Parameters for Interlock Tolerances
typedef struct {
    float max_allowable_psi;
    float min_launch_psi;
    float max_transducer_delta_psi;
    uint32_t watchdog_timeout_ms;
} InterlockConfig_t;

class SafetyInterlockManager {
private:
    SystemState_t current_state;
    uint16_t active_fault_flags;
    InterlockConfig_t config;
    uint32_t last_heartbeat_timestamp;
    bool hardware_key_switched;
    bool continuity_pin_high;

public:
    SafetyInterlockManager() {
        current_state = SYSTEM_STATE_INIT;
        active_fault_flags = FAULT_NONE;
        last_heartbeat_timestamp = 0;
        hardware_key_switched = false;
        continuity_pin_high = false;
        
        // Default safe thresholds for high-pressure pneumatic reservoirs
        config.max_allowable_psi = 4500.0f;
        config.min_launch_psi = 2200.0f;
        config.max_transducer_delta_psi = 150.0f;
        config.watchdog_timeout_ms = 250;
    }

    void initialize() {
        current_state = SYSTEM_STATE_STANDBY;
        active_fault_flags = FAULT_NONE;
    }

    // Evaluates dual independent pressure transducers for cross-consistency
    bool evaluateSensorVoter(float pressure_sensor_a, float pressure_sensor_b) {
        float delta = pressure_sensor_a > pressure_sensor_b ? 
                      (pressure_sensor_a - pressure_sensor_b) : 
                      (pressure_sensor_b - pressure_sensor_a);

        if (delta > config.max_transducer_delta_psi) {
            active_fault_flags |= FAULT_DUAL_TRANSDUCER_MISMATCH;
            triggerAbort("Transducer divergence detected beyond safety tolerance!");
            return false;
        }

        if (pressure_sensor_a > config.max_allowable_psi || pressure_sensor_b > config.max_allowable_psi) {
            active_fault_flags |= FAULT_OVER_PRESSURE_SPIKE;
            triggerAbort("Critical over-pressure threshold breached!");
            return false;
        }

        return true;
    }

    // Validates physical continuity of the pyrotechnic / solenoid firing loop
    void updateContinuityStatus(bool is_loop_closed) {
        continuity_pin_high = is_loop_closed;
        if (!continuity_pin_high && (current_state == SYSTEM_STATE_ARMED_HOT || current_state == SYSTEM_STATE_FIRING_SEQUENCE)) {
            active_fault_flags |= FAULT_LOOP_CONTINUITY_LOST;
            triggerAbort("Firing loop continuity dropped unexpectedly during active phase!");
        }
    }

    // Heartbeat check to ensure host controller or HIL simulator is alive
    void feedWatchdog(uint32_t current_timestamp_ms) {
        last_heartbeat_timestamp = current_timestamp_ms;
    }

    void checkWatchdog(uint32_t current_timestamp_ms) {
        if ((current_timestamp_ms - last_heartbeat_timestamp) > config.watchdog_timeout_ms) {
            active_fault_flags |= FAULT_WATCHDOG_TIMEOUT;
            triggerAbort("HIL communication watchdog expired!");
        }
    }

    void setKeySwitch(bool state) {
        hardware_key_switched = state;
        if (!hardware_key_switched && current_state > SYSTEM_STATE_STANDBY) {
            triggerAbort("Physical safety arming key pulled!");
        }
    }

    void triggerAbort(const char* reason) {
        current_state = SYSTEM_STATE_ABORT_FAULT;
        // Hardware level failsafe hook would cut power gates here
    }

    SystemState_t getState() const {
        return current_state;
    }

    uint16_t getFaultFlags() const {
        return active_fault_flags;
    }

    bool isSystemSafeToFire(float p1, float p2) {
        if (current_state != SYSTEM_STATE_ARMED_HOT) return false;
        if (active_fault_flags != FAULT_NONE) return false;
        if (!hardware_key_switched) return false;
        if (!continuity_pin_high) return false;
        if (p1 < config.min_launch_psi || p2 < config.min_launch_psi) return false;
        return true;
    }
};

#endif // SAFETY_INTERLOCKS_H
