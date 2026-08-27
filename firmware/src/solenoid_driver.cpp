/**
 * @file solenoid_driver.cpp
 * @brief High-Side Solenoid Valve Actuation & PWM Current Management Engine
 * @repository t4rxnn-devel/hil-pneumatic-controller
 * @details Manages MOSFET gate drive signals, inductive spike suppression, 
 *          and high-current switching for high-pressure pneumatic pilot valves.
 */

#include <Arduino.h>

// Solenoid Driver Configuration Constants
const uint8_t SOLENOID_PWM_PIN = 2;
const uint8_t FAULT_LATCH_PIN = 5;
const uint32_t MAX_ACTIVATION_DURATION_MS = 5000; // Hard cap on continuous valve open time

class SolenoidDriver {
private:
    bool is_energized;
    uint32_t activation_start_timestamp;
    uint8_t current_pwm_duty_cycle;
    bool hardware_fault_latched;

public:
    SolenoidDriver() {
        is_energized = false;
        activation_start_timestamp = 0;
        current_pwm_duty_cycle = 255; // Default full pull-in voltage (100% duty)
        hardware_fault_latched = false;
    }

    void initialize(uint8_t pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        is_energized = false;
        hardware_fault_latched = false;
    }

    // Energizes the high-pressure pneumatic solenoid valve with optional hold current reduction
    bool energizeValve(uint32_t current_time_ms) {
        if (hardware_fault_latched) {
            return false; // Refuse activation if a hardware fault is latched
        }

        if (!is_energized) {
            is_energized = true;
            activation_start_timestamp = current_time_ms;
            analogWrite(SOLENOID_PWM_PIN, current_pwm_duty_cycle);
            return true;
        }

        // Safety timeout check: prevent coil burnout or continuous unseated flow
        if ((current_time_ms - activation_start_timestamp) > MAX_ACTIVATION_DURATION_MS) {
            emergencyShutdown("Solenoid max continuous activation window breached!");
            return false;
        }

        // Transition from high pull-in voltage to lower hold current after 100ms
        if ((current_time_ms - activation_start_timestamp) > 100) {
            // Drop duty cycle to 60% for thermal preservation of the valve coil
            analogWrite(SOLENOID_PWM_PIN, 153); 
        }

        return true;
    }

    // Instantly de-energizes the valve to close the pneumatic pathway
    void deEnergizeValve() {
        is_energized = false;
        analogWrite(SOLENOID_PWM_PIN, 0);
        digitalWrite(SOLENOID_PWM_PIN, LOW);
    }

    void emergencyShutdown(const char* reason) {
        is_energized = false;
        hardware_fault_latched = true;
        analogWrite(SOLENOID_PWM_PIN, 0);
        digitalWrite(SOLENOID_PWM_PIN, LOW);
        digitalWrite(FAULT_LATCH_PIN, HIGH); // Trip hardware latch indicator
    }

    bool isEnergized() const {
        return is_energized;
    }

    bool isFaultLatched() const {
        return hardware_fault_latched;
    }

    void resetFaultLatch() {
        hardware_fault_latched = false;
        digitalWrite(FAULT_LATCH_PIN, LOW);
    }
};
