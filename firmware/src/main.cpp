/**
 * @file main.cpp
 * @brief Core Embedded Real-Time Control Loop for HIL Pneumatic Burst Controller
 * @repository t4rxnn-devel/hil-pneumatic-controller
 * @details Manages 1000Hz fixed-interval execution, redundant pressure reading, 
 *          solenoid valve actuation state machines, and serial telemetry framing.
 */

#include <Arduino.h>
#include "safety_interlocks.h"

// Hardware Pin Definitions for Teensy 4.1 / ESP32 Target
#define PIN_PRESSURE_TRANSDUCER_A  A0
#define PIN_PRESSURE_TRANSDUCER_B  A1
#define PIN_SOLENOID_GATE_HIGH     2
#define PIN_CONTINUITY_SENSE       4
#define PIN_ARMING_KEY_SWITCH      7
#define PIN_LED_STATUS             13

// Control Loop Timing Constants (1000 Hz = 1000 microseconds)
const uint32_t CONTROL_LOOP_INTERVAL_US = 1000;
uint32_t last_loop_timestamp_us = 0;

// Global Safety Interlock Instance
SafetyInterlockManager safetyManager;

// Function Prototypes for Modular Execution
float readTransducerPSI(uint8_t pin);
void updateHardwareOutputs(SystemState_t state);
void transmitTelemetryPacket(float p1, float p2, SystemState_t state, uint16_t faults);

void setup() {
    // Initialize high-speed serial interface for HIL communication
    Serial.begin(500000);
    while (!Serial && millis() < 2000); // Wait for connection or timeout

    // Configure Pin IO Modes
    pinMode(PIN_SOLENOID_GATE_HIGH, OUTPUT);
    pinMode(PIN_CONTINUITY_SENSE, INPUT_PULLUP);
    pinMode(PIN_ARMING_KEY_SWITCH, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);

    // Ensure solenoids are driven LOW (safe state) on startup
    digitalWrite(PIN_SOLENOID_GATE_HIGH, LOW);
    digitalWrite(PIN_LED_STATUS, LOW);

    // Initialize Safety Interlock Registry
    safetyManager.initialize();
    last_loop_timestamp_us = micros();
}

void loop() {
    uint32_t current_time_us = micros();

    // Enforce strict deterministic 1000Hz fixed loop interval
    if ((current_time_us - last_loop_timestamp_us) >= CONTROL_LOOP_INTERVAL_US) {
        last_loop_timestamp_us += CONTROL_LOOP_INTERVAL_US;
        uint32_t current_time_ms = millis();

        // 1. Sample redundant analog pressure transducers
        float psi_a = readTransducerPSI(PIN_PRESSURE_TRANSDUCER_A);
        float psi_b = readTransducerPSI(PIN_PRESSURE_TRANSDUCER_B);

        // 2. Update physical switch and loop inputs
        bool key_state = (digitalRead(PIN_ARMING_KEY_SWITCH) == LOW); // Active low pullup
        bool continuity_state = (digitalRead(PIN_CONTINUITY_SENSE) == HIGH);

        safetyManager.setKeySwitch(key_state);
        safetyManager.updateContinuityStatus(continuity_state);
        safetyManager.checkWatchdog(current_time_ms);

        // 3. Evaluate safety voter logic across sensor streams
        bool sensors_valid = safetyManager.evaluateSensorVoter(psi_a, psi_b);

        // 4. Handle incoming serial bytes from the HIL simulator (watchdog feed)
        if (Serial.available() > 0) {
            char incoming_byte = Serial.read();
            if (incoming_byte == 'H') { // Heartbeat command token from HIL plant
                safetyManager.feedWatchdog(current_time_ms);
            }
        }

        // 5. Query system state and execute hardware output actions
        SystemState_t current_state = safetyManager.getState();
        
        if (current_state != SYSTEM_STATE_ABORT_FAULT && sensors_valid) {
            // Check transition conditions for hot arming if pressures are nominal
            if (key_state && continuity_state && psi_a >= 2200.0f) {
                // State machine progression handled via command triggers
            }
        }

        updateHardwareOutputs(current_state);

        // 6. Stream real-time telemetry back to the host computer / ground station
        transmitTelemetryPacket(psi_a, psi_b, current_state, safetyManager.getFaultFlags());
    }
}

// Converts raw 12-bit ADC millivolt readings into engineering units (PSI)
float readTransducerPSI(uint8_t pin) {
    int raw_adc = analogRead(pin);
    // Calibration formula: 0V to 3.3V mapped to 0 to 5000 PSI transducer range
    float voltage = (raw_adc / 4095.0f) * 3.3f;
    float calculated_psi = (voltage / 3.3f) * 5000.0f;
    return calculated_psi;
}

// Controls physical power gates based on validated safety state machine
void updateHardwareOutputs(SystemState_t state) {
    if (state == SYSTEM_STATE_FIRING_SEQUENCE) {
        digitalWrite(PIN_SOLENOID_GATE_HIGH, HIGH); // Open burst valve
        digitalWrite(PIN_LED_STATUS, HIGH);
    } else {
        digitalWrite(PIN_SOLENOID_GATE_HIGH, LOW);  // Default closed state
        // Flash status LED if in fault mode, solid if standby
        if (state == SYSTEM_STATE_ABORT_FAULT) {
            digitalWrite(PIN_LED_STATUS, (millis() / 250) % 2);
        } else {
            digitalWrite(PIN_LED_STATUS, LOW);
        }
    }
}

// Frames binary telemetry packets for high-speed serial link
void transmitTelemetryPacket(float p1, float p2, SystemState_t state, uint16_t faults) {
    Serial.print("T1:");
    Serial.print(p1, 2);
    Serial.print(",T2:");
    Serial.print(p2, 2);
    Serial.print(",ST:");
    Serial.print((uint8_t)state);
    Serial.print(",FL:");
    Serial.println(faults, HEX);
}
