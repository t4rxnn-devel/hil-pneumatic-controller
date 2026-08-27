/**
 * @file telemetry_serial.cpp
 * @brief High-Speed Serial Telemetry Framing & Command Parser Engine
 * @repository t4rxnn-devel/hil-pneumatic-controller
 * @details Encodes real-time sensor streams, system states, and fault registers 
 *          into structured binary/ASCII packets and decodes inbound control tokens.
 */

#include <Arduino.h>

class TelemetrySerialEngine {
private:
    static const uint8_t RX_BUFFER_SIZE = 64;
    char rx_buffer[RX_BUFFER_SIZE];
    uint8_t rx_buffer_index;
    unsigned long last_packet_sent_ms;
    uint32_t packet_sequence_number;

public:
    TelemetrySerialEngine() {
        rx_buffer_index = 0;
        last_packet_sent_ms = 0;
        packet_sequence_number = 0;
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
    }

    void initialize(unsigned long baud_rate) {
        // Serial port is typically initialized in main, but we configure internal states here
        rx_buffer_index = 0;
        packet_sequence_number = 0;
    }

    // Formats and streams structured telemetry frame over UART
    void sendTelemetryFrame(float p1, float p2, uint8_t system_state, uint16_t fault_flags, float battery_volts) {
        unsigned long current_ms = millis();
        // Rate limit telemetry broadcast to every 20ms (50 Hz) to avoid serial saturation
        if ((current_ms - last_packet_sent_ms) >= 20) {
            last_packet_sent_ms = current_ms;
            packet_sequence_number++;

            Serial.print("$PKT,");
            Serial.print(packet_sequence_number);
            Serial.print(",");
            Serial.print(current_ms);
            Serial.print(",");
            Serial.print(p1, 2);
            Serial.print(",");
            Serial.print(p2, 2);
            Serial.print(",");
            Serial.print(system_state);
            Serial.print(",0x");
            Serial.print(fault_flags, HEX);
            Serial.print(",");
            Serial.print(battery_volts, 2);
            Serial.println("*CS"); // Placeholder for checksum termination
        }
    }

    // Non-blocking parser for inbound commands from the HIL simulator or ground station
    bool pollIncomingCommands(char &out_command_token) {
        while (Serial.available() > 0) {
            char incoming = Serial.read();
            if (incoming == '\n' || incoming == '\r') {
                if (rx_buffer_index > 0) {
                    out_command_token = rx_buffer[0];
                    rx_buffer_index = 0;
                    memset(rx_buffer, 0, RX_BUFFER_SIZE);
                    return true;
                }
            } else if (rx_buffer_index < (RX_BUFFER_SIZE - 1)) {
                rx_buffer[rx_buffer_index++] = incoming;
            } else {
                // Buffer overflow protection: reset index
                rx_buffer_index = 0;
                memset(rx_buffer, 0, RX_BUFFER_SIZE);
            }
        }
        return false;
    }

    uint32_t getSequenceNumber() const {
        return packet_sequence_number;
    }
};
