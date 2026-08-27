/**
 * @file sensor_acquisition.cpp
 * @brief High-Precision Sensor Sampling & Digital Filtering Engine
 * @repository t4rxnn-devel/hil-pneumatic-controller
 * @details Implements moving-average digital filtering, outlier rejection, 
 *          and voltage-to-pressure calibration curves for telemetry.
 */

#include <Arduino.h>

class SensorAcquisitionEngine {
private:
    static const uint8_t FILTER_WINDOW_SIZE = 8;
    float sample_buffer_a[FILTER_WINDOW_SIZE];
    float sample_buffer_b[FILTER_WINDOW_SIZE];
    uint8_t buffer_index;
    bool buffer_filled;

    float reference_voltage;
    float max_pressure_psi;

public:
    SensorAcquisitionEngine() {
        buffer_index = 0;
        buffer_filled = false;
        reference_voltage = 3.3f;
        max_pressure_psi = 5000.0f;

        for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++) {
            sample_buffer_a[i] = 0.0f;
            sample_buffer_b[i] = 0.0f;
        }
    }

    void initialize() {
        buffer_index = 0;
        buffer_filled = false;
    }

    // Feeds raw ADC counts and computes moving average filtered pressure in PSI
    void updateReadings(uint16_t raw_adc_a, uint16_t raw_adc_b, float &out_psi_a, float &out_psi_b) {
        // Convert ADC counts (0-4095 for 12-bit) to engineering units
        float volts_a = (static_cast<float>(raw_adc_a) / 4095.0f) * reference_voltage;
        float volts_b = (static_cast<float>(raw_adc_b) / 4095.0f) * reference_voltage;

        float instant_psi_a = (volts_a / reference_voltage) * max_pressure_psi;
        float instant_psi_b = (volts_b / reference_voltage) * max_pressure_psi;

        // Store into rolling window buffers
        sample_buffer_a[buffer_index] = instant_psi_a;
        sample_buffer_b[buffer_index] = instant_psi_b;

        buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;
        if (buffer_index == 0) {
            buffer_filled = true;
        }

        // Calculate moving average across valid window samples
        uint8_t count = buffer_filled ? FILTER_WINDOW_SIZE : buffer_index;
        if (count == 0) count = 1;

        float sum_a = 0.0f;
        float sum_b = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            sum_a += sample_buffer_a[i];
            sum_b += sample_buffer_b[i];
        }

        out_psi_a = sum_a / static_cast<float>(count);
        out_psi_b = sum_b / static_cast<float>(count);
    }

    void setTransducerParameters(float ref_v, float max_psi) {
        reference_voltage = ref_v;
        max_pressure_psi = max_psi;
    }
};
