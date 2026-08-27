#!/usr/bin/env python3
"""
@file calibration_sweep.py
@brief Transducer ADC-to-PSI Benchtop Calibration Utility
@repository t4rxnn-devel/hil-pneumatic-controller
@details Performs multi-point linear regression and offset calibration for 
          high-pressure strain-gauge transducers.
"""

import sys
import time

class TransducerCalibrationTool:
    def __init__(self, target_max_psi=5000.0, reference_voltage=3.3):
        self.target_max_psi = target_max_psi
        self.reference_voltage = reference_voltage
        self.calibration_points = []

    def add_calibration_point(self, raw_adc_counts, known_reference_psi):
        voltage = (raw_adc_counts / 4095.0) * self.reference_voltage
        self.calibration_points.append((voltage, known_reference_psi))
        print(f"[CALIBRATION] Added point: {raw_adc_counts} ADC ({voltage:.3f}V) -> {known_reference_psi} PSI")

    def compute_linear_coefficients(self):
        if len(self.calibration_points) < 2:
            print("[CALIBRATION ERROR] Need at least 2 calibration points to compute slope and offset.")
            return None, None

        n = len(self.calibration_points)
        sum_v = sum(p[0] for p in self.calibration_points)
        sum_p = sum(p[1] for p in self.calibration_points)
        sum_vp = sum(p[0] * p[1] for p in self.calibration_points)
        sum_v2 = sum(p[0] ** 2 for p in self.calibration_points)

        # Least squares linear regression: PSI = slope * Voltage + intercept
        slope = (n * sum_vp - sum_v * sum_p) / (n * sum_v2 - (sum_v ** 2))
        intercept = (sum_p - slope * sum_v) / n

        print(f"\n[CALIBRATION RESULTS]")
        print(f"Calculated Slope (PSI/Volt): {slope:.4f}")
        print(f"Calculated Offset (PSI): {intercept:.4f}")
        return slope, intercept

if __name__ == "__main__":
    print("--- Transducer Calibration Utility ---")
    cal_tool = TransducerCalibrationTool()
    
    # Example mock calibration points (ADC counts vs Known Pressure Gauge PSI)
    cal_tool.add_calibration_point(raw_adc_counts=0, known_reference_psi=0.0)
    cal_tool.add_calibration_point(raw_adc_counts=2482, known_reference_psi=3000.0)
    cal_tool.add_calibration_point(raw_adc_counts=4095, known_reference_psi=5000.0)
    
    cal_tool.compute_linear_coefficients()
