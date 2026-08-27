#!/usr/bin/env python3
"""
@file advanced_thermal_electrodynamics.py
@brief Advanced Polytropic Thermal & Solenoid Coil Electrodynamic Extension
@repository t4rxnn-devel/hil-pneumatic-controller
@details Add-on physics module simulating real gas expansion cooling (polytropic index n=1.3)
          and coil thermal resistance shift due to Joule heating without altering core files.
"""

import math

class AdvancedThermalElectrodynamics:
    def __init__(self, initial_temp_k=293.15, coil_resistance_ohm=12.0):
        self.gas_temp_k = initial_temp_k
        self.coil_resistance_ohm = coil_resistance_ohm
        self.polytropic_index = 1.3  # Real nitrogen/air expansion characteristic
        self.ambient_temp_k = initial_temp_k

    def compute_polytropic_temperature_drop(self, initial_pressure_psi, current_pressure_psi) -> float:
        """
        Computes real-time gas temperature inside the reservoir during blowdown
        using the polytropic relation: T_2 = T_1 * (P_2 / P_1)^((n-1)/n)
        """
        if initial_pressure_psi <= 0.0 or current_pressure_psi <= 0.0:
            return self.ambient_temp_k

        pressure_ratio = max(0.001, current_pressure_psi / initial_pressure_psi)
        exponent = (self.polytropic_index - 1.0) / self.polytropic_index
        self.gas_temp_k = self.ambient_temp_k * (pressure_ratio ** exponent)
        return self.gas_temp_k

    def compute_coil_thermal_resistance(self, current_amps, duration_seconds) -> float:
        """
        Computes copper winding resistance shift due to internal Joule heating:
        R(T) = R_0 * (1 + alpha * delta_T)
        """
        copper_temp_coefficient = 0.00393 # 1/°C
        power_dissipated_watts = (current_amps ** 2) * self.coil_resistance_ohm
        # Simple thermal mass lumped capacitance heat accumulation
        heat_capacity_j_k = 45.0 
        delta_t_celsius = (power_dissipated_watts * duration_seconds) / heat_capacity_j_k
        
        effective_resistance = self.coil_resistance_ohm * (1.0 + (copper_temp_coefficient * delta_t_celsius))
        return effective_resistance
