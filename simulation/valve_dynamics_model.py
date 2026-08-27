#!/usr/bin/env python3
"""
@file valve_dynamics_model.py
@brief Compressible Fluid Choke Flow & Orifice Discharge Mathematical Engine
@repository t4rxnn-devel/hil-pneumatic-controller
@details Computes mass flow rates, upstream/downstream pressure ratios, and sonic 
          choking limits for high-pressure pneumatic solenoid valves.
"""

import math

class ValveDynamicsModel:
    def __init__(self, orifice_diameter_mm=2.5, upstream_temp_k=293.15):
        self.orifice_diameter_mm = orifice_diameter_mm
        self.upstream_temp_k = upstream_temp_k
        self.discharge_coefficient = 0.62
        
        # Calculate cross-sectional area of the valve orifice (m^2)
        radius_m = (orifice_diameter_mm / 2.0) * 1e-3
        self.flow_area_m2 = math.pi * (radius_m ** 2)

    def compute_mass_flow_rate(self, p_upstream_psi, p_downstream_psi) -> float:
        """
        Computes mass flow rate (kg/s) through an orifice under compressible 
        isentropic flow conditions (Nitrogen/Air gas medium).
        """
        # Convert PSI to Pascals (Absolute)
        p_up_pa = max(101325.0, p_upstream_psi * 6894.76)
        p_down_pa = max(101325.0, p_downstream_psi * 6894.76)

        # Ratio of specific heats for Air/Nitrogen
        gamma = 1.4
        gas_constant_r = 287.05  # J/(kg*K)

        pressure_ratio = p_down_pa / p_up_pa
        critical_pressure_ratio = (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))

        # Check for sonic choking condition
        if pressure_ratio <= critical_pressure_ratio:
            # Choked flow equation
            term1 = gamma * p_up_pa * self.flow_area_m2
            term2 = (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (gamma - 1.0))
            mass_flow = self.discharge_coefficient * term1 * math.sqrt(term2 / (gas_constant_r * self.upstream_temp_k))
        else:
            # Unchoked subsonic orifice flow equation
            term_expansion = ((2.0 * gamma) / (gamma - 1.0)) * \
                             ((pressure_ratio ** (2.0 / gamma)) - (pressure_ratio ** ((gamma + 1.0) / gamma)))
            if term_expansion < 0.0:
                term_expansion = 0.0
            mass_flow = self.discharge_coefficient * self.flow_area_m2 * p_up_pa * \
                        math.sqrt((2.0 / (gas_constant_r * self.upstream_temp_k)) * term_expansion)

        return mass_flow

    def set_orifice_size(self, diameter_mm: float):
        self.orifice_diameter_mm = diameter_mm
        radius_m = (diameter_mm / 2.0) * 1e-3
        self.flow_area_m2 = math.pi * (radius_m ** 2)
