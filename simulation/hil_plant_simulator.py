#!/usr/bin/env python3
"""
@file hil_plant_simulator.py
@brief Hardware-In-The-Loop (HIL) Pneumatic Plant & Thermodynamics Simulator
@repository t4rxnn-devel/hil-pneumatic-controller
@details Simulates real-time pressure decay, redundant sensor feedback streams,
          and bidirectional serial telemetry exchange with the embedded microcontroller.
"""

import sys
import time
import math
import serial

class HILPlantSimulator:
    def __init__(self, port, baudrate=500000, max_psi=5000.0):
        self.port = port
        self.baudrate = baudrate
        self.max_psi = max_psi
        self.current_psi_a = 3000.0  # Initial tank pressure
        self.current_psi_b = 3000.0  # Redundant channel
        self.valve_open = False
        self.running = False
        
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.01)
            print(f"[HIL SIMULATOR] Connected successfully to target on {self.port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"[HIL SIMULATOR ERROR] Failed to open serial port {self.port}: {e}")
            self.serial_conn = None

    def update_thermodynamics(self, dt_seconds):
        """
        Models adiabatic-like blowdown discharge through the valve orifice.
        P(t) = P_0 * exp(-lambda * t) when valve is energized open.
        """
        if self.valve_open:
            # Rapid pressure bleed down simulation
            discharge_coefficient = 0.35
            self.current_psi_a = max(0.0, self.current_psi_a - (self.current_psi_a * discharge_coefficient * dt_seconds))
            self.current_psi_b = max(0.0, self.current_psi_b - (self.current_psi_b * discharge_coefficient * dt_seconds))
        else:
            # Minor micro-leakage or steady standby hold
            ambient_bleed = 0.05 * dt_seconds
            self.current_psi_a = max(0.0, self.current_psi_a - ambient_bleed)
            self.current_psi_b = max(0.0, self.current_psi_b - ambient_bleed)

    def run_simulation_loop(self):
        if not self.serial_conn:
            print("[HIL SIMULATOR ERROR] Cannot run loop without an active serial connection.")
            return

        self.running = True
        last_time = time.time()
        heartbeat_timer = time.time()

        print("[HIL SIMULATOR] Starting HIL plant loop. Press Ctrl+C to exit.")
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # 1. Update plant physical model
                self.update_thermodynamics(dt)

                # 2. Feed hardware watchdog every 100ms via 'H' token
                if (current_time - heartbeat_timer) >= 0.1:
                    heartbeat_timer = current_time
                    self.serial_conn.write(b'H\n')

                # 3. Read incoming serial frames from MCU
                if self.serial_conn.in_waiting > 0:
                    raw_line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line:
                        print(f"[MCU TELEMETRY] {raw_line}")
                        # Simple command interception if MCU triggers solenoid
                        if "ST:4" in raw_line or "SOL:HIGH" in raw_line:
                            self.valve_open = True

                # Small sleep to cap CPU usage at ~1000Hz equivalent polling
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n[HIL SIMULATOR] Shutting down simulation cleanly...")
        finally:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            print("[HIL SIMULATOR] Terminated.")

if __name__ == "__main__":
    target_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    simulator = HILPlantSimulator(port=target_port)
    simulator.run_simulation_loop()
