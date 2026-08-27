#!/usr/bin/env python3
"""
@file run_hil_testbench.py
@brief Root-level Master Orchestrator for HIL Pneumatic Testbench
@repository t4rxnn-devel/hil-pneumatic-controller
@details Initializes the virtual plant simulation loop and validates serial ports.
"""

import sys
import os
import subprocess

def main():
    print("==================================================")
    print("  HIL PNEUMATIC BURST CONTROLLER - TESTBENCH RIG")
    print("==================================================")
    
    target_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    sim_script_path = os.path.join("simulation", "hil_plant_simulator.py")
    
    if not os.path.exists(sim_script_path):
        print(f"[ERROR] Could not find plant simulator at {sim_script_path}")
        sys.exit(1)

    print(f"[TESTBENCH] Launching plant simulator targeting port: {target_port}")
    try:
        subprocess.run(["python3", sim_script_path, target_port], check=True)
    except KeyboardInterrupt:
        print("\n[TESTBENCH] Testbench terminated by user.")
    except Exception as e:
        print(f"[TESTBENCH ERROR] Execution failed: {e}")

if __name__ == "__main__":
    main()
