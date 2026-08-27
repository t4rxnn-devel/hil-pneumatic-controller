# 🧨 HIL Pneumatic Burst Controller & Thermal Rig

> *"Because venting 5,000 PSI of high-pressure nitrogen requires slightly more precision than a casual shrug."*

Welcome to the **HIL Pneumatic Burst Controller**, an industrial-grade, deterministic hardware-in-the-loop (HIL) testbench and embedded controller designed for high-pressure pneumatic systems, high-speed solenoid actuation, and failsafe safety interlocking. 

---

## 🛠️ Repository Architecture

Our directory layout is cleaner than a fresh pressure vessel:
* **`firmware/`**: Real-time embedded C++ firmware running on a Teensy 4.1, engineered around a strict 1000Hz fixed-interval execution loop.
* **`simulation/`**: Python-based digital twins modeling compressible gas thermodynamics, sonic choke flow, and polytropic cooling.
* **`scripts/`**: Calibration utilities for strain-gauge pressure transducers.
* **`.github/workflows/`**: Automated CI/CD pipelines ensuring your code never compiles with a prayer.

---

## 🔬 The Theory: Why Things Don't Explode

Building a pneumatic burst controller means living at the intersection of fluid dynamics, electrical engineering, and sheer survival instinct. Here is how the math keeps us safe:

### 1. 1000Hz Deterministic Loop Timing (`main.cpp`)
Real-time control doesn't care about your computer's feelings or garbage collection cycles. We enforce a rigid 1-millisecond tick rate using microsecond-precision hardware clocks (`micros()`). If a sensor read or voter check takes too long, the system knows immediately.

### 2. Redundant Sensor Voting (`safety_interlocks.h`)
We never trust a single analog sensor because Murphy's Law was written for transducers. The system samples two independent channels (**Transducer A** and **Transducer B**). If the pressure delta between the two streams drifts beyond safe tolerances, the system trips an immediate hardware abort.

### 3. Compressible Choke Flow & Polytropic Cooling (`valve_dynamics_model.py` & `thermal_electrodynamics.py`)
When high-pressure gas rushes through an orifice, it doesn't just hiss—it undergoes adiabatic expansion. As pressure drops, gas temperature plummets according to the polytropic relation:
$$T_2 = T_1 \left(\frac{P_2}{P_1}\right)^{\frac{n-1}{n}}$$
Where index $n = 1.3$ models real nitrogen behavior. If you ignore this, your seals freeze, your metal embrittles, and your pressure calculations become creative fiction. Furthermore, continuous solenoid activation induces Joule heating in the copper coils ($I^2R$), shifting winding resistance. We track both thermal drop and coil heating in real-time.

---

## ❓ Frequently Asked Questions (FAQs)

### Q: Why Teensy 4.1?
**A:** Because blinking an LED on a standard Arduino is fun, but processing dual-channel 12-bit ADC feeds, running 1000Hz safety voter state machines, and parsing high-speed binary telemetry streams at 500,000 baud requires actual compute muscle without operating system jitter.

### Q: What happens if the Python simulator loses connection to the microcontroller?
**A:** A hardware watchdog timer watches the serial pipe. If the HIL plant simulator stops feeding the heartbeat token (`'H'`) every 100ms, the MCU assumes the host computer caught fire, safely drops the solenoid gate pin LOW, and slams the system into a safe abort state.

### Q: Can I run this without physical hardware connected?
**A:** Absolutely. That is the entire point of a Hardware-In-The-Loop (HIL) testbench. Fire up `python3 run_hil_testbench.py` and let the digital twin simulate tank blowdown thermodynamics right in your local terminal.

---

## 🚀 Quick Start

1. **Clone the repo:**
   ```bash
   git clone [https://github.com/t4rxnn-devel/hil-pneumatic-controller.git](https://github.com/t4rxnn-devel/hil-pneumatic-controller.git)
   cd hil-pneumatic-controller
   ```
2. **Build the firmware:**
   ```bash
   cd firmware/
   pio run
   ```
3. **Launch the HIL testbench simulation:**
   ```bash
   python3 run_hil_testbench.py
   ```
