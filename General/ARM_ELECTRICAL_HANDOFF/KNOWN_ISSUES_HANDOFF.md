# BoxBunny Project: Known Issues & Troubleshooting Handoff

## Overview

This document serves as a technical handoff for the next development phase of the BoxBunny project, specifically focusing on the Upper Mechanism, Padding, and Height Adjustment subsystems. It outlines active limitations, mitigated risks requiring hardware upgrades, and a historical log of resolved defects to prevent regression.

---

## 1. Active Known Issues & Mitigations (Action Required)

### 1.1 ODrive Regen Clamp: Threshold Calibration & Resistor Validation
- **Layer:** Electrical (Power)
- **Status:** ⚠️ Active Issue (Calibration Fixed; Dynamic Sizing Pending)
- **Historical Symptom:** The ODrive power resistor was actively heating up and smoking even when the motors were completely stationary.
- **Diagnosed Root Cause (Resolved):** The clamping threshold potentiometer on the Regen Clamp V0.3 was uncalibrated and set below the nominal 24V bus voltage. This caused a continuous "tug-of-war" where the clamp was permanently open, actively dumping the main power supply's continuous current directly into the brake resistor, treating it as a dead short.
- **Handoff Action (Calibration Rule):** The threshold has been corrected. For all future tuning, the Regen Clamp must be calibrated under power but completely idle. The potentiometer must be turned until the blue indicator light turns off, ensuring the threshold (e.g., 26.5V) sits safely above the power supply's nominal output.
- **Handoff Action (Dynamic Hardware Sizing):** While the continuous smoking issue was resolved via calibration, the current test resistors (10W/25W) are still mathematically undersized for the kinetic energy of a dynamic sparring load. The next team must calculate the peak regenerative Joules for both the Base Rotation and Height Adjustment axes. Heavy-duty Aluminium Wirewound Resistors (e.g., 100W+) must be sourced and bolted to the metal chassis for thermal mass before full-speed kinematic testing resumes.
### 1.2 I²C Blocking Trap & CAN Bus Starvation

- **Layer:** Firmware (I²C / CAN interaction) & GUI
- **Status:** ⚠️ Active Issue (Firmware Flash & GUI Updates Pending)
- **Symptom:** When an MPU6050 sensor fails or disconnects, the Teensy freezes while waiting for an I²C timeout. This starves the Damiao motors of their 200Hz trajectory updates, causing violent motor stuttering and feeding zeroed "garbage" sensor data into the upstream control logic.
- **Root Cause:** The default Arduino `Wire` library uses blocking calls (`requestFrom`), meaning IMU polling failure directly blocks CAN transmission.
- **Current Mitigation:** A new **Teensy Firmware V5** (`teensy_firmware_V5.ino`) has been written. It implements a 3ms hardware I²C timeout (`Wire.setWireTimeout(3000, true)`) and a global `imu_healthy` flag to safely drop the robot into a limp mode on sensor failure.
- **Handoff Action:** Future teams **must flash the Teensy with Firmware V5**. Additionally, the **GUI must be updated and calibrated** to properly support this change (e.g., catching the limp state flag if `imu_healthy` fails, and showing a user-facing alert instead of continuing to send commands).

### 1.3 Structural Failures on 3D Printed Gears

- **Layer:** Mechanical Design
- **Status:** ⚠️ Active Issue (Hardware Upgrades Pending)
- **Symptom:** The current 3D printed gears are experiencing structural failures under operational loads.
- **Handoff Action:**
  1. **Short-Term Mitigation:** Reinforce the existing 3D printed gears with metal shafts to better distribute shear stress.
  2. **Long-Term Resolution:** Move towards fully metal gears with proper machined parts for durable, long-term reliability.

---

## 2. Resolved Defects (Regression Watchlist)

These defects were successfully resolved but are documented here to prevent future teams from inadvertently reverting the fixes during refactoring.

### 2.1 I²C Bus Hang on Long-Wire IMU Routing (Padding)

- **Layer:** Firmware (Electrical → Software boundary)
- **Root Cause:** Long unshielded jumper wires (30–40 cm) to the padding IMUs introduced parasitic capacitance. At 400 kHz Fast Mode, the I²C "Repeated Start" condition failed, causing the MPU6050 to latch the SDA line low and freeze the entire bus.
- **Resolution:** Implemented a **Hard STOP protocol** (`endTransmission(true)`) after every register write. This forces a full STOP + START sequence, resetting the MPU6050 state machine reliably. This adds ~20 µs of latency per read but guarantees bus stability.

### 2.2 Nyquist Blind-Spot in Strike Detection (Padding)

- **Layer:** Application (GUI software)
- **Root Cause:** The GUI refreshed at 20 Hz, but a punch impulse lasts ~30 ms. The original logic only checked the _latest_ sample at each GUI tick, completely missing inter-frame strikes (Nyquist blind-spot).
- **Resolution:** Upgraded to **Scan-window peak detection** (`np.max(mag_arr[-n_scan:])`), which evaluates all buffered samples (~10 samples at 200 Hz firmware rate) since the last GUI tick.

### 2.3 Motor Jitter & Boomerang Effect (Arm Actuation)

- **Layer:** Middleware (ROS 2) & Firmware
- **Root Cause:** Publishing CAN commands at the full 50 Hz ROS callback rate continuously reset the Damiao motor's internal trajectory planner, preventing smooth motion. Additionally, ghost GUI nodes published conflicting zero-targets.
- **Resolution:**
  1. Implemented **Sparse Edge-Triggering**: CAN commands are only published on position changes >0.01 rad or every 100 ms as a keep-alive.
  2. Pre-launch cleanup routine (`killall -9 python3`) added to kill ghost ROS nodes.

### 2.4 Micro-ROS Agent Disconnect Hang (Firmware)

- **Layer:** Firmware (micro-ROS transport)
- **Root Cause:** WSL USB passthrough drivers failed to signal DTR/RTS teardown when the host port closed, leaving the Teensy permanently hung in an `AGENT_CONNECTED` state.
- **Resolution:** Implemented a 1 Hz active ping. After 3 failed pings (1.5 s), the firmware triggers a **hardware silicon reset** (`SCB_AIRCR = 0x05FA0004`) for a clean reconnection state.

### 2.5 Phantom 8–9A Idle Current (Firmware)

- **Layer:** Firmware (CAN parsing)
- **Root Cause:** The Damiao motor encodes torque as a 12-bit integer, but the firmware parsed it as a 16-bit float, absorbing the adjacent temperature byte and causing wild current readings.
- **Resolution:** Corrected the CAN unpacking logic to cleanly extract the 12-bit MIT Torque field.

### 2.6 MDDS10 PWM/DIR Pin Transposition (Firmware)

- **Layer:** Firmware + Documentation
- **Root Cause:** Original wiring documentation transposed the AN1 (speed) and DIG1 (direction) labels, causing the motor to run at constant full speed while ignoring commands.
- **Resolution:** Swapped pins in firmware; wiring documentation has been corrected.

---

## 3. Important Architectural Quirks

### 3.1 Joint-Space Pitch Clamping

Because of the coaxial differential coupling, simple per-motor position clamping is impossible. Any roll rotation physically drives the pitch mechanism. To prevent the arm from breaking its structural limits during compound movements, limits are enforced via a **joint-space clamping pass** in `homing_tab.py`. Motor targets are converted to joint space, pitch is clamped, and then converted back to motor commands. **Do not attempt to implement basic motor-space clamping.**

### 3.2 Tactical Walking

The bevel gear "walking" effect (where Motor 1 and Motor 2 rotating in opposition produces a rearing motion) is an **intentional choreographic asset**. It is used purposefully in strike patterns like the "Jab Windup" to clear the user's defensive perimeter. It should be treated as a feature of the 4-quadrant actuation space, not a bug.
