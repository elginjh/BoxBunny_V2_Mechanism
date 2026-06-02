# BoxBunny V2 Mechanism: Project Handover Documentation

This document serves as the formal handover reference for the BoxBunny V2 Mechanism repository. It details the directory structure, critical design rationales, and essential system configurations required for subsequent development phases.

## 1. Directory Structure and Repository Navigation

### 1.1 Electrical and Firmware (`ARM_ELECTRICAL_HANDOFF/`)
This directory contains critical documentation and codebase for the control systems, firmware, and electrical routing.
- **`KNOWN_ISSUES_HANDOFF.md`**: Mandatory reading prior to system operation. It documents active hardware and electrical limitations (e.g., ODrive Regen Clamp resistor sizing, 3D printed gear structural limits) and historical defect resolutions to prevent regression. It also outlines essential architectural behaviors, such as Joint-Space Pitch Clamping and "Tactical Walking".
- **`teensy_firmware_V5/`**: Contains the production Teensy 4.0 firmware used for the 200 Hz control loop, CAN bus communication with Damiao motors, and I²C MPU6050 polling. Refer to the internal `README.md` for pin assignments, ROS 2 topics, and micro-ROS payload layouts.

### 1.2 Mechanical Design and CAD Models (`CAD Model/`)
Contains the physical design files and sub-assemblies for the BoxBunny mechanism.
- **`Final/FINAL ASSEM/`**: The primary directory for the overall CAD. Contains the full, latest assembly file (`FINAL ASSEM.asm`), systematically structured into subfolders corresponding to the various subsystems.
- **`Final/Bottom Assembly/v7_fab/`**: Contains the latest production bottom assembly files, logically organized into individual subfolders. The nested `report/` directory contains HTML documentation (`concept-generation.html`, `lift-structure-separation.html`) detailing the engineering rationale for the height adjustment subsystem.
- **`Interim/`**: Contains deprecated V2 iterations and reference sub-assemblies.
- *Note: Various `readme-and-terms-of-use-3d-cad-models.txt` files are included throughout this directory, providing licensing text for third-party COTS components sourced from PARTcommunity/3Dfindit.*

### 1.3 General Project Documentation and Reports (`General Project/`)
Contains legacy V1 project files, procurement details, and comprehensive engineering reports.
- **`Procurement/`**: Contains documentation for all purchased items claimed and selected for this project. Technical specifications are accessible via the provided purchase links or, if unavailable online, stored directly within the `additional tech specs` directory.
- **`Reports/Final Report/Handover - Lower Mechanism FINAL Detailed Reports/`**: Contains the comprehensive final engineering documentation detailing the design, analysis, and specifications for the height adjustment, rotation, and base mechanisms.
- **`Reports/Interim Report/report appendix b/jeanette appendix b.txt`**: A detailed text export of the engineering calculations and concept generation matrices. This is required reading for understanding:
  - **Upper Mechanism:** The justification for the 2DOF (pitch/yaw) actuator, including torque, inertia, and aerodynamic drag calculations for the training sticks.
  - **Lower Mechanism:** The rationale for decoupling the footwork motion into a linear rail (forward/backward) and a slewing bearing (rotation) over omnidirectional wheels, alongside motor sizing and selection matrices.
- **`V1 (EG3301R)/`**: Legacy files from the first iteration of the BoxBunny project.

## 2. Mandatory Pre-Operation Procedures

1. **Review Known Issues:** Prior to motor initialization, operators must review the ODrive Regen Clamp calibration procedures and I²C blocking mitigations detailed in `KNOWN_ISSUES_HANDOFF.md`.
2. **Firmware Verification:** Verify that the Teensy microcontroller is provisioned with **Firmware V5** (`teensy_firmware_V5.ino`). This build contains critical hardware I²C timeouts and failsafes for IMU disconnections to prevent uncontrolled motor oscillation.
