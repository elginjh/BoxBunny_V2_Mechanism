# Teensy Firmware V5

**Target:** Teensy 4.0 | **IDE:** Arduino IDE + Teensyduino

## What's in this sketch

| Feature | Detail |
|---------|--------|
| Control loop | 200 Hz unified loop (5 ms period) |
| CAN bus | FlexCAN_T4, 1 Mbps, 4× Damiao DM-J4310-2EC (base IDs 0x01–0x04; command frames 0x101–0x104) |
| IMU | 4× MPU6050 over dual I²C bus (Wire + Wire1, 400 kHz) |
| Height motor | Cytron MDDS10 via PWM (Pin 3) + DIR (Pin 2) |
| ROS 2 | micro-ROS over USB; publishes 21-double `/motor_feedback` payload |

## Feedback payload layout

```
[0..3]  Motor positions (rad)
[4..7]  Motor currents (A)
[8]     CAN RX frame count
[9..20] IMU accel XYZ for each of 4 sensors (m/s²)
```

## Changelog

### V5 Release (I²C Blocking Fix & Failsafe)
- **Hardware I²C Timeouts:** Added `Wire.setWireTimeout(3000, true)` on both I²C buses to prevent the default blocking behavior. If an MPU6050 disconnects or hangs, the bus safely aborts the read after 3ms instead of freezing the 200Hz loop and starving the CAN bus.
- **Sensor State Failsafe:** Introduced a global `imu_healthy` flag that tracks consecutive sensor errors. If an IMU drops offline (e.g. `imu_err_count > 3`), the firmware actively triggers a software override that sets `motors_enabled = false`. This drops the robot into a safe limp state, preventing violent jitter caused by "garbage" 0-value sensor data.

### 2026-04-06 — IMU range fix
- **ACCEL_CONFIG** changed `0x00` (±2g) → `0x18` (±16g)  
- **Conversion divisor** changed `16384.0` → `2048.0`

**Reason:** Empirical readings exceeded 19.62 m/s² (the ±2g saturation ceiling), confirming the sensor was running at ±16g hardware default. The old divisor underscaled all IMU output by 8×.

**After reflashing:** Re-run IMU gravity calibration in the GUI (Calibration tab → Re-calibrate IMU). Strike detection thresholds will need re-tuning — physical strike values will read approximately 8× higher than before.

## Libraries required

- `micro_ros_arduino` (micro-ROS)
- `FlexCAN_T4` (CAN bus)
- `Wire` (built-in — I²C)

## Pin assignments

| Pin | Function |
|-----|---------|
| 3 | MDDS10 AN1 — PWM speed |
| 2 | MDDS10 DIG1 — direction |
| 18 / 19 | Wire SDA / SCL (IMU bus 1) |
| 17 / 16 | Wire1 SDA / SCL (IMU bus 2) |
| 13 | LED (micro-ROS status) |
