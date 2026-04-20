# MARLINv1

**Autonomous Surface Vehicle** — first-generation prototype built for extended, proactive drowning detection in volatile aquatic environments.

---

## Overview

MARLINv1 is a custom-built autonomous surface vehicle (ASV) designed to patrol open water, detect potential drowning events, and operate continuously without human intervention. It runs on MicroPython on an ESP32-S3, using a PCA9685 PWM driver to control dual ESC-driven thrusters, a panning camera gimbal via stepper motor, and an onboard ESP32-CAM for video recording.

The system supports three operational modes — **Manual**, **Hover**, and **Autonomous** — switchable via an RC receiver over iBus. In autonomous mode, the vehicle can replay a previously recorded path or navigate by compass heading, all while logging telemetry and scanning its environment.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32-S3 |
| PWM Driver | PCA9685 (I2C @ 0x40) |
| IMU | MPU6050 (I2C @ 0x68) |
| Magnetometer | HMC5883L (I2C @ 0x1E) |
| GPS | NMEA over UART1 (TX=17, RX=18) |
| RC Receiver | iBus over UART2 (TX=36, RX=35) |
| Camera | ESP32-CAM (WiFi AP, IP: 192.168.4.1) |
| Stepper Driver | Step/Dir/En on GPIO 7/6/5 |
| Stepper Encoder | Quadrature on GPIO 10/11 |
| LED Indicator | GPIO 48 |
| Left ESC | PCA9685 Channel 1 |
| Right ESC | PCA9685 Channel 3 |
| Aux Motor | PCA9685 Channel 4 |

---

## RC Channel Mapping

| Channel | Function | Values |
|---|---|---|
| CH3 | Throttle | 1000–2000 |
| CH4 | Steering | 1000–2000 |
| CH5 | Path Planning | ≥1900 = record, ≤1100 = save |
| CH7 | Mode Select | ≤1100 = auto, ~1500 = hover, ≥1900 = manual |
| CH8 | Camera Record | ≥1900 = record ON |
| CH10 | Motor Trim | 1000–2000 (maps to ±500µs bias) |

> All mode and path planning switches require a **1-second hold** to activate, preventing accidental state changes.

---

## Operational Modes

### Manual
Full RC control. Left/right throttle mixing applied from CH3 (throttle) and CH4 (steering). Stepper/gimbal is disabled. Real-time motor trim from CH10 compensates for hull skew.

### Hover
Motors idle at `MIN_PULSE`. Stepper gimbal pans continuously for environmental scanning.

### Autonomous
Replays the most recently saved path file with linear interpolation between waypoints. Falls back to compass-based heading navigation (`TARGET_HEADING = 210°`) if no path is loaded. The gimbal continues panning throughout.

---

## Path Planning

MARLINv1 supports **record-then-replay** path execution:

1. **Record**: Flip CH5 high and hold 1 second → vehicle enters path planning mode. Drive manually; motor commands are written to `/path_logs/path_XXXX.csv` at 10 Hz in real-time (no RAM buffering — unlimited recording length).
2. **Save**: Flip CH5 low and hold 1 second → path is finalized and automatically loaded for playback.
3. **Replay**: Switch to autonomous mode → the vehicle streams the path from flash, interpolating between waypoints with time-accurate timing.

Path files are stored sequentially (`path_0001.csv`, `path_0002.csv`, …). The most recent path is loaded automatically on boot.

---

## Sensors

### Compass (HMC5883L)
Used for autonomous heading navigation. Requires calibration before first use — run `compassCalibration.py` and copy the resulting offsets into `MAG_OFFSET` in the main script. A low-pass filter (`ALPHA = 0.15`) smooths the heading output.

### IMU (MPU6050)
Provides 3-axis accelerometer (±2g) and gyroscope (±250°/s) data. Currently used for telemetry logging and onboard state awareness.

### GPS
Parses `$GPGGA` / `$GNGGA` NMEA sentences at 9600 baud. Logs fix quality, latitude, longitude, altitude, and satellite count. No GPS fix is required for operation.

---

## Camera

The ESP32-CAM runs as a WiFi access point (`ESP32_CAM_AP`). MARLINv1 connects to it on boot and sends HTTP POST commands to `/command` to start and stop video recording. Recording is toggled via CH8 on the RC transmitter and indicated by a blinking LED (2 Hz when recording, off otherwise).

If the camera is unavailable, the vehicle continues operating normally — camera commands fail silently.

---

## Telemetry

Telemetry is logged to `/telemetry_logs/log_XXXX.csv` at 1 Hz. Each session creates a new numbered file. Logs roll over automatically if a file exceeds 2 MB.

**Logged fields:**
`time_ms`, `mode`, `path_planning`, `heading`, `cardinal`, `target`, `error`, `left_motor`, `right_motor`, `auto_motor`, `gps_status`, `lat`, `lon`, `alt`, `sats`, `acc_x/y/z`, `gyro_x/y/z`, `encoder_pos`

To download logs, use Thonny's file browser or read directly:
```python
with open('/telemetry_logs/log_0001.csv', 'r') as f:
    print(f.read())
```

---

## Stepper / Gimbal

The camera gimbal uses a stepper motor controlled by an encoder-based pan system — **no step counting, no drift**. The encoder position is read continuously; the motor reverses direction when it hits the configured limits (`ENCODER_MIN = -4`, `ENCODER_MAX = 4`). This gives reliable, repeatable pan behavior across indefinite operation.

The gimbal runs only in **hover** and **autonomous** modes. It is disabled in manual mode.

---

## File Structure

```
MARLINv1/
├── README.md
└── src/
    ├── main/
    │   └── main.py                  # Production firmware
    └── debug/
        ├── features/
        │   ├── manualFeature.py     # Manual + recording only
        │   └── waypointFeature.py   # Heading nav + telemetry
        └── sensors/
            ├── compassCalibration.py
            ├── compassReader.py
            ├── gpsReader.py
            ├── ibusReader.py
            ├── motorCalibration.py
            └── stepperRunner.py
```

---

## Setup

### 1. Flash MicroPython
Flash MicroPython firmware to the ESP32-S3 using `esptool`.

### 2. Calibrate the Compass
```
# Run on the device via Thonny or mpremote
src/debug/sensors/compassCalibration.py
```
Rotate the vehicle slowly through all orientations for 25 seconds. Copy the printed `OFFSET` values into `MAG_OFFSET` in `main.py`.

### 3. Calibrate the ESCs
```
src/debug/sensors/motorCalibration.py
```
Powers ESCs through the standard max→min→mid arming sequence.

### 4. Deploy Main Firmware
Copy `src/main/main.py` to the device root as `main.py`. It will run automatically on boot.

### 5. Set Target Heading
Edit `TARGET_HEADING` in `main.py` to your desired autonomous patrol bearing (default: `210°`).

---

## Debug Scripts

| Script | Purpose |
|---|---|
| `ibusReader.py` | Print raw RC channel values |
| `compassReader.py` | Live heading + IMU readout |
| `compassCalibration.py` | Generate magnetometer offsets |
| `motorCalibration.py` | ESC arming sequence |
| `stepperRunner.py` | Basic stepper movement test |
| `gpsReader.py` | Raw NMEA sentence monitor |
| `manualFeature.py` | Isolated manual + camera control |
| `waypointFeature.py` | Heading nav + telemetry (no path planning) |

---

## Known Issues / Notes

- `waypointFeature.py` has a typo in the iBus baud rate (`115115` instead of `115200`) — use `main.py` for production.
- GPS altitude parsing assumes field index 9 in `$GPGGA`; verify against your specific GPS module's output if altitude reads incorrectly.
- Motor trim (CH10) is applied symmetrically: left += trim, right -= trim. Adjust the scale factor in `main.py` if finer resolution is needed.
- The path replay system opens and seeks the CSV file on every loop iteration. For very long paths on slow flash storage, this may introduce minor timing jitter.