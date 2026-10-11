# BoxDrive Arduino Library (v1.0.0)

<p align="center">
  <a href="https://boxpox.in" target="_blank">
    <img src="https://img.shields.io/badge/BoxPox-Technologies-facc15?style=for-the-badge&logoColor=000" alt="BoxPox">
  </a>
  <img src="https://img.shields.io/badge/Hardware-v3.26-0ea5e9?style=for-the-badge" alt="Hardware v3.26">
  <img src="https://img.shields.io/badge/MCU-ESP32--C6%20RISC--V-ef4444?style=for-the-badge" alt="ESP32-C6">
  <img src="https://img.shields.io/badge/Driver-TMC2209-10b981?style=for-the-badge" alt="TMC2209">
  <img src="https://img.shields.io/badge/Encoder-AS5600%2012--Bit-8b5cf6?style=for-the-badge" alt="AS5600">
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-f59e0b?style=for-the-badge" alt="License: MIT">
  </a>
</p>

The official, production-grade Arduino library for the **BoxPox BoxDrive (HW v3.26)** closed-loop stepper motor controller.

BoxDrive mounts directly to the rear face of any standard **NEMA 17 stepper motor** (42.31 × 42.31 mm) and integrates an **ESP32-C6 Mini** (RISC-V 160 MHz, Wi-Fi 6, BLE 5), a **Trinamic TMC2209** ultra-silent stepper driver, an **AS5600 12-bit magnetic rotary encoder**, and an **SN65HVD230D CAN bus transceiver** with XT30 daisy-chain power distribution (4.75V – 24.0V).

Official Website & Documentation: [https://boxpox.in](https://boxpox.in)  
GitHub Repository: [https://github.com/BoxPoxin/BoxDrive_Library](https://github.com/BoxPoxin/BoxDrive_Library)

---

## Table of Contents
1. [Key Features](#key-features)
2. [Hardware Pinout & Specifications](#hardware-pinout--specifications)
3. [Installation](#installation)
   - [Arduino IDE (Library Manager / ZIP)](#method-1-arduino-ide-zip-install)
   - [Git Clone](#method-2-git-clone)
   - [PlatformIO](#method-3-platformio)
4. [Arduino IDE Board Configuration](#arduino-ide-board-configuration)
5. [Quick Start](#quick-start)
6. [Complete Examples Catalog (20 Examples)](#complete-examples-catalog-20-examples)
7. [Core Capabilities Walkthrough](#core-capabilities-walkthrough)
   - [1. WiFi Hotspot & Clean Web Dashboard](#1-wifi-hotspot--clean-web-dashboard)
   - [2. Closed-Loop PID Position Control](#2-closed-loop-pid-position-control)
   - [3. Kinesthetic Hand Teaching & 5s Replay](#3-kinesthetic-hand-teaching--5s-replay)
   - [4. CAN Bus Daisy-Chain & Real-Time Copy-Control](#4-can-bus-daisy-chain--real-time-copy-control)
   - [5. BLE 5.0 Wireless Remote Control](#5-ble-50-wireless-remote-control)
   - [6. StallGuard4 Sensorless Stall Detection & Homing](#6-stallguard4-sensorless-detection--homing)
   - [7. S-Curve Jerk-Limited Speed Profiles](#7-s-curve-jerk-limited-speed-profiles)
8. [API Reference Summary](#api-reference-summary)
9. [Wiring, Power & Magnet Mounting](#wiring-power--magnet-mounting)
10. [Troubleshooting & FAQ](#troubleshooting--faq)
11. [License & Support](#license--support)

---

## Key Features

- **TMC2209 Ultra-Silent Driving**: Native StealthChop2 silent operation and SpreadCycle high-torque modes with software current scaling (up to 2.0 A RMS) and microstepping from full step up to 1/256.
- **AS5600 12-Bit Magnetic Feedback**: 0.0879° angular resolution via high-speed I2C, multi-turn tracking, angular velocity calculation, and real-time magnet health diagnostics (AGC / magnitude).
- **Smooth Closed-Loop PID Controller**: Continuous hardware-timed PID loop corrects position deviation, eliminates missed steps, and maintains precise angular lock with low-pass filtered derivative action to prevent jitter.
- **Kinesthetic Hand Teach & 5-Second Motion Playback**: Free-wheeling recording buffer samples shaft movement at 100 Hz (10 ms intervals) and replicates the exact motion on the stepper at variable playback speeds (0.5x, 1.0x, 1.5x, 2.0x) with linear trajectory interpolation.
- **CAN Bus Synchronous Copy-Control**: Daisy-chain multiple BoxDrives over CAN (up to 1 Mbps). One motor acts as Leader (broadcasting encoder angle), while follower motors replicate the motion in real time with zero packet loss.
- **Built-in WiFi Hotspot & Web Dashboard**: Self-contained Access Point (`BoxDrive` at `192.168.4.1`) serving an interactive modern light-theme HTML5 control panel (Black/White/Yellow accents, minimalist dial gauge, and live SVG telemetry charts) without requiring an external router or internet.
- **BLE 5.0 GATT Interface**: Connect via smartphones or gamepads for wireless jog, speed, and real-time telemetry streaming.
- **Sensorless StallGuard4 & Endstop Homing**: Eliminates mechanical limit switches by monitoring back-EMF load spikes.
- **S-Curve Jerk-Limited Motion Planning**: 3rd-order polynomial velocity profiles eliminate frame vibration and mechanical resonance.
- **Full Hardware Diagnostics**: Automated self-test verifying driver UART, coil continuity, short circuits, thermal thresholds, and magnet alignment.

---

## Hardware Pinout & Specifications

### Technical Specifications

| Parameter | Specification |
| :--- | :--- |
| **Microcontroller** | ESP32-C6FH4 Mini (32-bit RISC-V @ 160 MHz, 320 KB SRAM, 4 MB Flash) |
| **Wireless** | Wi-Fi 6 (802.11ax 2.4 GHz), Bluetooth 5.0 LE, Zigbee / Thread (802.15.4) |
| **Motor Driver** | Trinamic TMC2209-LA (UART configured, StealthChop2 / SpreadCycle) |
| **Drive Current** | 100 mA to 2000 mA RMS software-controlled |
| **Encoder** | ams AS5600 12-bit Contactless Magnetic Rotary Encoder (4096 LSB/rev, 0.0879°) |
| **CAN Bus** | SN65HVD230D 3.3V CAN Transceiver (up to 1 Mbps) |
| **Input Voltage** | 4.75V to 24.0V DC via XT30PW connector (daisy-chain in/out) |
| **Dimensions** | 42.31 mm × 42.31 mm × 9.1 mm (direct NEMA 17 rear mount) |
| **Mounting Holes** | 4× M3 threaded holes on 31.0 mm center-to-center pitch |

### ESP32-C6 GPIO Map

```
┌─────────────────────────────────────────────────────────────┐
│                 BOXDRIVE HW v3.26 PINOUT                    │
├────────┬────────────┬───────────────────────────────────────┤
│  GPIO  │   Signal   │ Function                              │
├────────┼────────────┼───────────────────────────────────────┤
│ GPIO 0 │ DIR        │ TMC2209 Direction control             │
│ GPIO 1 │ STEP       │ TMC2209 Step pulse signal             │
│ GPIO 2 │ EN         │ TMC2209 Driver Enable (Active LOW)    │
│ GPIO 3 │ DIAG       │ TMC2209 StallGuard / Step loss output │
│ GPIO 4 │ PWM        │ Auxiliary PWM output                  │
│ GPIO 14│ UART RX    │ TMC2209 UART Receive line             │
│ GPIO 18│ UART TX    │ TMC2209 UART Transmit line            │
│ GPIO 19│ LED        │ On-board Status LED (PWM-capable)     │
│ GPIO 20│ CAN TX     │ SN65HVD230D CAN Transmit              │
│ GPIO 21│ CAN RX     │ SN65HVD230D CAN Receive               │
│ GPIO 22│ SCL        │ AS5600 I2C Clock                      │
│ GPIO 23│ SDA        │ AS5600 I2C Data                       │
└────────┴────────────┴───────────────────────────────────────┘
```

---

## Installation

### Method 1: Arduino IDE (ZIP Install)
1. Download the latest release `.zip` from [GitHub Releases](https://github.com/BoxPoxin/BoxDrive_Library/releases).
2. In the Arduino IDE, navigate to **Sketch -> Include Library -> Add .ZIP Library...**.
3. Select the downloaded `.zip` file.
4. Restart Arduino IDE. The library and all 20 examples will be available under **File -> Examples -> BoxDrive**.

### Method 2: Git Clone
Clone directly into your Arduino `libraries` folder:

```bash
# macOS / Linux
cd ~/Documents/Arduino/libraries
git clone https://github.com/BoxPoxin/BoxDrive_Library.git BoxDrive

# Windows (PowerShell)
cd $HOME\Documents\Arduino\libraries
git clone https://github.com/BoxPoxin/BoxDrive_Library.git BoxDrive
```

### Method 3: PlatformIO
Add the repository directly to your `platformio.ini`:

```ini
[env:esp32-c6-devkitm-1]
platform = espressif32
board = esp32-c6-devkitm-1
framework = arduino
monitor_speed = 115200
lib_deps =
    https://github.com/BoxPoxin/BoxDrive_Library.git
```

---

## Arduino IDE Board Configuration

Ensure the official ESP32 Arduino Core (version 3.0.0 or later) is installed in your Boards Manager (`esp32` by Espressif Systems).

Under the **Tools** menu in Arduino IDE, select:
- **Board**: `ESP32C6 Dev Module`
- **CPU Frequency**: `160MHz`
- **Flash Frequency**: `80MHz`
- **Flash Mode**: `QIO`
- **Flash Size**: `4MB (32Mb)`
- **Partition Scheme**: `Default 4MB with spiffs` or `Huge App (3MB No OTA/1MB SPIFFS)`
- **Upload Speed**: `921600` (or `460800`)
- **USB CDC On Boot**: `Enabled` (for Serial monitor over USB)

---

## Quick Start

Here is a minimal 5-line working sketch to rotate a motor:

```cpp
#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    Serial.begin(115200);
    motor.begin();         // Initializes GPIO, TMC2209 UART, and AS5600 I2C
    motor.setCurrent(800); // 800 mA run current
    motor.enableMotor();   // Energize stepper coils
}

void loop() {
    motor.move(3200);      // 1 full revolution CW (at 16 microsteps)
    delay(1000);
    motor.move(-3200);     // 1 full revolution CCW
    delay(1000);
}
```

---

## Complete Examples Catalog (20 Examples)

The library includes **20 comprehensive, production-ready examples** located in the `examples/` directory:

| # | Example Name | Description | Key API Methods |
| :- | :--- | :--- | :--- |
| **01** | [`01_BasicMotorControl`](examples/01_BasicMotorControl/01_BasicMotorControl.ino) | Bare-minimum starter sketch. Initialization, current setting, and back-and-forth moves. | `begin()`, `setCurrent()`, `move()` |
| **02** | [`02_StealthChopSilent`](examples/02_StealthChopSilent/02_StealthChopSilent.ino) | Ultra-quiet motor driving for sound-sensitive environments (3D printers, medical gear). | `setMode(STEALTH_CHOP)`, `setMicrosteps(16)` |
| **03** | [`03_SpreadCycleTorque`](examples/03_SpreadCycleTorque/03_SpreadCycleTorque.ino) | High-torque, high-speed SpreadCycle mode for CNCs and heavy dynamic loads. | `setMode(SPREAD_CYCLE)`, `setCurrent(1500)` |
| **04** | [`04_MicrosteppingDemo`](examples/04_MicrosteppingDemo/04_MicrosteppingDemo.ino) | Cycles through 1, 2, 4, 8, 16, 32, 64, 128, and 256 microstepping. | `setMicrosteps()` |
| **05** | [`05_CurrentControl`](examples/05_CurrentControl/05_CurrentControl.ino) | Dynamic current scaling (run current vs idle hold current) to minimize thermals. | `setCurrent(run, hold)`, `getCurrent()` |
| **06** | [`06_EncoderReadPosition`](examples/06_EncoderReadPosition/06_EncoderReadPosition.ino) | Reads AS5600 12-bit angle, multi-turn accumulation, velocity (deg/s), and magnet health. | `encoder.readAngleDeg()`, `encoder.readAGC()` |
| **07** | [`07_ClosedLoopPosition`](examples/07_ClosedLoopPosition/07_ClosedLoopPosition.ino) | Full closed-loop PID control: motor uses encoder feedback to aggressively hold target position. | `enableClosedLoop()`, `updateClosedLoop()` |
| **08** | [`08_EncoderRecordPlayback`](examples/08_EncoderRecordPlayback/08_EncoderRecordPlayback.ino) | Hand kinesthetic teaching: record shaft movement for 5 seconds, then replay it with the motor. | `startRecording()`, `playRecording()`, `updateRecording()` |
| **09** | [`09_HotspotMotorControl`](examples/09_HotspotMotorControl/09_HotspotMotorControl.ino) | Spawns an on-board WiFi hotspot (`192.168.4.1`) with a responsive web dashboard for motor control. | `startHotspot()`, `handleHotspot()` |
| **10** | [`10_CANBusSingleAxis`](examples/10_CANBusSingleAxis/10_CANBusSingleAxis.ino) | CAN bus single-node control: send position commands and receive real-time telemetry over CAN. | `can.begin()`, `can.sendPosition()`, `can.receive()` |
| **11** | [`11_CANBusCopyControl`](examples/11_CANBusCopyControl/11_CANBusCopyControl.ino) | Synchronous master-follower copy control: rotate leader motor by hand, follower mirrors instantly. | `setCANLeader()`, `setCANFollower()`, `updateCAN()` |
| **12** | [`12_CANBusMultiAxis`](examples/12_CANBusMultiAxis/12_CANBusMultiAxis.ino) | Coordinated multi-axis motion controller coordinating synchronized multi-joint movements over CAN. | `can.send()`, `BD_CAN_MSG_SYNC` |
| **13** | [`13_StallGuardDetection`](examples/13_StallGuardDetection/13_StallGuardDetection.ino) | Sensorless load detection using TMC2209 StallGuard4 with live back-EMF load metric and DIAG pin stop. | `enableStallGuard()`, `isStalled()`, `readStallGuardValue()` |
| **14** | [`14_SpeedProfile`](examples/14_SpeedProfile/14_SpeedProfile.ino) | Side-by-side comparison of Trapezoidal acceleration vs S-Curve jerk-limited smooth trajectories. | `trapezoidalMove()`, `sCurveMove()` |
| **15** | [`15_WiFiOTAControl`](examples/15_WiFiOTAControl/15_WiFiOTAControl.ino) | Over-The-Air wireless firmware flashing with automatic safety motor shutdown during upload. | `ArduinoOTA`, `WiFi.begin()` |
| **16** | [`16_BLEJoystick`](examples/16_BLEJoystick/16_BLEJoystick.ino) | Bluetooth Low Energy 5.0 GATT server for wireless jog, speed, and angle control from smartphones. | `BLEDevice`, `BLEServer`, `notify()` |
| **17** | [`17_MotorDiagnostics`](examples/17_MotorDiagnostics/17_MotorDiagnostics.ino) | Automated self-test report checking I2C, magnet alignment, UART registers, short circuits, and coils. | `getStatus()`, `printDiagnostics()` |
| **18** | [`18_HomingRoutine`](examples/18_HomingRoutine/18_HomingRoutine.ino) | Dual-mode homing: sensorless StallGuard hard-stop detection and precision AS5600 magnetic index zero. | `homeStallGuard()`, `homeEncoder()` |
| **19** | [`19_SinusoidalMotion`](examples/19_SinusoidalMotion/19_SinusoidalMotion.ino) | Continuous harmonic sinusoidal oscillation with live output stream for Arduino Serial Plotter. | `sinusoidalMotion()`, `moveToAngle()` |
| **20** | [`20_FullSystemDemo`](examples/20_FullSystemDemo/20_FullSystemDemo.ino) | Master interactive showcase integrating Serial CLI, WiFi hotspot, closed-loop PID, record/replay, and CAN. | Master interactive multi-threaded loop |

---

## Core Capabilities Walkthrough

### 1. WiFi Hotspot & Clean Web Dashboard

BoxDrive broadcasts its own independent 2.4 GHz Wi-Fi network without requiring an external Wi-Fi router.

```cpp
#include <BoxDrive.h>
BoxDrive motor;

void setup() {
    motor.begin();
    motor.enableMotor();
    // Default Access Point: SSID "BoxDrive", Password "boxdrive1234"
    motor.startHotspot("BoxDrive", "boxdrive1234");
}

void loop() {
    motor.handleHotspot(); // Non-blocking WebServer request handler
}
```

Connect your phone or computer to the **BoxDrive** Wi-Fi network and open `http://192.168.4.1` in your browser.

**Web Dashboard Highlights:**
- **Modern Clean Theme**: Off-white background, crisp typography, and yellow accent highlights.
- **Instrument Angle Gauge**: Minimalist 360° SVG dial with thin needle, active yellow sweep track, target diamond, and clear digital readout.
- **Live Real-Time Telemetry Charts**: Shaft position and angular velocity stream smoothly in real time.
- **Full Driver Controls**: Energize/de-energize toggle, emergency stop, closed-loop PID mode, direction invert, StealthChop/SpreadCycle mode, current slider (100–2000 mA), speed slider, and microstepping selector.
- **Kinesthetic Hand Teach & Play**: Integrated recording trigger and playback with live progress bar.

---

### 2. Closed-Loop PID Position Control

In standard open-loop mode, stepper motors lose synchronization under unexpected mechanical torque shocks. BoxDrive's closed-loop controller reads the AS5600 magnetic encoder at hardware frequencies, filters derivative noise, applies a deadband (0.5°) to prevent hunting, and computes corrective step pulses:

```cpp
// 1. Enable closed-loop control with tuned PID gains (Kp=1.0, Ki=0.002, Kd=0.04)
motor.enableClosedLoop(1.0f, 0.002f, 0.04f);

// 2. Command target angle (in degrees, 0.0° – 360.0°)
motor.setTargetAngle(180.0f);

void loop() {
    // Call continuously in your loop: computes PID and generates smooth microsteps
    motor.updateClosedLoop();
}
```

---

### 3. Kinesthetic Hand Teaching & 5s Replay

Teach complex, organic movements by simply moving the motor shaft with your hand:

```cpp
#include <BoxDrive.h>
BoxDrive motor;

void setup() {
    motor.begin();
    // 1. Release coils so the user can easily rotate the motor by hand
    motor.disableMotor();
    
    // 2. Record 5 seconds of hand motion (samples at 100 Hz into RAM buffer)
    motor.startRecording(5000);
}

void loop() {
    // Record motion frames
    motor.updateRecording();
    
    // Once recording completes, energize motor and replay trajectory
    if (!motor.isRecording() && motor.getRecordedSampleCount() > 0) {
        motor.enableMotor();
        motor.playRecording(1.0f); // 1.0x normal speed (or 0.5x, 2.0x)
        while (motor.isPlaying()) {
            motor.updateRecording(); // Drives playback interpolation
        }
    }
}
```

---

### 4. CAN Bus Daisy-Chain & Real-Time Copy-Control

BoxDrive includes an on-board **SN65HVD230D CAN bus transceiver** and dual XT30 connectors for daisy-chaining power and signal across multiple motorized joints:

#### Leader Node (Rotated by Hand):
```cpp
#include <BoxDrive.h>
BoxDrive motor;

void setup() {
    motor.begin();
    motor.can.begin(BD_CAN_SPEED_500K, 0x01); // 500 kbps, Node ID 0x01
    motor.disableMotor();                     // Free shaft for manual guidance
    motor.setCANLeader();                     // Broadcast encoder angle frames
}

void loop() {
    motor.updateCAN();                        // Broadcasts encoder at 100 Hz
}
```

#### Follower Node (Synchronously Mirrors Leader):
```cpp
#include <BoxDrive.h>
BoxDrive motor;

void setup() {
    motor.begin();
    motor.setCurrent(1000);
    motor.enableMotor();
    motor.can.begin(BD_CAN_SPEED_500K, 0x02); // Node ID 0x02
    motor.setCANFollower(0x01);               // Follow Leader Node 0x01
}

void loop() {
    motor.updateCAN();                        // Tracks leader motion in real time
}
```

---

### 5. BLE 5.0 Wireless Remote Control

Control BoxDrive from any Bluetooth LE terminal app (e.g., nRF Connect) or custom Web-BLE interface:
- **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **Command Characteristic**: `beb5483e-36e1-4688-b7f5-ea07361b26a8` (Write)
- **Telemetry Characteristic**: `1c93d406-4ff5-4847-9c5e-f6da8f773418` (Notify)

ASCII commands: `MOVE 3200`, `ANGLE 90.0`, `SPEED 1500`, `STOP`.

---

### 6. StallGuard4 Sensorless Detection & Homing

TMC2209 StallGuard4 measures motor back-EMF to detect mechanical contact without external limit switches:

```cpp
motor.enableStallGuard(90); // Stall threshold: 0–255 (Higher = more sensitive)

// Automatic homing towards mechanical endstop
bool ok = motor.homeStallGuard(BoxDriveDir::CCW, 300.0f, 60);
if (ok) {
    motor.setPosition(0); // Zero position counter
}
```

---

### 7. S-Curve Jerk-Limited Speed Profiles

Eliminate mechanical vibration and frame resonance by limiting jerk (rate of change of acceleration):

```cpp
// Steps: 6400, MaxSpeed: 2500 steps/s, Accel: 5000 steps/s², Jerk: 20000 steps/s³
motor.sCurveMove(6400, 2500.0f, 5000.0f, 20000.0f);
```

---

## API Reference Summary

### `BoxDrive` Main Class
- `bool begin(uint32_t baudRate = 115200)`: Initialize GPIOs, TMC2209 UART, and AS5600 I2C.
- `void enableMotor()` / `void disableMotor()`: Energize or de-energize stepper coils.
- `void setCurrent(uint16_t runMA, uint16_t holdMA = 0)`: Set RMS current in mA (100–2000 mA).
- `void setMode(BoxDriveMode mode)`: Switch between `STEALTH_CHOP` and `SPREAD_CYCLE`.
- `void setMicrosteps(uint16_t microsteps)`: Set 1, 2, 4, 8, 16, 32, 64, 128, 256.
- `void move(int32_t steps)`: Relative step move.
- `void moveTo(int32_t position)`: Absolute step move.
- `void moveToAngle(float angleDeg)`: Absolute degree move (0.0° – 360.0°).
- `void setSpeed(float stepsPerSec)` / `void setAcceleration(float stepsPerSec2)`: Set velocity limits.
- `void trapezoidalMove(int32_t steps, float maxSpeed, float accel)`: Trapezoidal profile.
- `void sCurveMove(int32_t steps, float maxSpeed, float accel, float jerk)`: Jerk-limited profile.
- `void sinusoidalMotion(float amplitudeDeg, float frequencyHz, uint32_t cycles)`: Harmonic oscillation.
- `void enableClosedLoop(float kp, float ki, float kd)` / `disableClosedLoop()`: Closed-loop PID.
- `void startRecording(uint32_t durationMs)` / `playRecording(float speed)`: Hand teaching.
- `void startHotspot(const char* ssid, const char* pass)` / `handleHotspot()`: WiFi Web UI.
- `void setCANLeader()` / `void setCANFollower(uint8_t leaderId)`: CAN copy-control.
- `bool homeStallGuard(BoxDriveDir dir, float speed, uint8_t threshold)`: Sensorless homing.
- `bool homeEncoder(float speed)`: Absolute magnetic zero homing.
- `void setLED(BoxDriveLED pattern)`: `OFF`, `ON`, `BLINK_SLOW`, `BLINK_FAST`, `PULSE`, `ERROR_FLASH`.

### `BoxDriveEncoder` Class (`motor.encoder`)
- `float readAngleDeg()`: Current shaft angle (0.00° – 360.00°).
- `float readMultiTurnDeg()`: Cumulative angle across multiple revolutions.
- `void resetMultiTurn()`: Reset multi-turn revolution counter.
- `float getVelocityDPS()`: Angular velocity in degrees per second.
- `uint8_t readAGC()`: Automatic Gain Control (ideal range: 40–180).
- `bool magnetDetected()`, `magnetTooStrong()`, `magnetTooWeak()`: Field diagnostics.

### `BoxDriveCAN` Class (`motor.can`)
- `bool begin(uint32_t speed, uint8_t nodeId)`: Initialize TWAI CAN controller.
- `bool send(uint32_t msgId, const uint8_t* data, uint8_t len)`: Transmit CAN frame.
- `bool receive(uint32_t& msgId, uint8_t* data, uint8_t& len)`: Receive CAN frame.
- `void sendPosition(uint8_t targetNode, float angleDeg)`: Send position command.
- `void broadcastEncoder(float angleDeg)`: Broadcast encoder position.

---

## Wiring, Power & Magnet Mounting

1. **Power Supply**: Use a clean **12V to 24V DC** power supply capable of supplying at least **2A per motor**.
2. **XT30 Daisy-Chaining**: Connect power to the input XT30 connector. Use the second XT30 connector to pass power to adjacent BoxDrives.
3. **Magnet Placement**:
   - Glue the diametrically magnetized cylindrical magnet (included with kit) to the rear motor shaft.
   - Maintain an air gap of **0.5 mm to 1.5 mm** between magnet and the AS5600 IC.
   - Check `motor.encoder.readAGC()`: Values between **40 and 180** indicate optimal magnetic field strength.
4. **CAN Bus Termination**:
   - Ensure a **120 Ω termination resistor** is installed across CAN-H and CAN-L at both extreme physical ends of the CAN bus.

---

## Troubleshooting & FAQ

### Q: The motor vibrates or makes clicking noises without rotating.
- **Cause**: Stepper coil wiring phases (A1, A2, B1, B2) are swapped, or motor current is set too low for mechanical load.
- **Fix**: Check that coil phase A and phase B are properly matched. Increase current using `motor.setCurrent(1000)`.

### Q: `encoder.magnetDetected()` returns false or angle is unstable.
- **Cause**: The diametrically magnetized magnet is missing, air gap exceeds 2.5 mm, or an axially polarized magnet was used.
- **Fix**: Verify a diametrically magnetized disc is installed within 0.5–1.5 mm of the AS5600 sensor IC.

### Q: Closed-loop mode is hunting or jittering around target.
- **Cause**: Proportional gain `Kp` is too high, or derivative filter is receiving noisy encoder readings.
- **Fix**: Use default tuned gains `motor.enableClosedLoop(1.0, 0.002, 0.04)`. A 0.5° deadband is automatically active to ensure zero motor humming when at rest.

### Q: CAN frames are not received by followers.
- **Cause**: Missing 120 Ω bus termination resistors or mismatched baud rates.
- **Fix**: Verify all nodes initialize with `can.begin(BD_CAN_SPEED_500K)` and a 120 Ω resistor bridges CAN-H and CAN-L at each bus terminus.

---

## License & Support

This library is released under the **MIT License**. See [LICENSE](LICENSE) for details.

- **Designed & Engineered by**: [BoxPox Technologies](https://boxpox.in)
- **Technical Support**: [support@boxpox.in](mailto:support@boxpox.in)
- **Documentation & Updates**: [https://boxpox.in](https://boxpox.in)
