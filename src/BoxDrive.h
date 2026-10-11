/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║                              B O X D R I V E                               ║
 * ║           Arduino Library for the BoxPox BoxDrive Motor Controller          ║
 * ║                              HW v3.26                                      ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  MCU       : ESP32-C6 Mini (RISC-V 160 MHz, Wi-Fi 6, BLE 5)               ║
 * ║  Driver    : TMC2209 (StealthChop + SpreadCycle, UART, 2.0 A RMS)          ║
 * ║  Encoder   : AS5600 12-bit Magnetic Rotary (0.0879° resolution)            ║
 * ║  CAN       : SN65HVD230D Transceiver                                      ║
 * ║  Power     : 4.75V – 24.0V via XT30 (Daisy-Chain)                         ║
 * ║  Dimensions: 42.31 × 42.31 × 9.1 mm (NEMA 17 mount pattern)              ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  License   : MIT                                                           ║
 * ║  Author    : BoxPox (support@boxpox.in)                                    ║
 * ║  Repository: https://github.com/BoxPoxin/BoxDrive_Library                  ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * GPIO MAPPING (ESP32-C6):
 * ┌────────┬──────────┬───────────────────────────────────────┐
 * │  GPIO  │  Signal  │  Description                          │
 * ├────────┼──────────┼───────────────────────────────────────┤
 * │   0    │  DIR     │  TMC2209 Direction control             │
 * │   1    │  STEP    │  TMC2209 Step pulse signal              │
 * │   2    │  EN      │  TMC2209 Enable (Active LOW)           │
 * │   3    │  DIAG    │  TMC2209 StallGuard / Step-loss detect │
 * │   4    │  PWM     │  General purpose PWM output            │
 * │  14    │  UART RX │  TMC2209 UART Receive                  │
 * │  18    │  UART TX │  TMC2209 UART Transmit                 │
 * │  19    │  LED     │  On-board Status LED                   │
 * │  20    │  CAN TX  │  SN65HVD230D CAN Transmit              │
 * │  21    │  CAN RX  │  SN65HVD230D CAN Receive               │
 * │  22    │  SCL     │  AS5600 I2C Clock                      │
 * │  23    │  SDA     │  AS5600 I2C Data                       │
 * └────────┴──────────┴───────────────────────────────────────┘
 */

#ifndef BOXDRIVE_H
#define BOXDRIVE_H

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS — BoxDrive HW v3.26 (ESP32-C6)
// ═══════════════════════════════════════════════════════════════════════════════

#define BD_PIN_DIR      0    // TMC2209 Direction
#define BD_PIN_STEP     1    // TMC2209 Step pulse
#define BD_PIN_EN       2    // TMC2209 Enable (Active LOW)
#define BD_PIN_DIAG     3    // TMC2209 StallGuard diagnostic
#define BD_PIN_PWM      4    // General-purpose PWM output
#define BD_PIN_UART_RX  14   // TMC2209 UART RX
#define BD_PIN_UART_TX  18   // TMC2209 UART TX
#define BD_PIN_LED      19   // On-board status LED
#define BD_PIN_CAN_TX   20   // CAN transceiver TX
#define BD_PIN_CAN_RX   21   // CAN transceiver RX
#define BD_PIN_SCL      22   // AS5600 I2C clock
#define BD_PIN_SDA      23   // AS5600 I2C data

// ═══════════════════════════════════════════════════════════════════════════════
//  AS5600 ENCODER CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define AS5600_ADDRESS       0x36
#define AS5600_RAW_ANGLE_H   0x0C
#define AS5600_RAW_ANGLE_L   0x0D
#define AS5600_ANGLE_H       0x0E
#define AS5600_ANGLE_L       0x0F
#define AS5600_STATUS        0x0B
#define AS5600_AGC           0x1A
#define AS5600_MAGNITUDE_H   0x1B
#define AS5600_MAGNITUDE_L   0x1C
#define AS5600_CONF_H        0x07
#define AS5600_CONF_L        0x08
#define AS5600_ZPOS_H        0x01
#define AS5600_ZPOS_L        0x02
#define AS5600_MPOS_H        0x03
#define AS5600_MPOS_L        0x04
#define AS5600_MANG_H        0x05
#define AS5600_MANG_L        0x06
#define AS5600_BURN          0xFF

#define AS5600_RESOLUTION    4096   // 12-bit
#define AS5600_DEG_PER_LSB   0.0879f

// ═══════════════════════════════════════════════════════════════════════════════
//  TMC2209 UART REGISTER MAP (subset used by this library)
// ═══════════════════════════════════════════════════════════════════════════════

#define TMC2209_GCONF        0x00
#define TMC2209_GSTAT        0x01
#define TMC2209_IFCNT        0x02
#define TMC2209_IOIN         0x06
#define TMC2209_IHOLD_IRUN   0x10
#define TMC2209_TPOWERDOWN   0x11
#define TMC2209_TSTEP        0x12
#define TMC2209_TPWMTHRS     0x13
#define TMC2209_TCOOLTHRS    0x14
#define TMC2209_SGTHRS       0x40
#define TMC2209_SG_RESULT    0x41
#define TMC2209_COOLCONF     0x42
#define TMC2209_MSCNT        0x6A
#define TMC2209_CHOPCONF     0x6C
#define TMC2209_DRV_STATUS   0x6F
#define TMC2209_PWMCONF      0x70

// GCONF bit positions
#define TMC2209_I_SCALE_ANALOG   0
#define TMC2209_INTERNAL_RSENSE  1
#define TMC2209_EN_SPREADCYCLE   2
#define TMC2209_SHAFT            3
#define TMC2209_INDEX_OTPW       4
#define TMC2209_INDEX_STEP       5
#define TMC2209_PDN_DISABLE      6
#define TMC2209_MSTEP_REG_SELECT 7
#define TMC2209_MULTISTEP_FILT   8

// TMC2209 UART protocol constants
#define TMC2209_SYNC         0x05
#define TMC2209_SLAVE_ADDR   0x00  // Default address for single driver

// Microstepping constants
#define BD_MICROSTEP_256  0
#define BD_MICROSTEP_128  1
#define BD_MICROSTEP_64   2
#define BD_MICROSTEP_32   3
#define BD_MICROSTEP_16   4
#define BD_MICROSTEP_8    5
#define BD_MICROSTEP_4    6
#define BD_MICROSTEP_2    7
#define BD_MICROSTEP_FULL 8

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTOR CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define BD_STEPS_PER_REV     200     // Standard NEMA 17 (1.8° full step)
#define BD_MAX_CURRENT_MA    2000    // 2.0 A RMS max
#define BD_DEFAULT_CURRENT   800     // Safe default current (mA)
#define BD_DEFAULT_SPEED     1000    // Default speed (steps/sec)
#define BD_DEFAULT_ACCEL     2000    // Default acceleration (steps/sec²)

// ═══════════════════════════════════════════════════════════════════════════════
//  CAN BUS CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define BD_CAN_SPEED_125K    125000
#define BD_CAN_SPEED_250K    250000
#define BD_CAN_SPEED_500K    500000
#define BD_CAN_SPEED_1M      1000000

// CAN message IDs (BoxDrive protocol)
#define BD_CAN_MSG_POSITION    0x100  // Position command
#define BD_CAN_MSG_SPEED       0x101  // Speed command
#define BD_CAN_MSG_STATUS      0x102  // Status request/report
#define BD_CAN_MSG_SYNC        0x103  // Sync signal for multi-axis
#define BD_CAN_MSG_ENCODER     0x104  // Encoder value broadcast
#define BD_CAN_MSG_COPY        0x110  // Copy-control: mirror encoder from leader
#define BD_CAN_MSG_CONFIG      0x120  // Remote configuration
#define BD_CAN_MSG_HEARTBEAT   0x1F0  // Node alive heartbeat

// ═══════════════════════════════════════════════════════════════════════════════
//  RECORDING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define BD_MAX_RECORDING_SAMPLES  1000   // Up to 10 sec at 100 Hz (10 ms interval)
#define BD_RECORDING_INTERVAL_MS  10     // 10 ms between samples → 100 Hz capture rate

// ═══════════════════════════════════════════════════════════════════════════════
//  ENUMERATIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Motor driver mode selection.
 * StealthChop = silent, SpreadCycle = torque-optimized.
 */
enum class BoxDriveMode {
    STEALTH_CHOP,    ///< Silent mode (TMC2209 StealthChop PWM)
    SPREAD_CYCLE     ///< High-torque mode (TMC2209 SpreadCycle)
};

/**
 * Motor direction constants.
 */
enum class BoxDriveDir {
    CW  = 0,   ///< Clockwise
    CCW = 1    ///< Counter-clockwise
};

/**
 * LED blink patterns for status indication.
 */
enum class BoxDriveLED {
    OFF,             ///< LED fully off
    ON,              ///< LED fully on
    BLINK_SLOW,      ///< 1 Hz blink — idle/ready
    BLINK_FAST,      ///< 5 Hz blink — motor active
    PULSE,           ///< Smooth breathing — WiFi/BLE active
    ERROR_FLASH      ///< Triple-flash pattern — error
};

/**
 * CAN bus node role for copy-control.
 */
enum class BoxDriveCANRole {
    STANDALONE,   ///< Not participating in copy-control
    LEADER,       ///< Broadcasting encoder position on CAN
    FOLLOWER      ///< Listening and mirroring leader position
};

/**
 * Motor status flags.
 */
struct BoxDriveStatus {
    bool     motorEnabled;       ///< Is motor driver enabled?
    bool     stallDetected;      ///< StallGuard triggered?
    bool     overTemp;           ///< Over-temperature warning?
    bool     overTempShutdown;   ///< Over-temperature pre-warning?
    bool     shortToGnd;         ///< Short to ground detected?
    bool     openLoad;           ///< Open-load detected?
    uint8_t  csActual;           ///< Actual current scale (0-31)
    uint16_t sgResult;           ///< StallGuard result value
    uint32_t drvStatus;          ///< Raw DRV_STATUS register
};

/**
 * Recorded motion sample: timestamp + encoder angle.
 */
struct MotionSample {
    uint32_t timestampMs;   ///< Milliseconds since recording start
    float    angleDeg;      ///< Encoder angle in degrees (0-360)
};

// ═══════════════════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════════

class BoxDriveEncoder;
class BoxDriveCAN;

// ═══════════════════════════════════════════════════════════════════════════════
//  CLASS: BoxDriveEncoder
//  Reads the AS5600 12-bit magnetic encoder via I2C.
// ═══════════════════════════════════════════════════════════════════════════════

class BoxDriveEncoder {
public:
    BoxDriveEncoder();

    /**
     * Initialize I2C communication with the AS5600.
     * @return true if AS5600 responded on the bus.
     */
    bool begin();

    /**
     * Read the raw 12-bit angle (0–4095).
     */
    uint16_t readRawAngle();

    /**
     * Read the angle in degrees (0.0 – 360.0).
     */
    float readAngleDeg();

    /**
     * Read the angle in radians (0.0 – 2π).
     */
    float readAngleRad();

    /**
     * Get the cumulative multi-turn angle in degrees.
     * Tracks full revolutions beyond 360°.
     */
    float readMultiTurnDeg();

    /**
     * Reset the multi-turn counter to zero at the current position.
     */
    void resetMultiTurn();

    /**
     * Get the angular velocity in degrees per second.
     * Call readAngleDeg() periodically for this to update.
     */
    float getVelocityDPS();

    /**
     * Read the AS5600 status register.
     * Bit 5: magnet detected, Bit 4: too weak, Bit 3: too strong.
     */
    uint8_t readStatus();

    /**
     * Read the automatic gain control value (0–255).
     * Useful for diagnosing magnet distance.
     */
    uint8_t readAGC();

    /**
     * Read the magnet magnitude (proportional to field strength).
     */
    uint16_t readMagnitude();

    /**
     * Check if a magnet is detected.
     */
    bool magnetDetected();

    /**
     * Check if the magnet field is too strong.
     */
    bool magnetTooStrong();

    /**
     * Check if the magnet field is too weak.
     */
    bool magnetTooWeak();

    /**
     * Set the zero-position (ZPOS) on the AS5600.
     * @param rawAngle Raw 12-bit value to set as zero.
     */
    void setZeroPosition(uint16_t rawAngle);

    /**
     * Set the max-position (MPOS) on the AS5600.
     * @param rawAngle Raw 12-bit value to set as maximum.
     */
    void setMaxPosition(uint16_t rawAngle);

private:
    uint8_t  _readRegister8(uint8_t reg);
    uint16_t _readRegister16(uint8_t reg);
    void     _writeRegister8(uint8_t reg, uint8_t value);
    void     _writeRegister16(uint8_t reg, uint16_t value);

    float    _lastAngle;
    float    _multiTurnAngle;
    float    _velocity;
    uint32_t _lastReadTime;
    bool     _initialized;
    int32_t  _turnCount;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  CLASS: BoxDriveCAN
//  CAN bus communication via the TWAI peripheral on ESP32-C6.
// ═══════════════════════════════════════════════════════════════════════════════

class BoxDriveCAN {
public:
    BoxDriveCAN();

    /**
     * Initialize the TWAI (CAN) peripheral.
     * @param speed  Baud rate (BD_CAN_SPEED_125K/250K/500K/1M).
     * @param nodeId Unique node ID for this BoxDrive (0x01–0xFE).
     * @return true if TWAI driver installed successfully.
     */
    bool begin(uint32_t speed = BD_CAN_SPEED_500K, uint8_t nodeId = 0x01);

    /**
     * Stop the TWAI peripheral.
     */
    void end();

    /**
     * Send a CAN frame.
     * @param msgId  11-bit standard CAN ID.
     * @param data   Pointer to data bytes (max 8).
     * @param len    Number of data bytes.
     * @return true if sent successfully.
     */
    bool send(uint32_t msgId, const uint8_t* data, uint8_t len);

    /**
     * Check if a CAN frame is available.
     * @return true if a frame is waiting.
     */
    bool available();

    /**
     * Receive a CAN frame (non-blocking).
     * @param msgId  [out] The received message ID.
     * @param data   [out] Buffer for received data (must be ≥8 bytes).
     * @param len    [out] Number of received data bytes.
     * @return true if a frame was received.
     */
    bool receive(uint32_t& msgId, uint8_t* data, uint8_t& len);

    /**
     * Send a position command to a specific node.
     * @param targetNode  Target node ID.
     * @param angleDeg    Target angle in degrees.
     */
    void sendPosition(uint8_t targetNode, float angleDeg);

    /**
     * Send current encoder position as a broadcast.
     * @param angleDeg  Current angle.
     */
    void broadcastEncoder(float angleDeg);

    /**
     * Send a heartbeat message.
     */
    void sendHeartbeat();

    /**
     * Send a copy-control message (leader broadcasts encoder for followers).
     * @param angleDeg  Leader's current encoder angle.
     */
    void sendCopyControl(float angleDeg);

    /**
     * Parse an incoming copy-control message.
     * @param data  Received CAN data buffer.
     * @param len   Data length.
     * @return Angle in degrees, or -1.0 on parse failure.
     */
    float parseCopyControl(const uint8_t* data, uint8_t len);

    /**
     * Get this node's ID.
     */
    uint8_t getNodeId() const { return _nodeId; }

    /**
     * Set a message filter (only receive specified IDs).
     * @param acceptId  Message ID to accept.
     * @param mask      Acceptance mask.
     */
    void setFilter(uint32_t acceptId, uint32_t mask);

private:
    uint8_t  _nodeId;
    bool     _initialized;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  CLASS: BoxDrive  (MAIN CLASS)
//  Unified motor controller with TMC2209, AS5600, CAN, WiFi, BLE.
// ═══════════════════════════════════════════════════════════════════════════════

class BoxDrive {
public:
    // ── Public sub-objects ────────────────────────────────────────────────────
    BoxDriveEncoder encoder;   ///< Direct access to the AS5600 encoder
    BoxDriveCAN     can;       ///< Direct access to the CAN bus controller

    // ── Constructor ──────────────────────────────────────────────────────────

    BoxDrive();

    // ═══════════════════════════════════════════════════════════════════════════
    //  INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Initialize all BoxDrive peripherals.
     * Sets up GPIO, UART to TMC2209, I2C to AS5600, and the status LED.
     * Call this in setup().
     * @return true if all peripherals responded.
     */
    bool begin();

    /**
     * Initialize with custom UART baud rate for TMC2209.
     * @param baudRate UART baud rate (default 115200).
     * @return true if initialization successful.
     */
    bool begin(uint32_t baudRate);

    // ═══════════════════════════════════════════════════════════════════════════
    //  MOTOR ENABLE / DISABLE
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Enable the motor driver (pull EN pin LOW).
     * Motor will hold position with configured current.
     */
    void enableMotor();

    /**
     * Disable the motor driver (pull EN pin HIGH).
     * Motor shaft is free to rotate.
     */
    void disableMotor();

    /**
     * Check if the motor is currently enabled.
     */
    bool isMotorEnabled() const;

    // ═══════════════════════════════════════════════════════════════════════════
    //  CURRENT CONTROL
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set the motor run current in milliamps.
     * @param mA  Run current (1–2000 mA). Clamped to BD_MAX_CURRENT_MA.
     */
    void setCurrent(uint16_t mA);

    /**
     * Set the motor hold current in milliamps.
     * @param mA  Hold current (0–2000 mA). Usually 30-50% of run current.
     */
    void setHoldCurrent(uint16_t mA);

    /**
     * Set both run and hold current.
     * @param runMA   Run current in mA.
     * @param holdMA  Hold current in mA.
     */
    void setCurrent(uint16_t runMA, uint16_t holdMA);

    /**
     * Get the currently configured run current in mA.
     */
    uint16_t getCurrent() const;

    // ═══════════════════════════════════════════════════════════════════════════
    //  DRIVER MODE
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set the driver mode (StealthChop or SpreadCycle).
     * @param mode BoxDriveMode::STEALTH_CHOP or BoxDriveMode::SPREAD_CYCLE.
     */
    void setMode(BoxDriveMode mode);

    /**
     * Get the current driver mode.
     */
    BoxDriveMode getMode() const;

    /**
     * Set the microstepping resolution.
     * @param microsteps  Microsteps per full step: 1, 2, 4, 8, 16, 32, 64, 128, or 256.
     */
    void setMicrosteps(uint16_t microsteps);

    /**
     * Get the current microstepping setting.
     */
    uint16_t getMicrosteps() const;

    // ═══════════════════════════════════════════════════════════════════════════
    //  BASIC MOTION (STEP/DIR)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set the motor speed in steps per second.
     * @param stepsPerSec  Speed value. Use negative values for reverse.
     */
    void setSpeed(float stepsPerSec);

    /**
     * Set the acceleration in steps per second².
     * @param stepsPerSec2  Acceleration value.
     */
    void setAcceleration(float stepsPerSec2);

    /**
     * Move the motor a specific number of steps (blocking).
     * Positive = CW, Negative = CCW.
     * @param steps  Number of steps to move.
     */
    void move(int32_t steps);

    /**
     * Move the motor to a specific step position (blocking).
     * Uses acceleration profile if configured.
     * @param position  Absolute step position.
     */
    void moveTo(int32_t position);

    /**
     * Move the motor to a specific angle in degrees (blocking).
     * Accounts for microstepping.
     * @param angleDeg  Target angle in degrees.
     */
    void moveToAngle(float angleDeg);

    /**
     * Rotate continuously in one direction (non-blocking).
     * Call stop() to halt.
     * @param dir  BoxDriveDir::CW or BoxDriveDir::CCW.
     */
    void rotate(BoxDriveDir dir);

    /**
     * Stop the motor immediately.
     */
    void stop();

    /**
     * Smoothly decelerate to a stop.
     */
    void softStop();

    /**
     * Get the current step position (internal counter).
     */
    int32_t getPosition() const;

    /**
     * Set/reset the internal step position counter.
     * @param pos  New position value.
     */
    void setPosition(int32_t pos);

    /**
     * Check if the motor is currently moving.
     */
    bool isMoving() const;

    /**
     * Invert the motor stepping direction relative to encoder readings.
     * Use this if the motor vibrates in closed-loop or moves in reverse.
     * @param invert true to invert direction.
     */
    void invertDirection(bool invert);

    /**
     * Check if stepping direction is inverted.
     */
    bool isDirectionInverted() const;

    // ═══════════════════════════════════════════════════════════════════════════
    //  CLOSED-LOOP POSITION CONTROL (Encoder Feedback)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Enable closed-loop position control.
     * The motor will use encoder feedback to maintain position.
     * @param kp  Proportional gain (default 2.0).
     * @param ki  Integral gain (default 0.01).
     * @param kd  Derivative gain (default 0.5).
     */
    void enableClosedLoop(float kp = 2.0f, float ki = 0.01f, float kd = 0.5f);

    /**
     * Disable closed-loop mode; revert to open-loop step/dir.
     */
    void disableClosedLoop();

    /**
     * Check if closed-loop mode is active.
     */
    bool isClosedLoopEnabled() const;

    /**
     * Set the target angle for closed-loop control.
     * @param angleDeg  Desired angle in degrees.
     */
    void setTargetAngle(float angleDeg);

    /**
     * Get the current target angle.
     */
    float getTargetAngle() const;

    /**
     * Get the current position error in degrees.
     */
    float getPositionError();

    /**
     * Update the closed-loop controller. 
     * MUST be called frequently (ideally every loop iteration).
     * Reads encoder, computes PID, and generates step pulses.
     */
    void updateClosedLoop();

    /**
     * Set PID gains for the closed-loop controller.
     * @param kp  Proportional gain.
     * @param ki  Integral gain.
     * @param kd  Derivative gain.
     */
    void setPIDGains(float kp, float ki, float kd);

    /**
     * Set the closed-loop position tolerance.
     * Motor stops correcting when error < tolerance.
     * @param degrees  Tolerance in degrees (default 0.5).
     */
    void setPositionTolerance(float degrees);

    // ═══════════════════════════════════════════════════════════════════════════
    //  STALLGUARD (Sensorless Load Detection)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Enable StallGuard sensorless stall detection.
     * @param threshold  StallGuard threshold (0–255). Lower = more sensitive.
     */
    void enableStallGuard(uint8_t threshold = 100);

    /**
     * Disable StallGuard.
     */
    void disableStallGuard();

    /**
     * Read the current StallGuard result value.
     * Low values indicate higher mechanical load.
     */
    uint16_t readStallGuardValue();

    /**
     * Check if a stall has been detected (DIAG pin).
     */
    bool isStalled();

    /**
     * Set a callback function for stall events.
     * @param callback  Function to call when stall is detected.
     */
    void onStall(void (*callback)());

    // ═══════════════════════════════════════════════════════════════════════════
    //  MOTION RECORDING & PLAYBACK
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Start recording encoder positions.
     * Samples are stored in an internal buffer at 1 kHz.
     * @param durationMs  Max recording duration in milliseconds (default 5000).
     */
    void startRecording(uint32_t durationMs = 5000);

    /**
     * Stop recording.
     */
    void stopRecording();

    /**
     * Check if recording is in progress.
     */
    bool isRecording() const;

    /**
     * Get the number of recorded samples.
     */
    uint16_t getRecordingSampleCount() const;

    /**
     * Play back the recorded motion.
     * The motor will replicate the recorded encoder trajectory.
     * @param speedMultiplier  Playback speed (1.0 = real-time, 2.0 = double).
     */
    void playRecording(float speedMultiplier = 1.0f);

    /**
     * Check if playback is in progress.
     */
    bool isPlaying() const;

    /**
     * Stop playback.
     */
    void stopPlayback();

    /**
     * Get a pointer to the recorded samples buffer.
     * @param count [out] Number of samples.
     * @return Pointer to the MotionSample array.
     */
    const MotionSample* getRecordingBuffer(uint16_t& count) const;

    /**
     * Load samples into the recording buffer from an external source.
     * @param samples  Array of MotionSample structs.
     * @param count    Number of samples.
     */
    void loadRecording(const MotionSample* samples, uint16_t count);

    /**
     * Update recording/playback state machine.
     * MUST be called in loop() when recording or playing.
     */
    void updateRecording();

    // ═══════════════════════════════════════════════════════════════════════════
    //  CAN BUS COPY-CONTROL
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set this node as a CAN leader (broadcaster).
     * Encoder position is continuously sent over CAN.
     */
    void setCANLeader();

    /**
     * Set this node as a CAN follower (listener).
     * Motor mirrors the position received from the leader.
     * @param leaderNodeId  Node ID of the leader to follow.
     */
    void setCANFollower(uint8_t leaderNodeId);

    /**
     * Set this node as standalone (no copy-control).
     */
    void setCANStandalone();

    /**
     * Get the current CAN role.
     */
    BoxDriveCANRole getCANRole() const;

    /**
     * Update CAN copy-control state machine.
     * MUST be called in loop() when using copy-control.
     */
    void updateCAN();

    // ═══════════════════════════════════════════════════════════════════════════
    //  WIFI HOTSPOT CONTROL
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Start a WiFi Access Point with a web UI for motor control.
     * Connect to the AP and navigate to 192.168.4.1.
     * @param ssid      AP name (default "BoxDrive").
     * @param password  AP password (default "boxdrive1234", min 8 chars).
     */
    void startHotspot(const char* ssid = "BoxDrive", const char* password = "boxdrive1234");

    /**
     * Stop the WiFi hotspot and web server.
     */
    void stopHotspot();

    /**
     * Check if the hotspot is active.
     */
    bool isHotspotActive() const;

    /**
     * Handle incoming HTTP requests.
     * MUST be called in loop() when hotspot is active.
     */
    void handleHotspot();

    // ═══════════════════════════════════════════════════════════════════════════
    //  SPEED PROFILES
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Execute a trapezoidal speed profile.
     * @param steps       Total steps to move.
     * @param maxSpeed    Peak speed in steps/sec.
     * @param accel       Acceleration in steps/sec².
     */
    void trapezoidalMove(int32_t steps, float maxSpeed, float accel);

    /**
     * Execute an S-curve speed profile for ultra-smooth motion.
     * @param steps       Total steps to move.
     * @param maxSpeed    Peak speed in steps/sec.
     * @param accel       Acceleration in steps/sec².
     * @param jerk        Jerk in steps/sec³.
     */
    void sCurveMove(int32_t steps, float maxSpeed, float accel, float jerk);

    /**
     * Execute a sinusoidal (back-and-forth) motion.
     * @param amplitudeDeg  Swing amplitude in degrees.
     * @param frequencyHz   Oscillation frequency.
     * @param cycles        Number of cycles (0 = infinite, call stop()).
     */
    void sinusoidalMotion(float amplitudeDeg, float frequencyHz, uint32_t cycles = 0);

    // ═══════════════════════════════════════════════════════════════════════════
    //  HOMING
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Home the motor using StallGuard (sensorless).
     * Motor moves in specified direction until stall is detected.
     * @param dir        Direction to home towards.
     * @param speed      Homing speed in steps/sec.
     * @param threshold  StallGuard threshold for homing.
     * @return true if homing was successful.
     */
    bool homeStallGuard(BoxDriveDir dir = BoxDriveDir::CCW, 
                        float speed = 200.0f,
                        uint8_t threshold = 50);

    /**
     * Home using the encoder zero position.
     * Motor rotates until encoder reads the stored ZPOS.
     * @param speed  Homing speed in steps/sec.
     * @return true if homing was successful.
     */
    bool homeEncoder(float speed = 200.0f);

    // ═══════════════════════════════════════════════════════════════════════════
    //  STATUS & DIAGNOSTICS
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Get the full motor/driver status.
     */
    BoxDriveStatus getStatus();

    /**
     * Read the TMC2209 DRV_STATUS register directly.
     */
    uint32_t readDriverStatus();

    /**
     * Get the driver temperature indication.
     * @return true if over-temperature warning is active.
     */
    bool isOverTemp();

    /**
     * Print a full diagnostic report to Serial.
     */
    void printDiagnostics();

    /**
     * Print the current encoder status to Serial.
     */
    void printEncoderStatus();

    // ═══════════════════════════════════════════════════════════════════════════
    //  LED CONTROL
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Set the on-board LED state/pattern.
     * @param pattern  BoxDriveLED enum value.
     */
    void setLED(BoxDriveLED pattern);

    /**
     * Update LED animation (call in loop() for animated patterns).
     */
    void updateLED();

    /**
     * Set LED brightness via PWM.
     * @param brightness  0–255.
     */
    void setLEDBrightness(uint8_t brightness);

    // ═══════════════════════════════════════════════════════════════════════════
    //  TMC2209 UART REGISTER ACCESS (Advanced)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Write a 32-bit value to a TMC2209 register via UART.
     * @param reg    Register address.
     * @param value  32-bit value to write.
     */
    void writeRegister(uint8_t reg, uint32_t value);

    /**
     * Read a 32-bit value from a TMC2209 register via UART.
     * @param reg  Register address.
     * @return 32-bit register value.
     */
    uint32_t readRegister(uint8_t reg);

private:
    // ── Internal methods ─────────────────────────────────────────────────────

    void _initPins();
    void _initUART(uint32_t baudRate);
    void _initTMC2209();
    void _stepPulse();
    void _setDirection(BoxDriveDir dir);
    void _delayMicroseconds(uint32_t us);
    uint8_t _calcCRC(uint8_t* data, uint8_t len);

    // Closed-loop internals
    void _pidCompute();

    // WiFi hotspot internals
    void _buildWebPage(String& html);

    // Recording internals
    void _recordSample();
    void _playSample();

    // ── Internal state ───────────────────────────────────────────────────────

    // Motor state
    bool         _motorEnabled;
    bool         _invertDirection;
    uint16_t     _runCurrentMA;
    uint16_t     _holdCurrentMA;
    uint16_t     _microsteps;
    float        _speed;          // steps/sec
    float        _acceleration;   // steps/sec²
    int32_t      _currentPosition;
    int32_t      _targetPosition;
    bool         _isMoving;
    BoxDriveMode _mode;

    // Closed-loop PID
    bool     _closedLoopEnabled;
    float    _targetAngle;
    float    _kp, _ki, _kd;
    float    _pidIntegral;
    float    _pidPrevError;
    float    _pidFilteredError;
    float    _pidDErrorFiltered;
    float    _positionTolerance;
    uint32_t _pidLastTime;
    uint32_t _pidLastStepTime;

    // StallGuard
    bool     _stallGuardEnabled;
    uint8_t  _stallThreshold;
    void     (*_stallCallback)();

    // Recording/Playback
    MotionSample _recordingBuffer[BD_MAX_RECORDING_SAMPLES];
    uint16_t     _recordingCount;
    uint32_t     _recordingStartTime;
    uint32_t     _recordingDuration;
    uint32_t     _lastSampleTime;
    bool         _isRecording;
    bool         _isPlaying;
    uint16_t     _playbackIndex;
    float        _playbackSpeed;
    uint32_t     _playbackStartTime;
    float        _recBaseAngle;
    int32_t      _playBaseStep;
    uint32_t     _lastPlayStepTime;

    // CAN copy-control
    BoxDriveCANRole _canRole;
    uint8_t         _leaderNodeId;
    uint32_t        _lastCANBroadcast;

    // WiFi Hotspot
    bool _hotspotActive;

    // LED
    BoxDriveLED _ledPattern;
    uint32_t    _ledLastToggle;
    bool        _ledState;
    uint8_t     _ledBrightness;

    // UART serial for TMC2209
    HardwareSerial* _tmcSerial;
};

#endif // BOXDRIVE_H
