/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  BoxDrive.cpp — Implementation of the BoxDrive Arduino Library             ║
 * ║  BoxPox BoxDrive HW v3.26 (ESP32-C6 + TMC2209 + AS5600 + CAN)             ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "BoxDrive.h"

// ESP32-C6 specific headers for CAN (TWAI) and WiFi
#ifdef ESP32
  #include <WiFi.h>
  #include <WebServer.h>
  #include <driver/twai.h>
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  BoxDriveEncoder IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

BoxDriveEncoder::BoxDriveEncoder()
    : _lastAngle(0.0f)
    , _multiTurnAngle(0.0f)
    , _velocity(0.0f)
    , _lastReadTime(0)
    , _initialized(false)
    , _turnCount(0)
{}

bool BoxDriveEncoder::begin() {
    Wire.begin(BD_PIN_SDA, BD_PIN_SCL);
    Wire.setClock(400000); // 400 kHz fast mode

    // Verify AS5600 is on the bus
    Wire.beginTransmission(AS5600_ADDRESS);
    uint8_t error = Wire.endTransmission();
    _initialized = (error == 0);

    if (_initialized) {
        _lastAngle = readAngleDeg();
        _lastReadTime = micros();
    }

    return _initialized;
}

uint16_t BoxDriveEncoder::readRawAngle() {
    return _readRegister16(AS5600_RAW_ANGLE_H);
}

float BoxDriveEncoder::readAngleDeg() {
    uint16_t raw = readRawAngle();
    float angle = raw * (360.0f / AS5600_RESOLUTION);

    // Update velocity and multi-turn calculation
    uint32_t now = micros();
    uint32_t dt = now - _lastReadTime;
    if (dt > 0 && _lastReadTime > 0) {
        float rawDelta = angle - _lastAngle;

        // Detect 360° boundary crossing BEFORE wrapping delta!
        if (rawDelta < -180.0f) {
            // Forward wrap (e.g. 355° -> 5°, rawDelta = -350°)
            _turnCount++;
        } else if (rawDelta > 180.0f) {
            // Reverse wrap (e.g. 5° -> 355°, rawDelta = +350°)
            _turnCount--;
        }

        float delta = rawDelta;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;

        _velocity = (delta / (float)dt) * 1000000.0f; // degrees/sec
        _multiTurnAngle = (_turnCount * 360.0f) + angle;
    } else {
        _multiTurnAngle = (_turnCount * 360.0f) + angle;
    }

    _lastAngle = angle;
    _lastReadTime = now;

    return angle;
}

float BoxDriveEncoder::readAngleRad() {
    return readAngleDeg() * DEG_TO_RAD;
}

float BoxDriveEncoder::readMultiTurnDeg() {
    readAngleDeg(); // Update internal state
    return _multiTurnAngle;
}

void BoxDriveEncoder::resetMultiTurn() {
    _turnCount = 0;
    _multiTurnAngle = _lastAngle;
}

float BoxDriveEncoder::getVelocityDPS() {
    return _velocity;
}

uint8_t BoxDriveEncoder::readStatus() {
    return _readRegister8(AS5600_STATUS);
}

uint8_t BoxDriveEncoder::readAGC() {
    return _readRegister8(AS5600_AGC);
}

uint16_t BoxDriveEncoder::readMagnitude() {
    return _readRegister16(AS5600_MAGNITUDE_H);
}

bool BoxDriveEncoder::magnetDetected() {
    return (readStatus() & 0x20) != 0; // Bit 5 = MD
}

bool BoxDriveEncoder::magnetTooStrong() {
    return (readStatus() & 0x08) != 0; // Bit 3 = MH
}

bool BoxDriveEncoder::magnetTooWeak() {
    return (readStatus() & 0x10) != 0; // Bit 4 = ML
}

void BoxDriveEncoder::setZeroPosition(uint16_t rawAngle) {
    _writeRegister16(AS5600_ZPOS_H, rawAngle & 0x0FFF);
}

void BoxDriveEncoder::setMaxPosition(uint16_t rawAngle) {
    _writeRegister16(AS5600_MPOS_H, rawAngle & 0x0FFF);
}

// ── Private I2C helpers ──────────────────────────────────────────────────────

uint8_t BoxDriveEncoder::_readRegister8(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

uint16_t BoxDriveEncoder::_readRegister16(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint16_t hi = Wire.read();
        uint16_t lo = Wire.read();
        return (hi << 8) | lo;
    }
    return 0;
}

void BoxDriveEncoder::_writeRegister8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void BoxDriveEncoder::_writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    Wire.endTransmission();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BoxDriveCAN IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

BoxDriveCAN::BoxDriveCAN()
    : _nodeId(0x01)
    , _initialized(false)
{}

bool BoxDriveCAN::begin(uint32_t speed, uint8_t nodeId) {
#ifdef ESP32
    _nodeId = nodeId;

    // Configure TWAI (CAN) peripheral
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)BD_PIN_CAN_TX, 
        (gpio_num_t)BD_PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 32;

    twai_timing_config_t t_config;
    switch (speed) {
        case BD_CAN_SPEED_125K:  t_config = TWAI_TIMING_CONFIG_125KBITS();  break;
        case BD_CAN_SPEED_250K:  t_config = TWAI_TIMING_CONFIG_250KBITS();  break;
        case BD_CAN_SPEED_1M:    t_config = TWAI_TIMING_CONFIG_1MBITS();    break;
        default:                 t_config = TWAI_TIMING_CONFIG_500KBITS();   break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) return false;

    err = twai_start();
    if (err != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    _initialized = true;
    return true;
#else
    return false; // CAN only supported on ESP32
#endif
}

void BoxDriveCAN::end() {
#ifdef ESP32
    if (_initialized) {
        twai_stop();
        twai_driver_uninstall();
        _initialized = false;
    }
#endif
}

bool BoxDriveCAN::send(uint32_t msgId, const uint8_t* data, uint8_t len) {
#ifdef ESP32
    if (!_initialized || len > 8) return false;

    twai_message_t msg;
    msg.identifier = msgId;
    msg.extd = 0;       // Standard 11-bit ID
    msg.rtr = 0;        // Not a remote request
    msg.data_length_code = len;
    memcpy(msg.data, data, len);

    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
#else
    return false;
#endif
}

bool BoxDriveCAN::available() {
#ifdef ESP32
    if (!_initialized) return false;
    twai_status_info_t status;
    twai_get_status_info(&status);
    return (status.msgs_to_rx > 0);
#else
    return false;
#endif
}

bool BoxDriveCAN::receive(uint32_t& msgId, uint8_t* data, uint8_t& len) {
#ifdef ESP32
    if (!_initialized) return false;

    twai_message_t msg;
    if (twai_receive(&msg, 0) == ESP_OK) {
        msgId = msg.identifier;
        len = msg.data_length_code;
        memcpy(data, msg.data, len);
        return true;
    }
    return false;
#else
    return false;
#endif
}

void BoxDriveCAN::sendPosition(uint8_t targetNode, float angleDeg) {
    uint8_t data[8];
    data[0] = targetNode;
    data[1] = _nodeId;
    // Pack float as 4 bytes (little-endian)
    memcpy(&data[2], &angleDeg, sizeof(float));
    data[6] = 0;
    data[7] = 0;
    send(BD_CAN_MSG_POSITION, data, 8);
}

void BoxDriveCAN::broadcastEncoder(float angleDeg) {
    uint8_t data[6];
    data[0] = _nodeId;
    data[1] = 0xFF; // broadcast
    memcpy(&data[2], &angleDeg, sizeof(float));
    send(BD_CAN_MSG_ENCODER, data, 6);
}

void BoxDriveCAN::sendHeartbeat() {
    uint8_t data[2];
    data[0] = _nodeId;
    data[1] = 0x01; // alive
    send(BD_CAN_MSG_HEARTBEAT, data, 2);
}

void BoxDriveCAN::sendCopyControl(float angleDeg) {
    uint8_t data[6];
    data[0] = _nodeId;       // leader ID
    data[1] = 0xFF;          // broadcast to all followers
    memcpy(&data[2], &angleDeg, sizeof(float));
    send(BD_CAN_MSG_COPY, data, 6);
}

float BoxDriveCAN::parseCopyControl(const uint8_t* data, uint8_t len) {
    if (len < 6) return -1.0f;
    float angle;
    memcpy(&angle, &data[2], sizeof(float));
    return angle;
}

void BoxDriveCAN::setFilter(uint32_t acceptId, uint32_t mask) {
    // Note: Filter must be set before starting TWAI.
    // For runtime filter changes, stop → reconfigure → start.
    (void)acceptId;
    (void)mask;
    // Implementation depends on ESP-IDF version; left as extension point.
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BoxDrive MAIN CLASS IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

BoxDrive::BoxDrive()
    : _motorEnabled(false)
    , _invertDirection(false)
    , _runCurrentMA(BD_DEFAULT_CURRENT)
    , _holdCurrentMA(BD_DEFAULT_CURRENT / 2)
    , _microsteps(16)
    , _speed(BD_DEFAULT_SPEED)
    , _acceleration(BD_DEFAULT_ACCEL)
    , _currentPosition(0)
    , _targetPosition(0)
    , _isMoving(false)
    , _mode(BoxDriveMode::STEALTH_CHOP)
    , _closedLoopEnabled(false)
    , _targetAngle(0.0f)
    , _kp(1.0f), _ki(0.002f), _kd(0.04f)
    , _pidIntegral(0.0f)
    , _pidPrevError(0.0f)
    , _pidFilteredError(0.0f)
    , _pidDErrorFiltered(0.0f)
    , _positionTolerance(0.5f)
    , _pidLastTime(0)
    , _pidLastStepTime(0)
    , _stallGuardEnabled(false)
    , _stallThreshold(100)
    , _stallCallback(nullptr)
    , _recordingCount(0)
    , _recordingStartTime(0)
    , _recordingDuration(5000)
    , _lastSampleTime(0)
    , _isRecording(false)
    , _isPlaying(false)
    , _playbackIndex(0)
    , _playbackSpeed(1.0f)
    , _playbackStartTime(0)
    , _recBaseAngle(0.0f)
    , _playBaseStep(0)
    , _lastPlayStepTime(0)
    , _canRole(BoxDriveCANRole::STANDALONE)
    , _leaderNodeId(0x01)
    , _lastCANBroadcast(0)
    , _hotspotActive(false)
    , _ledPattern(BoxDriveLED::OFF)
    , _ledLastToggle(0)
    , _ledState(false)
    , _ledBrightness(255)
    , _tmcSerial(nullptr)
{}

// ═══════════════════════════════════════════════════════════════════════════════
//  INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

bool BoxDrive::begin() {
    return begin(115200);
}

bool BoxDrive::begin(uint32_t baudRate) {
    Serial.begin(115200);
    delay(100);

    Serial.println(F("╔══════════════════════════════════════════╗"));
    Serial.println(F("║           BOXDRIVE v3.26                 ║"));
    Serial.println(F("║      BoxPox Motor Controller             ║"));
    Serial.println(F("╚══════════════════════════════════════════╝"));

    // Initialize GPIO pins
    _initPins();

    // Initialize TMC2209 UART
    _initUART(baudRate);

    // Initialize TMC2209 driver registers
    _initTMC2209();

    // Initialize AS5600 encoder
    bool encoderOk = encoder.begin();
    Serial.print(F("[ENCODER] AS5600: "));
    Serial.println(encoderOk ? F("OK") : F("NOT FOUND"));

    if (encoderOk) {
        Serial.print(F("[ENCODER] Magnet: "));
        if (encoder.magnetDetected()) {
            Serial.println(F("DETECTED"));
            Serial.print(F("[ENCODER] Initial angle: "));
            Serial.print(encoder.readAngleDeg(), 2);
            Serial.println(F("°"));
        } else {
            Serial.println(F("NOT DETECTED — Mount magnet on shaft!"));
        }
    }

    // Status LED: solid ON = ready
    setLED(BoxDriveLED::ON);

    Serial.println(F("[BOXDRIVE] Initialization complete."));
    Serial.println();

    return encoderOk;
}

void BoxDrive::_initPins() {
    pinMode(BD_PIN_DIR,  OUTPUT);
    pinMode(BD_PIN_STEP, OUTPUT);
    pinMode(BD_PIN_EN,   OUTPUT);
    pinMode(BD_PIN_DIAG, INPUT);
    pinMode(BD_PIN_PWM,  OUTPUT);
    pinMode(BD_PIN_LED,  OUTPUT);

    digitalWrite(BD_PIN_DIR,  LOW);
    digitalWrite(BD_PIN_STEP, LOW);
    digitalWrite(BD_PIN_EN,   HIGH); // Start disabled (EN is active LOW)
    digitalWrite(BD_PIN_LED,  LOW);

    Serial.println(F("[GPIO] Pins configured."));
}

void BoxDrive::_initUART(uint32_t baudRate) {
    // Use Serial1 for TMC2209 UART communication
    _tmcSerial = &Serial1;
    _tmcSerial->begin(baudRate, SERIAL_8N1, BD_PIN_UART_RX, BD_PIN_UART_TX);
    delay(50);
    Serial.print(F("[UART] TMC2209 UART initialized at "));
    Serial.print(baudRate);
    Serial.println(F(" baud."));
}

void BoxDrive::_initTMC2209() {
    // GCONF: Enable PDN_DISABLE for UART access, set MSTEP_REG_SELECT
    uint32_t gconf = 0;
    gconf |= (1 << TMC2209_PDN_DISABLE);      // Enable UART
    gconf |= (1 << TMC2209_MSTEP_REG_SELECT);  // Use MRES from register
    gconf |= (1 << TMC2209_MULTISTEP_FILT);    // Enable multistepping filter
    // StealthChop by default (EN_SPREADCYCLE = 0)
    writeRegister(TMC2209_GCONF, gconf);

    // CHOPCONF: Set microstepping and defaults
    uint32_t chopconf = 0;
    chopconf |= (4 << 24);   // MRES = 16 microsteps (value 4)
    chopconf |= (1 << 15);   // INTPOL: interpolation to 256
    chopconf |= (3 << 0);    // TOFF = 3
    chopconf |= (5 << 4);    // HSTRT = 5
    chopconf |= (2 << 7);    // HEND = 2
    chopconf |= (1 << 13);   // TBL = 1
    writeRegister(TMC2209_CHOPCONF, chopconf);

    // IHOLD_IRUN: Set default currents
    setCurrent(_runCurrentMA, _holdCurrentMA);

    // TPOWERDOWN: Motor standstill power down delay
    writeRegister(TMC2209_TPOWERDOWN, 20);

    // PWMCONF: StealthChop PWM configuration
    uint32_t pwmconf = 0;
    pwmconf |= (1 << 18);  // pwm_autoscale
    pwmconf |= (1 << 19);  // pwm_autograd
    pwmconf |= (1 << 0);   // PWM_OFS = 1
    pwmconf |= (36 << 8);  // PWM_GRAD = 36
    writeRegister(TMC2209_PWMCONF, pwmconf);

    // TPWMTHRS: StealthChop → SpreadCycle threshold
    writeRegister(TMC2209_TPWMTHRS, 0); // 0 = always StealthChop

    Serial.println(F("[TMC2209] Driver initialized (StealthChop, 16 µsteps)."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTOR ENABLE / DISABLE
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::enableMotor() {
    digitalWrite(BD_PIN_EN, LOW); // Active LOW
    _motorEnabled = true;
    Serial.println(F("[MOTOR] Enabled."));
}

void BoxDrive::disableMotor() {
    digitalWrite(BD_PIN_EN, HIGH);
    _motorEnabled = false;
    _isMoving = false;
    Serial.println(F("[MOTOR] Disabled (free-running)."));
}

bool BoxDrive::isMotorEnabled() const {
    return _motorEnabled;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CURRENT CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::setCurrent(uint16_t mA) {
    setCurrent(mA, mA / 2);
}

void BoxDrive::setHoldCurrent(uint16_t mA) {
    _holdCurrentMA = min(mA, (uint16_t)BD_MAX_CURRENT_MA);
    setCurrent(_runCurrentMA, _holdCurrentMA);
}

void BoxDrive::setCurrent(uint16_t runMA, uint16_t holdMA) {
    _runCurrentMA  = min(runMA,  (uint16_t)BD_MAX_CURRENT_MA);
    _holdCurrentMA = min(holdMA, (uint16_t)BD_MAX_CURRENT_MA);

    // Convert mA to TMC2209 current scale (0-31)
    // Rsense = 0.11Ω (typical for TMC2209 with internal Rsense)
    // CS = (I_rms * 32 * 1.41421 * Rsense / Vfs) - 1
    // With internal Rsense, Vfs ≈ 0.325V
    uint8_t irun  = (uint8_t)constrain((_runCurrentMA * 32 * 1.41421f * 0.11f / 325.0f) - 1, 0, 31);
    uint8_t ihold = (uint8_t)constrain((_holdCurrentMA * 32 * 1.41421f * 0.11f / 325.0f) - 1, 0, 31);
    uint8_t iholddelay = 6; // Smooth transition

    uint32_t iholdrun = 0;
    iholdrun |= ((uint32_t)ihold & 0x1F);
    iholdrun |= (((uint32_t)irun & 0x1F) << 8);
    iholdrun |= (((uint32_t)iholddelay & 0x0F) << 16);

    writeRegister(TMC2209_IHOLD_IRUN, iholdrun);
}

uint16_t BoxDrive::getCurrent() const {
    return _runCurrentMA;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DRIVER MODE
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::setMode(BoxDriveMode mode) {
    _mode = mode;
    uint32_t gconf = readRegister(TMC2209_GCONF);

    if (mode == BoxDriveMode::SPREAD_CYCLE) {
        gconf |= (1 << TMC2209_EN_SPREADCYCLE);
        Serial.println(F("[TMC2209] Mode: SpreadCycle (high torque)."));
    } else {
        gconf &= ~(1 << TMC2209_EN_SPREADCYCLE);
        Serial.println(F("[TMC2209] Mode: StealthChop (silent)."));
    }

    writeRegister(TMC2209_GCONF, gconf);
}

BoxDriveMode BoxDrive::getMode() const {
    return _mode;
}

void BoxDrive::setMicrosteps(uint16_t microsteps) {
    uint8_t mres;
    switch (microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case  64: mres = 2; break;
        case  32: mres = 3; break;
        case  16: mres = 4; break;
        case   8: mres = 5; break;
        case   4: mres = 6; break;
        case   2: mres = 7; break;
        case   1: mres = 8; break;
        default:  mres = 4; microsteps = 16; break; // Default 16
    }

    _microsteps = microsteps;

    uint32_t chopconf = readRegister(TMC2209_CHOPCONF);
    chopconf &= ~(0x0F << 24);     // Clear MRES bits
    chopconf |= ((uint32_t)mres << 24);
    writeRegister(TMC2209_CHOPCONF, chopconf);

    Serial.print(F("[TMC2209] Microsteps: "));
    Serial.println(microsteps);
}

uint16_t BoxDrive::getMicrosteps() const {
    return _microsteps;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BASIC MOTION (STEP/DIR)
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::setSpeed(float stepsPerSec) {
    _speed = abs(stepsPerSec);
}

void BoxDrive::setAcceleration(float stepsPerSec2) {
    _acceleration = abs(stepsPerSec2);
}

void BoxDrive::_stepPulse() {
    digitalWrite(BD_PIN_STEP, HIGH);
    delayMicroseconds(2);  // Minimum 100ns, use 2µs for safety
    digitalWrite(BD_PIN_STEP, LOW);
    delayMicroseconds(2);
}

void BoxDrive::_setDirection(BoxDriveDir dir) {
    bool pinState = (dir == BoxDriveDir::CW) ? LOW : HIGH;
    if (_invertDirection) pinState = !pinState;
    digitalWrite(BD_PIN_DIR, pinState ? HIGH : LOW);
    delayMicroseconds(5); // Direction setup time
}

void BoxDrive::invertDirection(bool invert) {
    _invertDirection = invert;
    Serial.print(F("[MOTOR] Direction inversion: "));
    Serial.println(invert ? F("INVERTED") : F("NORMAL"));
}

bool BoxDrive::isDirectionInverted() const {
    return _invertDirection;
}

void BoxDrive::move(int32_t steps) {
    if (!_motorEnabled) {
        Serial.println(F("[MOTOR] Cannot move — motor not enabled!"));
        return;
    }

    _isMoving = true;
    BoxDriveDir dir = (steps >= 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
    _setDirection(dir);
    int32_t absSteps = abs(steps);

    if (_acceleration <= 0) {
        // Constant speed (no acceleration profile)
        uint32_t stepDelayUs = (uint32_t)(1000000.0f / _speed);
        for (int32_t i = 0; i < absSteps; i++) {
            _stepPulse();
            delayMicroseconds(stepDelayUs);
            _currentPosition += (steps > 0) ? 1 : -1;
            if ((i & 0x1F) == 0) {
                encoder.readAngleDeg(); // Keep multi-turn tracking in sync
            }
        }
    } else {
        // Trapezoidal acceleration profile
        float currentSpeed = 100.0f; // Start speed
        float maxSpeed = _speed;
        uint32_t accelSteps = (uint32_t)((maxSpeed * maxSpeed - currentSpeed * currentSpeed) / (2.0f * _acceleration));
        if (accelSteps > (uint32_t)(absSteps / 2)) {
            accelSteps = absSteps / 2;
        }
        uint32_t decelStart = absSteps - accelSteps;

        for (int32_t i = 0; i < absSteps; i++) {
            if ((uint32_t)i < accelSteps) {
                // Accelerating
                currentSpeed = sqrt(100.0f * 100.0f + 2.0f * _acceleration * i);
                if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
            } else if ((uint32_t)i >= decelStart) {
                // Decelerating
                float remaining = absSteps - i;
                currentSpeed = sqrt(2.0f * _acceleration * remaining);
                if (currentSpeed < 100.0f) currentSpeed = 100.0f;
                if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
            }

            uint32_t stepDelayUs = (uint32_t)(1000000.0f / currentSpeed);
            _stepPulse();
            delayMicroseconds(stepDelayUs);
            _currentPosition += (steps > 0) ? 1 : -1;
            if ((i & 0x1F) == 0) {
                encoder.readAngleDeg(); // Keep multi-turn tracking in sync
            }
        }
    }

    encoder.readAngleDeg(); // Final sync
    _isMoving = false;
}

void BoxDrive::moveTo(int32_t position) {
    int32_t delta = position - _currentPosition;
    if (delta != 0) {
        move(delta);
    }
    _targetPosition = position;
}

void BoxDrive::moveToAngle(float angleDeg) {
    int32_t stepsPerRev = BD_STEPS_PER_REV * _microsteps;
    int32_t targetSteps = (int32_t)(angleDeg / 360.0f * stepsPerRev);
    moveTo(targetSteps);
}

void BoxDrive::rotate(BoxDriveDir dir) {
    if (!_motorEnabled) return;
    _isMoving = true;
    _setDirection(dir);

    // Continuous rotation: run until stop() is called
    // This is non-blocking by setting up a timer-based step generator
    // For simplicity, we use a flag-based approach; user calls stop()
    uint32_t stepDelayUs = (uint32_t)(1000000.0f / _speed);
    while (_isMoving) {
        _stepPulse();
        delayMicroseconds(stepDelayUs);
        _currentPosition += (dir == BoxDriveDir::CW) ? 1 : -1;
        yield(); // Allow ESP32 background tasks
    }
}

void BoxDrive::stop() {
    _isMoving = false;
}

void BoxDrive::softStop() {
    if (!_isMoving) return;

    // Decelerate from current speed to zero
    float currentSpeed = _speed;
    while (currentSpeed > 50.0f) {
        uint32_t stepDelayUs = (uint32_t)(1000000.0f / currentSpeed);
        _stepPulse();
        delayMicroseconds(stepDelayUs);
        currentSpeed -= _acceleration * (float)stepDelayUs / 1000000.0f;
        yield();
    }
    _isMoving = false;
}

int32_t BoxDrive::getPosition() const {
    return _currentPosition;
}

void BoxDrive::setPosition(int32_t pos) {
    _currentPosition = pos;
}

bool BoxDrive::isMoving() const {
    return _isMoving;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CLOSED-LOOP POSITION CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::enableClosedLoop(float kp, float ki, float kd) {
    _closedLoopEnabled = true;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _pidIntegral = 0.0f;
    _pidPrevError = 0.0f;
    _pidFilteredError = 0.0f;
    _pidDErrorFiltered = 0.0f;
    _pidLastTime = micros();
    _pidLastStepTime = micros();
    _targetAngle = encoder.readAngleDeg(); // Start at current position
    Serial.println(F("[CLOSED-LOOP] Enabled (Smooth vibration-free mode)."));
}

void BoxDrive::disableClosedLoop() {
    _closedLoopEnabled = false;
    Serial.println(F("[CLOSED-LOOP] Disabled."));
}

bool BoxDrive::isClosedLoopEnabled() const {
    return _closedLoopEnabled;
}

void BoxDrive::setTargetAngle(float angleDeg) {
    _targetAngle = angleDeg;
}

float BoxDrive::getTargetAngle() const {
    return _targetAngle;
}

float BoxDrive::getPositionError() {
    return _targetAngle - encoder.readAngleDeg();
}

void BoxDrive::updateClosedLoop() {
    // Never run PID closed loop during trajectory replay to prevent violent motor fighting!
    if (!_closedLoopEnabled || !_motorEnabled || _isPlaying) return;
    _pidCompute();
}

void BoxDrive::_pidCompute() {
    uint32_t now = micros();
    // Throttle PID calculation loop to 1000 Hz (1 ms interval)
    if (now - _pidLastTime < 1000) return;
    float dt = (now - _pidLastTime) / 1000000.0f;
    _pidLastTime = now;

    float currentAngle = encoder.readAngleDeg();
    float rawError = _targetAngle - currentAngle;

    // Shortest angular error (-180.0° to +180.0°)
    while (rawError > 180.0f) rawError -= 360.0f;
    while (rawError < -180.0f) rawError += 360.0f;

    // Filtered error to eliminate sensor noise jitter (AS5600 has ~0.15° noise)
    _pidFilteredError = (_pidFilteredError * 0.75f) + (rawError * 0.25f);
    float error = (abs(rawError) > 2.0f) ? rawError : _pidFilteredError;

    // Deadband check: if within tolerance, do not generate corrective pulses!
    // This completely prevents buzzing, twitching, and vibration while holding position!
    if (abs(error) <= _positionTolerance) {
        _pidIntegral = 0.0f;
        _pidPrevError = error;
        _pidDErrorFiltered = 0.0f;
        return;
    }

    // Integral with anti-windup clamping
    _pidIntegral += error * dt;
    _pidIntegral = constrain(_pidIntegral, -10.0f, 10.0f);

    // Filtered derivative to eliminate AS5600 ADC noise kick
    float dError = (error - _pidPrevError) / dt;
    _pidDErrorFiltered = (_pidDErrorFiltered * 0.85f) + (dError * 0.15f);
    _pidPrevError = error;

    float output = (_kp * error) + (_ki * _pidIntegral) + (_kd * _pidDErrorFiltered);

    // Convert degrees error to microsteps:
    // e.g. 16 microsteps * 200 = 3200 steps/rev / 360° = 8.889 steps/deg
    float stepsPerDeg = (float)(BD_STEPS_PER_REV * _microsteps) / 360.0f;
    int32_t targetSteps = (int32_t)(output * stepsPerDeg);

    if (targetSteps != 0) {
        // _setDirection already handles _invertDirection correctly!
        BoxDriveDir dir = (targetSteps > 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
        _setDirection(dir);

        // Smooth rate-limiting: take 1 microstep per cycle for small errors (<2°),
        // or up to 2 microsteps for larger deviations, with proper pulse spacing
        int32_t maxStepsThisCycle = (abs(error) > 3.0f) ? 2 : 1;
        int32_t absSteps = constrain(abs(targetSteps), 1, maxStepsThisCycle);

        uint32_t pulseIntervalUs = constrain((uint32_t)(1000000.0f / _speed), 400, 1500);

        for (int32_t i = 0; i < absSteps; i++) {
            _stepPulse();
            if (absSteps > 1) delayMicroseconds(pulseIntervalUs);
        }
        _currentPosition += (targetSteps > 0) ? absSteps : -absSteps;
    }
}

void BoxDrive::setPIDGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void BoxDrive::setPositionTolerance(float degrees) {
    _positionTolerance = degrees;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  STALLGUARD
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::enableStallGuard(uint8_t threshold) {
    _stallGuardEnabled = true;
    _stallThreshold = threshold;

    // Set StallGuard threshold
    writeRegister(TMC2209_SGTHRS, (uint32_t)threshold);

    // Enable DIAG output for StallGuard
    uint32_t gconf = readRegister(TMC2209_GCONF);
    gconf &= ~(1 << TMC2209_INDEX_OTPW); // Clear INDEX_OTPW
    writeRegister(TMC2209_GCONF, gconf);

    // Set TCOOLTHRS to enable StallGuard above a certain speed
    writeRegister(TMC2209_TCOOLTHRS, 0xFFFFF); // Always active

    Serial.print(F("[STALLGUARD] Enabled, threshold: "));
    Serial.println(threshold);
}

void BoxDrive::disableStallGuard() {
    _stallGuardEnabled = false;
    writeRegister(TMC2209_SGTHRS, 0);
    writeRegister(TMC2209_TCOOLTHRS, 0);
    Serial.println(F("[STALLGUARD] Disabled."));
}

uint16_t BoxDrive::readStallGuardValue() {
    uint32_t result = readRegister(TMC2209_SG_RESULT);
    return (uint16_t)(result & 0x3FF);
}

bool BoxDrive::isStalled() {
    return (digitalRead(BD_PIN_DIAG) == HIGH);
}

void BoxDrive::onStall(void (*callback)()) {
    _stallCallback = callback;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTION RECORDING & PLAYBACK
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::startRecording(uint32_t durationMs) {
    _recordingCount = 0;
    _recordingDuration = min(durationMs, (uint32_t)10000); // Up to 10 seconds
    _recordingStartTime = millis();
    _lastSampleTime = _recordingStartTime;
    _isRecording = true;
    _isPlaying = false;

    // Release coils so the user can freely rotate the shaft by hand
    disableMotor();
    setLED(BoxDriveLED::BLINK_FAST);

    // Capture starting multi-turn baseline
    _recBaseAngle = encoder.readMultiTurnDeg();

    // First sample at t = 0 ms, delta = 0.0°
    _recordingBuffer[0].timestampMs = 0;
    _recordingBuffer[0].angleDeg = 0.0f;
    _recordingCount = 1;

    Serial.print(F("[RECORD] Hand kinesthetic recording started. Duration: "));
    Serial.print(_recordingDuration);
    Serial.println(F(" ms. Rotate motor shaft freely by hand now!"));
}

void BoxDrive::stopRecording() {
    if (!_isRecording) return;
    _isRecording = false;
    setLED(BoxDriveLED::BLINK_SLOW);
    Serial.print(F("[RECORD] Stopped. Captured samples: "));
    Serial.println(_recordingCount);
}

bool BoxDrive::isRecording() const {
    return _isRecording;
}

uint16_t BoxDrive::getRecordingSampleCount() const {
    return _recordingCount;
}

void BoxDrive::_recordSample() {
    if (_recordingCount >= BD_MAX_RECORDING_SAMPLES) {
        stopRecording();
        return;
    }

    _recordingBuffer[_recordingCount].timestampMs = millis() - _recordingStartTime;
    // Continuous angular delta from the gesture start pose
    _recordingBuffer[_recordingCount].angleDeg = encoder.readMultiTurnDeg() - _recBaseAngle;
    _recordingCount++;
}

void BoxDrive::playRecording(float speedMultiplier) {
    if (_recordingCount < 2) {
        Serial.println(F("[PLAYBACK] No recorded motion buffer to play!"));
        return;
    }

    _isPlaying = true;
    _isRecording = false;
    _playbackIndex = 0;
    _playbackSpeed = (speedMultiplier > 0.05f) ? speedMultiplier : 1.0f;
    _playbackStartTime = millis();
    _playBaseStep = _currentPosition;
    _lastPlayStepTime = micros();

    // Energize motor coils
    enableMotor();
    setLED(BoxDriveLED::BLINK_FAST);

    Serial.print(F("[PLAYBACK] Smooth continuous replay started. Samples: "));
    Serial.print(_recordingCount);
    Serial.print(F(", Speed: "));
    Serial.print(_playbackSpeed, 1);
    Serial.println(F("x"));
}

bool BoxDrive::isPlaying() const {
    return _isPlaying;
}

void BoxDrive::stopPlayback() {
    if (!_isPlaying) return;
    _isPlaying = false;
    setLED(BoxDriveLED::BLINK_SLOW);
    Serial.println(F("[PLAYBACK] Playback finished."));
}

void BoxDrive::_playSample() {
    if (!_isPlaying || _recordingCount < 2) return;

    uint32_t nowMs = millis();
    uint32_t elapsed = (uint32_t)((nowMs - _playbackStartTime) * _playbackSpeed);
    uint32_t totalDuration = _recordingBuffer[_recordingCount - 1].timestampMs;
    int32_t stepsPerRev = BD_STEPS_PER_REV * _microsteps;

    if (elapsed >= totalDuration) {
        // Reached end of trajectory — ensure we smoothly reach exact final step
        float finalAngle = _recordingBuffer[_recordingCount - 1].angleDeg;
        int32_t finalTargetStep = _playBaseStep + (int32_t)((finalAngle / 360.0f) * stepsPerRev);

        if (_currentPosition != finalTargetStep) {
            uint32_t minStepIntervalUs = (uint32_t)(1000000.0f / _speed);
            if (minStepIntervalUs < 100) minStepIntervalUs = 100;
            uint32_t nowUs = micros();
            if (nowUs - _lastPlayStepTime >= minStepIntervalUs) {
                _lastPlayStepTime = nowUs;
                int32_t diff = finalTargetStep - _currentPosition;
                BoxDriveDir dir = (diff > 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
                _setDirection(dir);
                _stepPulse();
                _currentPosition += (diff > 0) ? 1 : -1;
                _targetAngle = encoder.readAngleDeg();
            }
            return;
        }

        stopPlayback();
        return;
    }

    // Advance _playbackIndex to bracket the current elapsed time
    while (_playbackIndex < _recordingCount - 2 && 
           _recordingBuffer[_playbackIndex + 1].timestampMs <= elapsed) {
        _playbackIndex++;
    }

    uint32_t t0 = _recordingBuffer[_playbackIndex].timestampMs;
    uint32_t t1 = _recordingBuffer[_playbackIndex + 1].timestampMs;
    float a0 = _recordingBuffer[_playbackIndex].angleDeg;
    float a1 = _recordingBuffer[_playbackIndex + 1].angleDeg;

    float frac = 0.0f;
    if (t1 > t0) {
        frac = (float)(elapsed - t0) / (float)(t1 - t0);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
    }

    // Interpolate continuous sub-sample angle
    float targetDeltaAngle = a0 + frac * (a1 - a0);
    int32_t targetStep = _playBaseStep + (int32_t)((targetDeltaAngle / 360.0f) * stepsPerRev);

    int32_t stepDiff = targetStep - _currentPosition;
    if (stepDiff != 0) {
        uint32_t minStepIntervalUs = (uint32_t)(1000000.0f / _speed);
        if (minStepIntervalUs < 100) minStepIntervalUs = 100;
        uint32_t nowUs = micros();
        if (nowUs - _lastPlayStepTime >= minStepIntervalUs) {
            _lastPlayStepTime = nowUs;
            BoxDriveDir dir = (stepDiff > 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
            _setDirection(dir);
            _stepPulse();
            _currentPosition += (stepDiff > 0) ? 1 : -1;
            _targetAngle = encoder.readAngleDeg();
        }
    }
}

const MotionSample* BoxDrive::getRecordingBuffer(uint16_t& count) const {
    count = _recordingCount;
    return _recordingBuffer;
}

void BoxDrive::loadRecording(const MotionSample* samples, uint16_t count) {
    uint16_t copyCount = min(count, (uint16_t)BD_MAX_RECORDING_SAMPLES);
    memcpy(_recordingBuffer, samples, copyCount * sizeof(MotionSample));
    _recordingCount = copyCount;
    Serial.print(F("[RECORD] Loaded "));
    Serial.print(copyCount);
    Serial.println(F(" samples."));
}

void BoxDrive::updateRecording() {
    if (_isRecording) {
        uint32_t now = millis();
        if (now - _lastSampleTime >= BD_RECORDING_INTERVAL_MS) {
            _recordSample();
            _lastSampleTime = now;
        }

        // Auto-stop after duration
        if (now - _recordingStartTime >= _recordingDuration) {
            stopRecording();
        }
    }

    if (_isPlaying) {
        _playSample();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CAN BUS COPY-CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::setCANLeader() {
    _canRole = BoxDriveCANRole::LEADER;
    Serial.println(F("[CAN] Role: LEADER (broadcasting encoder)."));
}

void BoxDrive::setCANFollower(uint8_t leaderNodeId) {
    _canRole = BoxDriveCANRole::FOLLOWER;
    _leaderNodeId = leaderNodeId;
    if (!_closedLoopEnabled) {
        enableClosedLoop();
    }
    Serial.print(F("[CAN] Role: FOLLOWER (mirroring node 0x"));
    Serial.print(leaderNodeId, HEX);
    Serial.println(F(")."));
}

void BoxDrive::setCANStandalone() {
    _canRole = BoxDriveCANRole::STANDALONE;
    Serial.println(F("[CAN] Role: STANDALONE."));
}

BoxDriveCANRole BoxDrive::getCANRole() const {
    return _canRole;
}

void BoxDrive::updateCAN() {
    if (_canRole == BoxDriveCANRole::STANDALONE) return;

    uint32_t now = millis();

    if (_canRole == BoxDriveCANRole::LEADER) {
        // Broadcast encoder position at ~100 Hz
        if (now - _lastCANBroadcast >= 10) {
            float angle = encoder.readAngleDeg();
            can.sendCopyControl(angle);
            _lastCANBroadcast = now;
        }
    }
    else if (_canRole == BoxDriveCANRole::FOLLOWER) {
        // Check for incoming copy-control messages
        uint32_t msgId;
        uint8_t  data[8];
        uint8_t  len;

        while (can.available()) {
            if (can.receive(msgId, data, len)) {
                if (msgId == BD_CAN_MSG_COPY) {
                    uint8_t senderId = data[0];
                    if (senderId == _leaderNodeId) {
                        float targetAngle = can.parseCopyControl(data, len);
                        if (targetAngle >= 0) {
                            setTargetAngle(targetAngle);
                        }
                    }
                }
            }
        }
        // Update closed-loop to follow the target
        updateClosedLoop();
    }

    // Heartbeat every 1 second
    static uint32_t lastHeartbeat = 0;
    if (now - lastHeartbeat >= 1000) {
        can.sendHeartbeat();
        lastHeartbeat = now;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WIFI HOTSPOT CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

#ifdef ESP32
static WebServer* _webServer = nullptr;
static BoxDrive*  _hotspotInstance = nullptr;
#endif

void BoxDrive::startHotspot(const char* ssid, const char* password) {
#ifdef ESP32
    WiFi.softAP(ssid, password);
    IPAddress ip = WiFi.softAPIP();

    _webServer = new WebServer(80);
    _hotspotInstance = this;

    // Root page — web-based motor control dashboard
    _webServer->on("/", HTTP_GET, [this]() {
        String html;
        _buildWebPage(html);
        _webServer->send(200, "text/html", html);
    });

    // API: Enable motor
    _webServer->on("/api/enable", HTTP_POST, [this]() {
        enableMotor();
        _webServer->send(200, "application/json", "{\"status\":\"enabled\"}");
    });

    // API: Disable motor
    _webServer->on("/api/disable", HTTP_POST, [this]() {
        disableMotor();
        _webServer->send(200, "application/json", "{\"status\":\"disabled\"}");
    });

    // API: Move to angle or steps
    _webServer->on("/api/move", HTTP_GET, [this]() {
        if (_webServer->hasArg("angle")) {
            float angle = _webServer->arg("angle").toFloat();
            if (_closedLoopEnabled) {
                setTargetAngle(angle);
            } else {
                moveToAngle(angle);
            }
            _webServer->send(200, "application/json", 
                "{\"status\":\"moving\",\"target\":" + String(angle, 2) + "}");
        } else if (_webServer->hasArg("steps")) {
            int32_t steps = _webServer->arg("steps").toInt();
            move(steps);
            _webServer->send(200, "application/json",
                "{\"status\":\"moving\",\"steps\":" + String(steps) + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing angle or steps\"}");
        }
    });

    // API: Toggle closed loop
    _webServer->on("/api/closedloop", HTTP_GET, [this]() {
        if (_webServer->hasArg("enable")) {
            bool en = (_webServer->arg("enable") == "1" || _webServer->arg("enable") == "true");
            if (en) enableClosedLoop();
            else disableClosedLoop();
            _webServer->send(200, "application/json", "{\"status\":\"ok\",\"closed_loop\":" + String(en ? "true" : "false") + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing enable\"}");
        }
    });

    // API: Toggle direction inversion
    _webServer->on("/api/invert", HTTP_GET, [this]() {
        if (_webServer->hasArg("inv")) {
            bool inv = (_webServer->arg("inv") == "1" || _webServer->arg("inv") == "true");
            invertDirection(inv);
            _webServer->send(200, "application/json", "{\"status\":\"ok\",\"inverted\":" + String(inv ? "true" : "false") + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing inv\"}");
        }
    });

    // API: Kinesthetic hand teaching record & playback
    _webServer->on("/api/record", HTTP_GET, [this]() {
        if (_webServer->hasArg("action")) {
            String act = _webServer->arg("action");
            if (act == "start") {
                startRecording(5000);
                _webServer->send(200, "application/json", "{\"status\":\"recording\"}");
            } else if (act == "stop") {
                stopRecording();
                _webServer->send(200, "application/json", "{\"status\":\"stopped\",\"samples\":" + String(_recordingCount) + "}");
            } else if (act == "play") {
                float spd = _webServer->hasArg("speed") ? _webServer->arg("speed").toFloat() : 1.0f;
                playRecording(spd);
                _webServer->send(200, "application/json", "{\"status\":\"playing\",\"speed\":" + String(spd) + "}");
            } else {
                _webServer->send(400, "application/json", "{\"error\":\"unknown action\"}");
            }
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing action\"}");
        }
    });

    // API: Set current
    _webServer->on("/api/current", HTTP_GET, [this]() {
        if (_webServer->hasArg("ma")) {
            uint16_t ma = (uint16_t)_webServer->arg("ma").toInt();
            setCurrent(ma);
            _webServer->send(200, "application/json",
                "{\"status\":\"ok\",\"current\":" + String(ma) + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing ma\"}");
        }
    });

    // API: Set speed
    _webServer->on("/api/speed", HTTP_GET, [this]() {
        if (_webServer->hasArg("sps")) {
            float sps = _webServer->arg("sps").toFloat();
            setSpeed(sps);
            _webServer->send(200, "application/json",
                "{\"status\":\"ok\",\"speed\":" + String(sps) + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing sps\"}");
        }
    });

    // API: Get status (JSON)
    _webServer->on("/api/status", HTTP_GET, [this]() {
        float angle = encoder.readAngleDeg();
        float vel = encoder.getVelocityDPS();
        uint8_t agc = encoder.readAGC();
        bool mag = encoder.magnetDetected();
        bool ot = isOverTemp();
        String json = "{";
        json += "\"angle\":" + String(angle, 2) + ",";
        json += "\"multi_angle\":" + String(encoder.readMultiTurnDeg(), 2) + ",";
        json += "\"target\":" + String(_targetAngle, 2) + ",";
        json += "\"error\":" + String(_targetAngle - angle, 2) + ",";
        json += "\"velocity\":" + String(vel, 1) + ",";
        json += "\"position\":" + String(_currentPosition) + ",";
        json += "\"enabled\":" + String(_motorEnabled ? "true" : "false") + ",";
        json += "\"closed_loop\":" + String(_closedLoopEnabled ? "true" : "false") + ",";
        json += "\"inverted\":" + String(_invertDirection ? "true" : "false") + ",";
        json += "\"current\":" + String(_runCurrentMA) + ",";
        json += "\"microsteps\":" + String(_microsteps) + ",";
        json += "\"speed\":" + String(_speed, 0) + ",";
        json += "\"mode\":\"" + String(_mode == BoxDriveMode::STEALTH_CHOP ? "stealth" : "spread") + "\",";
        json += "\"magnet\":" + String(mag ? "true" : "false") + ",";
        json += "\"agc\":" + String(agc) + ",";
        json += "\"overtemp\":" + String(ot ? "true" : "false") + ",";
        json += "\"recording\":" + String(_isRecording ? "true" : "false") + ",";
        json += "\"playing\":" + String(_isPlaying ? "true" : "false") + ",";
        json += "\"samples\":" + String(_recordingCount);
        json += "}";
        _webServer->send(200, "application/json", json);
    });

    // API: Set mode
    _webServer->on("/api/mode", HTTP_GET, [this]() {
        if (_webServer->hasArg("m")) {
            String m = _webServer->arg("m");
            if (m == "stealth") setMode(BoxDriveMode::STEALTH_CHOP);
            else if (m == "spread") setMode(BoxDriveMode::SPREAD_CYCLE);
            _webServer->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing m\"}");
        }
    });

    // API: Set microsteps
    _webServer->on("/api/microsteps", HTTP_GET, [this]() {
        if (_webServer->hasArg("ms")) {
            uint16_t ms = (uint16_t)_webServer->arg("ms").toInt();
            setMicrosteps(ms);
            _webServer->send(200, "application/json",
                "{\"status\":\"ok\",\"microsteps\":" + String(ms) + "}");
        } else {
            _webServer->send(400, "application/json", "{\"error\":\"missing ms\"}");
        }
    });

    // API: Stop motor
    _webServer->on("/api/stop", HTTP_POST, [this]() {
        stop();
        _webServer->send(200, "application/json", "{\"status\":\"stopped\"}");
    });

    _webServer->begin();
    _hotspotActive = true;

    Serial.println(F("[HOTSPOT] WiFi AP started."));
    Serial.print(F("[HOTSPOT] SSID: "));
    Serial.println(ssid);
    Serial.print(F("[HOTSPOT] IP: "));
    Serial.println(ip);
    Serial.println(F("[HOTSPOT] Open http://192.168.4.1 in your browser."));
#endif
}

void BoxDrive::stopHotspot() {
#ifdef ESP32
    if (_webServer) {
        _webServer->stop();
        delete _webServer;
        _webServer = nullptr;
    }
    WiFi.softAPdisconnect(true);
    _hotspotActive = false;
    _hotspotInstance = nullptr;
    Serial.println(F("[HOTSPOT] Stopped."));
#endif
}

bool BoxDrive::isHotspotActive() const {
    return _hotspotActive;
}

void BoxDrive::handleHotspot() {
#ifdef ESP32
    if (_hotspotActive && _webServer) {
        _webServer->handleClient();
    }
#endif
    if (_isRecording || _isPlaying) {
        updateRecording();
    }
}

void BoxDrive::_buildWebPage(String& html) {
    html = F(R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>BoxDrive • Precision Closed-Loop Stepper • boxpox.in</title>
<style>
:root{
  --bg:#f8fafc;
  --surface:#ffffff;
  --border:#e2e8f0;
  --border-sub:#cbd5e1;
  --text:#0f172a;
  --text-muted:#64748b;
  --yellow:#facc15;
  --yellow-dark:#eab308;
  --yellow-light:#fef08a;
  --yellow-bg:#fefce8;
  --green:#16a34a;
  --green-bg:#f0fdf4;
  --red:#dc2626;
  --red-bg:#fef2f2;
  --blue:#0284c7;
  --blue-bg:#f0f9ff;
  --purple:#7c3aed;
  --purple-bg:#faf5ff;
}
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif}
body{background:var(--bg);color:var(--text);min-height:100vh;padding-bottom:3rem;-webkit-font-smoothing:antialiased}

/* HEADER NAVBAR */
.header{background:var(--surface);border-bottom:1px solid var(--border);padding:0.9rem 1.5rem;box-shadow:0 1px 3px rgba(0,0,0,0.03)}
.header-inner{max-width:1140px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:0.75rem}
.header-left{display:flex;align-items:center;gap:12px}
.brand{display:flex;align-items:center;gap:10px;text-decoration:none;color:var(--text)}
.logo-icon{width:34px;height:34px;background:var(--yellow);color:var(--text);border-radius:8px;display:flex;align-items:center;justify-content:center;font-size:18px;font-weight:700;box-shadow:0 2px 6px rgba(234,179,8,0.25)}
.brand-text{display:flex;flex-direction:column}
.brand-title{font-size:1.25rem;font-weight:300;letter-spacing:1.5px;color:var(--text);line-height:1.1}
.brand-title b{font-weight:600}
.brand-sub{font-size:0.72rem;color:var(--text-muted);font-weight:400;letter-spacing:0.5px}
.brand-sub a{color:var(--yellow-dark);text-decoration:none;font-weight:500}
.header-right{display:flex;align-items:center;gap:10px}
.status-pill{display:inline-flex;align-items:center;gap:7px;background:var(--green-bg);border:1px solid #bbf7d0;color:#166534;padding:5px 12px;border-radius:99px;font-size:0.72rem;font-weight:500;letter-spacing:0.5px}
.pulsing{width:7px;height:7px;border-radius:50%;background:#22c55e;box-shadow:0 0 6px #22c55e;animation:pulse 1.6s infinite}
@keyframes pulse{0%{opacity:1;transform:scale(1)}50%{opacity:0.35;transform:scale(0.8)}100%{opacity:1;transform:scale(1)}}
.site-link{color:var(--text);text-decoration:none;font-size:0.75rem;font-weight:500;padding:5px 12px;border:1px solid var(--border);border-radius:8px;background:var(--bg);transition:all 0.15s}
.site-link:hover{background:var(--yellow-light);border-color:var(--yellow)}

/* LAYOUT CONTAINER */
.container{max-width:1140px;margin:1.5rem auto;padding:0 1.25rem;display:grid;grid-template-columns:1fr;gap:1.25rem}
@media(min-width:992px){.container{grid-template-columns:460px 1fr}}

/* CARDS */
.card{background:var(--surface);border:1px solid var(--border);border-radius:14px;padding:1.25rem;box-shadow:0 1px 3px rgba(0,0,0,0.03),0 6px 16px -4px rgba(0,0,0,0.02)}
.card-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:1rem;border-bottom:1px solid var(--border);padding-bottom:0.6rem}
.card-title{font-size:0.8rem;font-weight:600;text-transform:uppercase;letter-spacing:1px;color:var(--text);display:flex;align-items:center;gap:7px}

/* MINIMALIST ANGLE GAUGE */
.dial-box{width:210px;height:210px;margin:0 auto 0.5rem;position:relative}
.dial-svg{width:100%;height:100%;display:block}
.gauge-readout{text-align:center;padding:0.4rem 0 0.8rem}
.gauge-angle-row{display:inline-flex;align-items:baseline;justify-content:center}
.gauge-angle-val{font-size:2.4rem;font-weight:300;color:var(--text);letter-spacing:-0.5px;line-height:1}
.gauge-angle-deg{font-size:1.5rem;font-weight:300;color:var(--yellow-dark);margin-left:2px}
.gauge-label{font-size:0.65rem;font-weight:600;color:var(--text-muted);letter-spacing:1.5px;text-transform:uppercase;margin-top:3px}
.gauge-pills{display:flex;gap:8px;justify-content:center;margin-top:7px}
.pill{font-size:0.7rem;padding:3px 9px;border-radius:6px;font-weight:400}
.pill-target{background:var(--blue-bg);color:#0369a1;border:1px solid #bae6fd}
.pill-error{background:var(--yellow-bg);color:#854d0e;border:1px solid #fef08a}

/* STATS GRID */
.grid-2{display:grid;grid-template-columns:1fr 1fr;gap:0.6rem}
.grid-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:0.5rem}
.stat-box{background:var(--bg);border:1px solid var(--border);padding:0.7rem 0.5rem;border-radius:10px;text-align:center}
.stat-val{font-size:1.25rem;font-weight:300;color:var(--text)}
.stat-lbl{font-size:0.65rem;font-weight:500;text-transform:uppercase;color:var(--text-muted);letter-spacing:0.5px;margin-top:3px}

/* BUTTONS */
.btn{background:var(--surface);color:var(--text);border:1px solid var(--border-sub);padding:0.65rem 0.9rem;border-radius:8px;font-weight:400;font-size:0.8rem;cursor:pointer;transition:all 0.15s;display:inline-flex;align-items:center;justify-content:center;gap:6px;user-select:none}
.btn:hover{background:var(--bg);border-color:var(--text)}
.btn:active{transform:scale(0.98)}
.btn-yellow{background:var(--yellow);border-color:var(--yellow-dark);color:var(--text);font-weight:500}
.btn-yellow:hover{background:#fde047}
.btn-green{background:var(--green-bg);border-color:#86efac;color:#15803d}
.btn-green:hover{background:#dcfce7}
.btn-red{background:var(--red-bg);border-color:#fca5a5;color:#b91c1c}
.btn-red:hover{background:#fee2e2}
.btn-blue{background:var(--blue-bg);border-color:#7dd3fc;color:#0369a1}
.btn-purple{background:var(--purple-bg);border-color:#d8b4fe;color:#6b21a8}
.btn-active{box-shadow:0 0 0 2px currentColor}
.full-width{width:100%}

/* FORM CONTROLS */
.row{display:flex;gap:0.6rem;align-items:center;margin-bottom:0.7rem}
.row-label{font-size:0.78rem;color:var(--text-muted);min-width:85px;font-weight:400}
input[type=range]{flex:1;accent-color:var(--yellow-dark);height:5px;background:var(--border);border-radius:4px;outline:none}
input[type=number]{background:var(--surface);border:1px solid var(--border-sub);color:var(--text);padding:0.5rem;border-radius:7px;width:75px;text-align:center;font-weight:400;font-size:0.85rem}
select{background:var(--surface);border:1px solid var(--border-sub);color:var(--text);padding:0.5rem;border-radius:7px;flex:1;font-weight:400;font-size:0.82rem}

/* CHARTS */
.chart-box{background:var(--bg);border:1px solid var(--border);border-radius:10px;padding:0.7rem;margin-top:0.7rem}
.chart-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px;font-size:0.7rem;color:var(--text-muted);font-weight:500;text-transform:uppercase}
canvas{width:100%;height:100px;display:block}

/* PROGRESS BAR */
.progress-bar{height:5px;background:var(--border);border-radius:3px;overflow:hidden;margin-top:8px;display:none}
.progress-inner{height:100%;width:0%;background:linear-gradient(90deg,var(--yellow),var(--red));transition:width 0.08s linear}

/* FOOTER */
.footer{text-align:center;padding:2rem 1rem 0;color:var(--text-muted);font-size:0.75rem;font-weight:400;grid-column:1/-1;border-top:1px solid var(--border);margin-top:1rem;line-height:1.6}
.footer a{color:var(--text);text-decoration:none;font-weight:500}
.footer a:hover{text-decoration:underline}
</style>
</head>
<body>

<header class="header">
  <div class="header-inner">
    <div class="header-left">
      <a class="brand" href="https://boxpox.in" target="_blank">
        <div class="logo-icon">⬡</div>
        <div class="brand-text">
          <span class="brand-title">BOX<b>DRIVE</b></span>
          <span class="brand-sub">Hardware v3.26 &bull; <a href="https://boxpox.in" target="_blank">boxpox.in</a></span>
        </div>
      </a>
    </div>
    <div class="header-right">
      <div class="status-pill"><span class="pulsing"></span> ONLINE &bull; 192.168.4.1</div>
      <a class="site-link" href="https://boxpox.in" target="_blank">boxpox.in ↗</a>
    </div>
  </div>
</header>

<div class="container">

  <!-- LEFT COLUMN: LIVE TELEMETRY & PRECISION GAUGE -->
  <div style="display:flex;flex-direction:column;gap:1.25rem;">
    
    <!-- MINIMALIST SHAFT ORIENTATION GAUGE CARD -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>🧭</span> Shaft Orientation</span>
        <span id="cl-badge" style="font-size:0.7rem;font-weight:500;color:var(--text-muted);">OPEN-LOOP</span>
      </div>

      <div class="dial-box">
        <svg class="dial-svg" viewBox="0 0 200 200">
          <!-- Background Base Ring -->
          <circle cx="100" cy="100" r="95" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.5"/>
          <circle cx="100" cy="100" r="91" fill="#f8fafc" stroke="#f1f5f9" stroke-width="1"/>

          <!-- 10° Minor Graduation Ticks -->
          <g stroke="#cbd5e1" stroke-width="0.8">
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(10 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(20 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(40 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(50 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(70 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(80 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(100 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(110 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(130 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(140 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(160 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(170 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(190 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(200 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(220 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(230 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(250 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(260 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(280 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(290 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(310 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(320 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(340 100 100)"/>
            <line x1="100" y1="12" x2="100" y2="15" transform="rotate(350 100 100)"/>
          </g>

          <!-- 30° Major Ticks -->
          <g stroke="#94a3b8" stroke-width="1.5">
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(30 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(60 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(120 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(150 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(210 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(240 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(300 100 100)"/>
            <line x1="100" y1="10" x2="100" y2="16" transform="rotate(330 100 100)"/>
          </g>

          <!-- Cardinal Ticks & Minimalist Numbers (Clean, Thin, Uncluttered) -->
          <line x1="100" y1="9" x2="100" y2="18" stroke="#0f172a" stroke-width="2"/>
          <line x1="191" y1="100" x2="182" y2="100" stroke="#0f172a" stroke-width="2"/>
          <line x1="100" y1="191" x2="100" y2="182" stroke="#0f172a" stroke-width="2"/>
          <line x1="9" y1="100" x2="18" y2="100" stroke="#0f172a" stroke-width="2"/>

          <text x="100" y="27" fill="#64748b" font-size="8" font-weight="500" text-anchor="middle">0°</text>
          <text x="173" y="103" fill="#64748b" font-size="8" font-weight="500" text-anchor="middle">90°</text>
          <text x="100" y="177" fill="#64748b" font-size="8" font-weight="500" text-anchor="middle">180°</text>
          <text x="27" y="103" fill="#64748b" font-size="8" font-weight="500" text-anchor="middle">270°</text>

          <!-- Rail Track (R=84, Circumference=527.79) -->
          <circle cx="100" cy="100" r="84" fill="none" stroke="#e2e8f0" stroke-width="3.5"/>

          <!-- Active Sweeping Yellow Arc -->
          <circle id="gauge-arc" cx="100" cy="100" r="84" fill="none" stroke="#eab308" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="527.79" stroke-dashoffset="527.79" transform="rotate(-90 100 100)"/>

          <!-- Target Pip (Clean Sky-Blue Diamond) -->
          <g id="target-pip" transform="rotate(0 100 100)">
            <polygon points="100,20 97,14 103,14" fill="#0284c7"/>
          </g>

          <!-- Dial Center Plate & Thin Watermark -->
          <circle cx="100" cy="100" r="54" fill="#ffffff" stroke="#f1f5f9" stroke-width="1"/>
          <text x="100" y="74" fill="#cbd5e1" font-size="7" font-weight="500" letter-spacing="1.5" text-anchor="middle">AS5600 12-BIT</text>

          <!-- Razor-Thin Needle Pointer -->
          <g id="needle-group" transform="rotate(0 100 100)">
            <!-- Needle blade -->
            <polygon points="99,100 99.5,32 100,24 100.5,32 101,100" fill="#0f172a"/>
            <!-- Yellow accent needle tip -->
            <polygon points="99.5,32 100,24 100.5,32" fill="#eab308"/>
            <!-- Counterweight tail -->
            <polygon points="99,100 99.4,112 100.6,112 101,100" fill="#94a3b8"/>
            <!-- Center Pivot Hub -->
            <circle cx="100" cy="100" r="5.5" fill="#0f172a"/>
            <circle cx="100" cy="100" r="2" fill="#facc15"/>
          </g>
        </svg>
      </div>

      <!-- FORMATTED DIGITAL READOUT (BELOW DIAL) -->
      <div class="gauge-readout">
        <div class="gauge-angle-row">
          <span class="gauge-angle-val" id="d-angle">0.0</span>
          <span class="gauge-angle-deg">°</span>
        </div>
        <div class="gauge-label">Measured Shaft Angle</div>
        <div class="gauge-pills">
          <span class="pill pill-target">Target: <b id="d-target">0.0°</b></span>
          <span class="pill pill-error">Error: <b id="d-err">0.0°</b></span>
        </div>
      </div>

      <div class="grid-3">
        <div class="stat-box"><div class="stat-val" id="d-vel">0.0</div><div class="stat-lbl">Speed (°/s)</div></div>
        <div class="stat-box"><div class="stat-val" id="d-pos">0</div><div class="stat-lbl">Steps</div></div>
        <div class="stat-box"><div class="stat-val" id="d-cur">800</div><div class="stat-lbl">mA Current</div></div>
      </div>
    </div>

    <!-- REAL-TIME GRAPHS CARD -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>📈</span> Telemetry Graphs</span>
        <span style="font-size:0.7rem;color:var(--text-muted);font-weight:400;">Live Feed</span>
      </div>

      <div class="chart-box">
        <div class="chart-header">
          <span>Shaft Angle vs Target (0° – 360°)</span>
          <span><span style="color:var(--yellow-dark)">■ Angle</span> &bull; <span style="color:var(--blue)">┄ Target</span></span>
        </div>
        <canvas id="c-angle"></canvas>
      </div>

      <div class="chart-box">
        <div class="chart-header">
          <span>Angular Velocity (°/sec)</span>
          <span style="color:var(--green)">■ Velocity</span>
        </div>
        <canvas id="c-vel"></canvas>
      </div>
    </div>

    <!-- HARDWARE DIAGNOSTICS CARD -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>🩺</span> Hardware Diagnostics</span>
        <a href="https://boxpox.in" target="_blank" style="font-size:0.72rem;color:var(--text-muted);text-decoration:none;">boxpox.in</a>
      </div>
      <div class="grid-3">
        <div class="stat-box"><div class="stat-val" id="d-mag" style="color:var(--green);font-size:1.1rem;font-weight:500;">✓ OK</div><div class="stat-lbl">AS5600 Magnet</div></div>
        <div class="stat-box"><div class="stat-val" id="d-agc">--</div><div class="stat-lbl">AGC Gain</div></div>
        <div class="stat-box"><div class="stat-val" id="d-temp" style="color:var(--green);font-size:1.1rem;font-weight:500;">COOL</div><div class="stat-lbl">TMC2209 Temp</div></div>
      </div>
    </div>

  </div>

  <!-- RIGHT COLUMN: CONTROLS & MOTOR ACTIONS -->
  <div style="display:flex;flex-direction:column;gap:1.25rem;">
    
    <!-- POWER & CORE DRIVER MODES -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>⚡</span> Power & Core Modes</span>
      </div>
      
      <div class="grid-2" style="margin-bottom:0.6rem;">
        <button id="b-power" class="btn btn-green full-width" onclick="togglePower()">Energized (ON)</button>
        <button class="btn btn-red full-width" onclick="api('/api/stop','POST')">Emergency Stop</button>
      </div>

      <div class="grid-3">
        <button id="b-cl" class="btn btn-blue" onclick="toggleClosedLoop()">PID: OFF</button>
        <button id="b-inv" class="btn btn-purple" onclick="toggleInvert()">DIR: NORM</button>
        <button id="b-mode" class="btn" onclick="toggleMode()">SILENT</button>
      </div>
    </div>

    <!-- KINESTHETIC HAND RECORD & REPLAY -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>🎥</span> Kinesthetic Hand Record & Replay</span>
        <span id="rec-status-tag" style="font-size:0.7rem;font-weight:500;color:var(--text-muted);">READY</span>
      </div>
      <p style="font-size:0.75rem;color:var(--text-muted);margin-bottom:0.8rem;line-height:1.4;">
        Teach-and-repeat: Click Record, rotate the motor shaft freely by hand for 5 seconds. Click Play to watch BoxDrive replicate the exact trajectory on the NEMA 17!
      </p>

      <div class="grid-2">
        <button id="b-rec" class="btn btn-red full-width" onclick="startRecord()">● Record 5s</button>
        <button id="b-play" class="btn btn-yellow full-width" onclick="playRecord()">▶ Play Replay</button>
      </div>

      <div class="progress-bar" id="p-bar"><div class="progress-inner" id="p-inner"></div></div>

      <div class="row" style="margin-top:0.8rem;">
        <span class="row-label">Speed</span>
        <select id="sel-speed">
          <option value="0.5">0.5x (Slow Smooth)</option>
          <option value="1.0" selected>1.0x (Original Speed)</option>
          <option value="1.5">1.5x (Fast 1.5x)</option>
          <option value="2.0">2.0x (Double Speed)</option>
        </select>
        <span id="s-count" style="font-size:0.75rem;color:var(--text-muted);font-weight:500;margin-left:auto;">0 samples</span>
      </div>
      <div style="margin-top:0.4rem;">
        <button class="btn full-width" style="font-size:0.75rem;padding:0.4rem;" onclick="stopRecord()">■ Stop Motion Replay</button>
      </div>
    </div>

    <!-- ANGLE POSITIONING & JOG -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>🎯</span> Angle Positioning</span>
      </div>

      <div style="margin-bottom:0.8rem;">
        <div style="font-size:0.68rem;color:var(--text-muted);font-weight:500;margin-bottom:4px;text-transform:uppercase;">Quick Presets</div>
        <div class="grid-3" style="margin-bottom:0.4rem;">
          <button class="btn" onclick="setAngle(0)">0°</button>
          <button class="btn" onclick="setAngle(45)">45°</button>
          <button class="btn" onclick="setAngle(90)">90°</button>
        </div>
        <div class="grid-3">
          <button class="btn" onclick="setAngle(180)">180°</button>
          <button class="btn" onclick="setAngle(270)">270°</button>
          <button class="btn" onclick="setAngle(360)">360°</button>
        </div>
      </div>

      <div style="margin-bottom:0.8rem;">
        <div style="font-size:0.68rem;color:var(--text-muted);font-weight:500;margin-bottom:4px;text-transform:uppercase;">Step Jog</div>
        <div class="grid-2">
          <button class="btn" onclick="jog(-45)">◀◀ -45°</button>
          <button class="btn" onclick="jog(45)">+45° ▶▶</button>
        </div>
        <div class="grid-2" style="margin-top:0.4rem;">
          <button class="btn" onclick="jog(-5)">◀ -5°</button>
          <button class="btn" onclick="jog(5)">+5° ▶</button>
        </div>
      </div>

      <div class="row">
        <span class="row-label">Target Angle</span>
        <input type="range" id="sl-angle" min="0" max="360" value="90" oninput="document.getElementById('in-angle').value=this.value">
        <input type="number" id="in-angle" min="0" max="360" value="90" oninput="document.getElementById('sl-angle').value=this.value">
        <button class="btn btn-yellow" onclick="setAngle(document.getElementById('in-angle').value)">GO</button>
      </div>

      <div class="row">
        <span class="row-label">Move Steps</span>
        <input type="number" id="in-steps" value="3200" step="200" style="flex:1;">
        <button class="btn" onclick="api('/api/move?steps='+document.getElementById('in-steps').value)">MOVE</button>
      </div>
    </div>

    <!-- DRIVER CONFIGURATION & CURRENT SLIDER -->
    <div class="card">
      <div class="card-header">
        <span class="card-title"><span>⚙️</span> Driver Configuration</span>
      </div>

      <div class="row">
        <span class="row-label">Current</span>
        <input type="range" id="sl-cur" min="100" max="2000" step="50" value="800" oninput="document.getElementById('in-cur').value=this.value">
        <input type="number" id="in-cur" min="100" max="2000" step="50" value="800" oninput="document.getElementById('sl-cur').value=this.value">
        <button class="btn" onclick="api('/api/current?ma='+document.getElementById('in-cur').value)">SET</button>
      </div>

      <div class="row">
        <span class="row-label">Speed</span>
        <input type="range" id="sl-spd" min="100" max="5000" step="100" value="1200" oninput="document.getElementById('in-spd').value=this.value">
        <input type="number" id="in-spd" min="100" max="5000" step="100" value="1200" oninput="document.getElementById('sl-spd').value=this.value">
        <button class="btn" onclick="api('/api/speed?sps='+document.getElementById('in-spd').value)">SET</button>
      </div>

      <div class="row">
        <span class="row-label">Microsteps</span>
        <select id="sel-ms" onchange="api('/api/microsteps?ms='+this.value)">
          <option value="1">Full Step (1/1)</option>
          <option value="2">Half Step (1/2)</option>
          <option value="4">1/4 Step</option>
          <option value="8">1/8 Step</option>
          <option value="16" selected>1/16 Step (Standard)</option>
          <option value="32">1/32 Step</option>
          <option value="64">1/64 Step</option>
          <option value="128">1/128 Step</option>
          <option value="256">1/256 Step (Silent)</option>
        </select>
      </div>
    </div>

  </div>

  <footer class="footer">
    BoxPox Technologies &bull; Precision Motion Control for NEMA 17 Stepper Motors<br>
    Visit official website & documentation at <a href="https://boxpox.in" target="_blank">boxpox.in</a> &bull; Support: <a href="mailto:support@boxpox.in">support@boxpox.in</a>
  </footer>

</div>

<script>
let state = { enabled: false, closed_loop: false, inverted: false, mode: 'stealth', angle: 0, target: 0, vel: 0, samples: 0, current: 800, recording: false, playing: false };
const angleHistory = new Array(50).fill(0);
const targetHistory = new Array(50).fill(0);
const velHistory = new Array(50).fill(0);

function api(url, method){
  return fetch(url, { method: method || 'GET' })
    .then(r => r.json())
    .catch(e => console.error(e));
}

function setAngle(ang){
  api('/api/move?angle=' + ang);
}

function jog(delta){
  let target = (state.angle + delta);
  while(target < 0) target += 360;
  while(target >= 360) target -= 360;
  setAngle(target.toFixed(1));
}

function togglePower(){
  api(state.enabled ? '/api/disable' : '/api/enable', 'POST').then(updateStatus);
}

function toggleClosedLoop(){
  api('/api/closedloop?enable=' + (state.closed_loop ? '0' : '1')).then(updateStatus);
}

function toggleInvert(){
  api('/api/invert?inv=' + (state.inverted ? '0' : '1')).then(updateStatus);
}

function toggleMode(){
  api('/api/mode?m=' + (state.mode === 'stealth' ? 'spread' : 'stealth')).then(updateStatus);
}

let recTimer = null;
let isRecUi = false;
let isPlayUi = false;

function startRecord(){
  api('/api/record?action=start').then(res => {
    isRecUi = true;
    const bar = document.getElementById('p-bar');
    const inner = document.getElementById('p-inner');
    const bRec = document.getElementById('b-rec');
    bar.style.display = 'block';
    inner.style.width = '0%';
    bRec.textContent = '● Recording...';
    bRec.className = 'btn btn-red full-width btn-active';
    document.getElementById('rec-status-tag').textContent = 'RECORDING';
    document.getElementById('rec-status-tag').style.color = 'var(--red)';
    
    let start = Date.now();
    if (recTimer) clearInterval(recTimer);
    recTimer = setInterval(() => {
      let elapsed = Date.now() - start;
      let p = (elapsed / 5000) * 100;
      if (p >= 100){
        p = 100;
        clearInterval(recTimer);
        setTimeout(() => {
          bar.style.display = 'none';
          bRec.textContent = '● Record 5s';
          bRec.className = 'btn btn-red full-width';
          isRecUi = false;
          updateStatus();
        }, 500);
      }
      inner.style.width = p + '%';
    }, 50);
  });
}

function playRecord(){
  const spd = document.getElementById('sel-speed').value;
  api('/api/record?action=play&speed=' + spd).then(res => {
    isPlayUi = true;
    const bar = document.getElementById('p-bar');
    const inner = document.getElementById('p-inner');
    const bPlay = document.getElementById('b-play');
    bar.style.display = 'block';
    inner.style.width = '0%';
    bPlay.textContent = '▶ Replaying...';
    bPlay.className = 'btn btn-yellow full-width btn-active';
    document.getElementById('rec-status-tag').textContent = 'REPLAYING';
    document.getElementById('rec-status-tag').style.color = 'var(--yellow-dark)';
    
    let dur = (state.samples > 0 ? (state.samples * 10) : 5000) / parseFloat(spd);
    let start = Date.now();
    if (recTimer) clearInterval(recTimer);
    recTimer = setInterval(() => {
      let p = ((Date.now() - start) / dur) * 100;
      if (p >= 100){
        p = 100;
        clearInterval(recTimer);
        setTimeout(() => {
          bar.style.display = 'none';
          bPlay.textContent = '▶ Play Replay';
          bPlay.className = 'btn btn-yellow full-width';
          isPlayUi = false;
          updateStatus();
        }, 400);
      }
      inner.style.width = p + '%';
    }, 50);
  });
}

function stopRecord(){
  if (recTimer) clearInterval(recTimer);
  document.getElementById('p-bar').style.display = 'none';
  document.getElementById('b-rec').textContent = '● Record 5s';
  document.getElementById('b-rec').className = 'btn btn-red full-width';
  document.getElementById('b-play').textContent = '▶ Play Replay';
  document.getElementById('b-play').className = 'btn btn-yellow full-width';
  isRecUi = false;
  isPlayUi = false;
  api('/api/record?action=stop').then(updateStatus);
}

function updateStatus(){
  api('/api/status').then(d => {
    if (!d) return;
    state = d;

    // Numerical readouts
    document.getElementById('d-angle').textContent = d.angle.toFixed(1);
    document.getElementById('d-target').textContent = d.target.toFixed(1) + '°';
    document.getElementById('d-vel').textContent = Math.abs(d.velocity).toFixed(1);
    document.getElementById('d-err').textContent = (d.error >= 0 ? '+' : '') + d.error.toFixed(1) + '°';
    document.getElementById('d-pos').textContent = d.position;
    if (d.current) document.getElementById('d-cur').textContent = d.current;

    // Magnet and diagnostics
    document.getElementById('d-mag').textContent = d.magnet ? '✓ OK' : '✗ NONE';
    document.getElementById('d-mag').style.color = d.magnet ? 'var(--green)' : 'var(--red)';
    document.getElementById('d-agc').textContent = d.agc;
    document.getElementById('d-temp').textContent = d.overtemp ? 'OVERTEMP' : 'COOL';
    document.getElementById('d-temp').style.color = d.overtemp ? 'var(--red)' : 'var(--green)';
    document.getElementById('s-count').textContent = d.samples + ' samples';

    // Buttons state
    const bPow = document.getElementById('b-power');
    if (d.enabled){
      bPow.textContent = 'Energized (ON)';
      bPow.className = 'btn btn-green full-width';
    } else {
      bPow.textContent = 'Disabled (FREE)';
      bPow.className = 'btn btn-red full-width';
    }

    const bCl = document.getElementById('b-cl');
    bCl.textContent = d.closed_loop ? 'PID: ON' : 'PID: OFF';
    bCl.className = d.closed_loop ? 'btn btn-blue btn-active' : 'btn btn-blue';
    document.getElementById('cl-badge').textContent = d.closed_loop ? 'CLOSED-LOOP PID' : 'OPEN-LOOP';
    document.getElementById('cl-badge').style.color = d.closed_loop ? 'var(--blue)' : 'var(--text-muted)';

    const bInv = document.getElementById('b-inv');
    bInv.textContent = d.inverted ? 'DIR: INVERT' : 'DIR: NORM';
    bInv.className = d.inverted ? 'btn btn-purple btn-active' : 'btn btn-purple';

    const bMode = document.getElementById('b-mode');
    bMode.textContent = (d.mode === 'stealth') ? 'SILENT' : 'TORQUE';
    bMode.className = (d.mode === 'stealth') ? 'btn btn-yellow' : 'btn btn-blue';

    // Update Recording tags
    if (!isRecUi && !isPlayUi) {
      const rTag = document.getElementById('rec-status-tag');
      if (d.recording) {
        rTag.textContent = 'RECORDING';
        rTag.style.color = 'var(--red)';
      } else if (d.playing) {
        rTag.textContent = 'REPLAYING';
        rTag.style.color = 'var(--yellow-dark)';
      } else {
        rTag.textContent = d.samples > 0 ? (d.samples + ' SAMPLES') : 'READY';
        rTag.style.color = 'var(--text-muted)';
      }
    }

    // Vector Dial Rotation Updates
    const deg = d.angle;
    const needle = document.getElementById('needle-group');
    if (needle) needle.setAttribute('transform', 'rotate(' + deg + ' 100 100)');

    const pip = document.getElementById('target-pip');
    if (pip) pip.setAttribute('transform', 'rotate(' + d.target + ' 100 100)');

    const arc = document.getElementById('gauge-arc');
    if (arc) {
      const offset = Math.max(0, 527.79 - (deg / 360) * 527.79);
      arc.style.strokeDashoffset = offset;
    }

    // Push into history buffers
    angleHistory.push(d.angle);
    angleHistory.shift();
    targetHistory.push(d.target);
    targetHistory.shift();
    velHistory.push(Math.abs(d.velocity));
    velHistory.shift();

    drawCharts();
  });
}

function initCanvas(id){
  const c = document.getElementById(id);
  const dpr = window.devicePixelRatio || 1;
  const rect = c.getBoundingClientRect();
  c.width = rect.width * dpr;
  c.height = rect.height * dpr;
  const ctx = c.getContext('2d');
  ctx.scale(dpr, dpr);
  return { canvas: c, ctx: ctx, width: rect.width, height: rect.height };
}

let cAngleObj = null;
let cVelObj = null;

function drawCharts(){
  if (!cAngleObj) cAngleObj = initCanvas('c-angle');
  if (!cVelObj) cVelObj = initCanvas('c-vel');

  // Chart 1: Angle (Yellow) vs Target (Blue)
  const ctx1 = cAngleObj.ctx;
  const w1 = cAngleObj.width;
  const h1 = cAngleObj.height;
  ctx1.clearRect(0, 0, w1, h1);

  ctx1.strokeStyle = '#f1f5f9';
  ctx1.lineWidth = 1;
  ctx1.beginPath();
  for (let y = 0; y <= h1; y += h1 / 4) { ctx1.moveTo(0, y); ctx1.lineTo(w1, y); }
  ctx1.stroke();

  ctx1.setLineDash([4, 4]);
  ctx1.strokeStyle = '#0284c7';
  ctx1.lineWidth = 1.5;
  ctx1.beginPath();
  for (let i = 0; i < targetHistory.length; i++){
    const x = (i / (targetHistory.length - 1)) * w1;
    const y = h1 - (targetHistory[i] / 360) * h1;
    if (i === 0) ctx1.moveTo(x, y); else ctx1.lineTo(x, y);
  }
  ctx1.stroke();
  ctx1.setLineDash([]);

  ctx1.strokeStyle = '#eab308';
  ctx1.lineWidth = 2.2;
  ctx1.beginPath();
  for (let i = 0; i < angleHistory.length; i++){
    const x = (i / (angleHistory.length - 1)) * w1;
    const y = h1 - (angleHistory[i] / 360) * h1;
    if (i === 0) ctx1.moveTo(x, y); else ctx1.lineTo(x, y);
  }
  ctx1.stroke();

  // Chart 2: Velocity (Green)
  const ctx2 = cVelObj.ctx;
  const w2 = cVelObj.width;
  const h2 = cVelObj.height;
  ctx2.clearRect(0, 0, w2, h2);

  ctx2.strokeStyle = '#f1f5f9';
  ctx2.lineWidth = 1;
  ctx2.beginPath();
  for (let y = 0; y <= h2; y += h2 / 3) { ctx2.moveTo(0, y); ctx2.lineTo(w2, y); }
  ctx2.stroke();

  let maxVel = 200;
  for (let v of velHistory) if (v > maxVel) maxVel = v;

  ctx2.strokeStyle = '#16a34a';
  ctx2.lineWidth = 2.2;
  ctx2.beginPath();
  for (let i = 0; i < velHistory.length; i++){
    const x = (i / (velHistory.length - 1)) * w2;
    const y = h2 - (velHistory[i] / maxVel) * (h2 - 10) - 5;
    if (i === 0) ctx2.moveTo(x, y); else ctx2.lineTo(x, y);
  }
  ctx2.stroke();
}

window.addEventListener('resize', () => { cAngleObj = null; cVelObj = null; });
setInterval(updateStatus, 300);
updateStatus();
</script>

</body>
</html>
)rawliteral");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SPEED PROFILES
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::trapezoidalMove(int32_t steps, float maxSpeed, float accel) {
    if (!_motorEnabled) return;
    _isMoving = true;

    BoxDriveDir dir = (steps >= 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
    _setDirection(dir);
    int32_t absSteps = abs(steps);

    float currentSpeed = 0;
    uint32_t accelSteps = (uint32_t)(maxSpeed * maxSpeed / (2.0f * accel));
    if (accelSteps > (uint32_t)(absSteps / 2)) accelSteps = absSteps / 2;
    uint32_t decelStart = absSteps - accelSteps;

    for (int32_t i = 0; i < absSteps; i++) {
        if ((uint32_t)i < accelSteps) {
            currentSpeed = sqrt(2.0f * accel * (i + 1));
            if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
        } else if ((uint32_t)i >= decelStart) {
            currentSpeed = sqrt(2.0f * accel * (absSteps - i));
            if (currentSpeed < 50.0f) currentSpeed = 50.0f;
        } else {
            currentSpeed = maxSpeed;
        }

        uint32_t delayUs = (uint32_t)(1000000.0f / currentSpeed);
        _stepPulse();
        delayMicroseconds(delayUs);
        _currentPosition += (steps > 0) ? 1 : -1;
        yield();
    }

    _isMoving = false;
}

void BoxDrive::sCurveMove(int32_t steps, float maxSpeed, float accel, float jerk) {
    if (!_motorEnabled) return;
    _isMoving = true;

    BoxDriveDir dir = (steps >= 0) ? BoxDriveDir::CW : BoxDriveDir::CCW;
    _setDirection(dir);
    int32_t absSteps = abs(steps);

    // S-curve: v(t) = maxSpeed * (1 - cos(π*t/T)) / 2
    float totalTime = (float)absSteps / (maxSpeed * 0.5f);

    for (int32_t i = 0; i < absSteps; i++) {
        float progress = (float)i / (float)absSteps;
        float speedFactor;

        if (progress < 0.25f) {
            // Acceleration ramp-up (jerk increasing)
            speedFactor = 2.0f * progress * progress;
        } else if (progress < 0.5f) {
            // Acceleration ramp-down
            float t = progress - 0.25f;
            speedFactor = 0.125f + 0.5f * (progress - 0.125f);
        } else if (progress < 0.75f) {
            // Deceleration ramp-up
            speedFactor = 1.0f - 2.0f * (1.0f - progress) * (1.0f - progress);
        } else {
            // Deceleration ramp-down
            speedFactor = 2.0f * (1.0f - progress) * (1.0f - progress);
        }

        float currentSpeed = maxSpeed * max(speedFactor, 0.02f);
        uint32_t delayUs = (uint32_t)(1000000.0f / currentSpeed);
        _stepPulse();
        delayMicroseconds(delayUs);
        _currentPosition += (steps > 0) ? 1 : -1;
        yield();
    }

    _isMoving = false;
    (void)jerk;   // Used implicitly in curve shape
    (void)totalTime;
}

void BoxDrive::sinusoidalMotion(float amplitudeDeg, float frequencyHz, uint32_t cycles) {
    if (!_motorEnabled) return;

    int32_t stepsPerRev = BD_STEPS_PER_REV * _microsteps;
    int32_t amplitude = (int32_t)(amplitudeDeg / 360.0f * stepsPerRev);
    uint32_t periodUs = (uint32_t)(1000000.0f / frequencyHz);

    _isMoving = true;
    uint32_t startTime = micros();
    uint32_t cycleCount = 0;
    int32_t lastStep = 0;

    while (_isMoving) {
        uint32_t elapsed = micros() - startTime;
        float phase = (float)(elapsed % periodUs) / (float)periodUs;
        int32_t targetStep = (int32_t)(amplitude * sin(phase * 2.0f * PI));

        int32_t delta = targetStep - lastStep;
        if (delta != 0) {
            _setDirection(delta > 0 ? BoxDriveDir::CW : BoxDriveDir::CCW);
            for (int32_t i = 0; i < abs(delta); i++) {
                _stepPulse();
                delayMicroseconds(50);
            }
            _currentPosition += delta;
            lastStep = targetStep;
        }

        // Check cycle count
        if (elapsed / periodUs > cycleCount) {
            cycleCount = elapsed / periodUs;
            if (cycles > 0 && cycleCount >= cycles) break;
        }

        delayMicroseconds(100);
        yield();
    }

    _isMoving = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HOMING
// ═══════════════════════════════════════════════════════════════════════════════

bool BoxDrive::homeStallGuard(BoxDriveDir dir, float speed, uint8_t threshold) {
    Serial.println(F("[HOME] StallGuard homing started..."));

    enableStallGuard(threshold);
    enableMotor();

    _setDirection(dir);
    uint32_t stepDelay = (uint32_t)(1000000.0f / speed);
    uint32_t timeout = millis() + 30000; // 30 sec timeout

    // Wait for stall
    while (!isStalled()) {
        _stepPulse();
        delayMicroseconds(stepDelay);

        if (millis() > timeout) {
            Serial.println(F("[HOME] Timeout!"));
            disableStallGuard();
            return false;
        }
        yield();
    }

    stop();
    setPosition(0);
    encoder.resetMultiTurn();
    disableStallGuard();

    Serial.println(F("[HOME] Home position found."));
    return true;
}

bool BoxDrive::homeEncoder(float speed) {
    Serial.println(F("[HOME] Encoder homing started..."));

    enableMotor();
    _setDirection(BoxDriveDir::CCW);
    uint32_t stepDelay = (uint32_t)(1000000.0f / speed);
    uint32_t timeout = millis() + 30000;

    float startAngle = encoder.readAngleDeg();
    bool crossedZero = false;

    while (!crossedZero) {
        _stepPulse();
        delayMicroseconds(stepDelay);

        float angle = encoder.readAngleDeg();
        // Detect zero crossing (going from high angle to low angle)
        if (startAngle > 180.0f && angle < 10.0f) {
            crossedZero = true;
        }
        startAngle = angle;

        if (millis() > timeout) {
            Serial.println(F("[HOME] Timeout!"));
            return false;
        }
        yield();
    }

    stop();
    setPosition(0);
    encoder.resetMultiTurn();

    Serial.println(F("[HOME] Encoder zero found."));
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  STATUS & DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════════

BoxDriveStatus BoxDrive::getStatus() {
    BoxDriveStatus status;
    uint32_t drvStatus = readDriverStatus();

    status.motorEnabled     = _motorEnabled;
    status.drvStatus        = drvStatus;
    status.stallDetected    = (drvStatus & (1 << 24)) != 0;  // stallGuard
    status.overTemp         = (drvStatus & (1 << 25)) != 0;  // ot
    status.overTempShutdown = (drvStatus & (1 << 26)) != 0;  // otpw
    status.shortToGnd       = ((drvStatus & (1 << 12)) | (drvStatus & (1 << 13))) != 0;
    status.openLoad         = ((drvStatus & (1 << 10)) | (drvStatus & (1 << 11))) != 0;
    status.csActual         = (drvStatus >> 16) & 0x1F;
    status.sgResult         = readStallGuardValue();

    return status;
}

uint32_t BoxDrive::readDriverStatus() {
    return readRegister(TMC2209_DRV_STATUS);
}

bool BoxDrive::isOverTemp() {
    uint32_t status = readDriverStatus();
    return (status & (1 << 25)) != 0;
}

void BoxDrive::printDiagnostics() {
    BoxDriveStatus s = getStatus();

    Serial.println(F("╔══════════════════════════════════════════╗"));
    Serial.println(F("║           BOXDRIVE DIAGNOSTICS           ║"));
    Serial.println(F("╠══════════════════════════════════════════╣"));

    Serial.print(F("║ Motor:        "));
    Serial.println(s.motorEnabled ? F("ENABLED") : F("DISABLED"));

    Serial.print(F("║ Mode:         "));
    Serial.println(_mode == BoxDriveMode::STEALTH_CHOP ? F("StealthChop") : F("SpreadCycle"));

    Serial.print(F("║ Current:      "));
    Serial.print(_runCurrentMA);
    Serial.print(F(" / "));
    Serial.print(_holdCurrentMA);
    Serial.println(F(" mA (run/hold)"));

    Serial.print(F("║ Microsteps:   1/"));
    Serial.println(_microsteps);

    Serial.print(F("║ Position:     "));
    Serial.print(_currentPosition);
    Serial.println(F(" steps"));

    Serial.print(F("║ Speed:        "));
    Serial.print(_speed, 0);
    Serial.println(F(" steps/sec"));

    Serial.print(F("║ CS Actual:    "));
    Serial.println(s.csActual);

    Serial.print(F("║ StallGuard:   "));
    Serial.println(s.sgResult);

    Serial.print(F("║ Stall:        "));
    Serial.println(s.stallDetected ? F("YES ⚠") : F("No"));

    Serial.print(F("║ Over-temp:    "));
    Serial.println(s.overTemp ? F("YES ⚠") : F("No"));

    Serial.print(F("║ Short-GND:    "));
    Serial.println(s.shortToGnd ? F("YES ⚠") : F("No"));

    Serial.print(F("║ Open-load:    "));
    Serial.println(s.openLoad ? F("YES ⚠") : F("No"));

    Serial.print(F("║ DRV_STATUS:   0x"));
    Serial.println(s.drvStatus, HEX);

    Serial.println(F("╠══════════════════════════════════════════╣"));

    printEncoderStatus();

    Serial.println(F("╚══════════════════════════════════════════╝"));
}

void BoxDrive::printEncoderStatus() {
    float angle = encoder.readAngleDeg();
    uint16_t raw = encoder.readRawAngle();
    uint8_t agc = encoder.readAGC();
    uint16_t mag = encoder.readMagnitude();
    uint8_t status = encoder.readStatus();

    Serial.print(F("║ Encoder angle: "));
    Serial.print(angle, 2);
    Serial.println(F("°"));

    Serial.print(F("║ Raw value:     "));
    Serial.println(raw);

    Serial.print(F("║ Multi-turn:    "));
    Serial.print(encoder.readMultiTurnDeg(), 2);
    Serial.println(F("°"));

    Serial.print(F("║ Velocity:      "));
    Serial.print(encoder.getVelocityDPS(), 1);
    Serial.println(F(" °/s"));

    Serial.print(F("║ AGC:           "));
    Serial.println(agc);

    Serial.print(F("║ Magnitude:     "));
    Serial.println(mag);

    Serial.print(F("║ Magnet:        "));
    if (encoder.magnetDetected()) Serial.println(F("DETECTED ✓"));
    else if (encoder.magnetTooWeak()) Serial.println(F("TOO WEAK ⚠"));
    else if (encoder.magnetTooStrong()) Serial.println(F("TOO STRONG ⚠"));
    else Serial.println(F("NOT DETECTED ✗"));

    Serial.print(F("║ Status reg:    0x"));
    Serial.println(status, HEX);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LED CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void BoxDrive::setLED(BoxDriveLED pattern) {
    _ledPattern = pattern;
    _ledLastToggle = millis();

    switch (pattern) {
        case BoxDriveLED::OFF:
            digitalWrite(BD_PIN_LED, LOW);
            _ledState = false;
            break;
        case BoxDriveLED::ON:
            analogWrite(BD_PIN_LED, _ledBrightness);
            _ledState = true;
            break;
        default:
            break;
    }
}

void BoxDrive::updateLED() {
    uint32_t now = millis();

    switch (_ledPattern) {
        case BoxDriveLED::BLINK_SLOW:
            if (now - _ledLastToggle >= 500) {
                _ledState = !_ledState;
                digitalWrite(BD_PIN_LED, _ledState ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;

        case BoxDriveLED::BLINK_FAST:
            if (now - _ledLastToggle >= 100) {
                _ledState = !_ledState;
                digitalWrite(BD_PIN_LED, _ledState ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;

        case BoxDriveLED::PULSE: {
            // Smooth breathing effect using sine wave
            float phase = (float)(now % 3000) / 3000.0f;
            uint8_t brightness = (uint8_t)(127.5f * (1.0f + sin(phase * 2.0f * PI)));
            analogWrite(BD_PIN_LED, brightness);
            break;
        }

        case BoxDriveLED::ERROR_FLASH: {
            // Triple-flash pattern: ON-OFF-ON-OFF-ON-OFF --- pause ---
            uint32_t cycle = (now / 100) % 12;
            bool on = (cycle == 0 || cycle == 2 || cycle == 4);
            digitalWrite(BD_PIN_LED, on ? HIGH : LOW);
            break;
        }

        default:
            break;
    }
}

void BoxDrive::setLEDBrightness(uint8_t brightness) {
    _ledBrightness = brightness;
    if (_ledPattern == BoxDriveLED::ON) {
        analogWrite(BD_PIN_LED, _ledBrightness);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TMC2209 UART REGISTER ACCESS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * TMC2209 UART Protocol:
 * Write datagram:  [SYNC] [SLAVE_ADDR] [REG|0x80] [DATA3] [DATA2] [DATA1] [DATA0] [CRC]
 * Read request:    [SYNC] [SLAVE_ADDR] [REG]       [CRC]
 * Read response:   [SYNC] [0xFF]       [REG]       [DATA3] [DATA2] [DATA1] [DATA0] [CRC]
 */

uint8_t BoxDrive::_calcCRC(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

void BoxDrive::writeRegister(uint8_t reg, uint32_t value) {
    if (!_tmcSerial) return;

    uint8_t datagram[8];
    datagram[0] = TMC2209_SYNC;
    datagram[1] = TMC2209_SLAVE_ADDR;
    datagram[2] = reg | 0x80; // Write flag
    datagram[3] = (value >> 24) & 0xFF;
    datagram[4] = (value >> 16) & 0xFF;
    datagram[5] = (value >> 8)  & 0xFF;
    datagram[6] = value & 0xFF;
    datagram[7] = _calcCRC(datagram, 7);

    // Flush any pending data
    while (_tmcSerial->available()) _tmcSerial->read();

    for (uint8_t i = 0; i < 8; i++) {
        _tmcSerial->write(datagram[i]);
    }
    _tmcSerial->flush();
    delay(2); // Give driver time to process
}

uint32_t BoxDrive::readRegister(uint8_t reg) {
    if (!_tmcSerial) return 0;

    // Send read request
    uint8_t request[4];
    request[0] = TMC2209_SYNC;
    request[1] = TMC2209_SLAVE_ADDR;
    request[2] = reg & 0x7F; // Read flag (bit 7 = 0)
    request[3] = _calcCRC(request, 3);

    while (_tmcSerial->available()) _tmcSerial->read();

    for (uint8_t i = 0; i < 4; i++) {
        _tmcSerial->write(request[i]);
    }
    _tmcSerial->flush();

    // Wait for response (8 bytes)
    delay(5);

    // Skip echo of our request (4 bytes)
    for (uint8_t i = 0; i < 4; i++) {
        if (_tmcSerial->available()) _tmcSerial->read();
    }

    // Read response (8 bytes: sync + addr + reg + 4 data + crc)
    uint8_t response[8];
    uint32_t timeout = millis() + 50;
    uint8_t idx = 0;

    while (idx < 8 && millis() < timeout) {
        if (_tmcSerial->available()) {
            response[idx++] = _tmcSerial->read();
        }
    }

    if (idx < 8) return 0; // Timeout

    // Verify CRC
    uint8_t crc = _calcCRC(response, 7);
    if (crc != response[7]) return 0;

    // Extract 32-bit value
    uint32_t value = 0;
    value |= ((uint32_t)response[3] << 24);
    value |= ((uint32_t)response[4] << 16);
    value |= ((uint32_t)response[5] << 8);
    value |= ((uint32_t)response[6]);

    return value;
}
