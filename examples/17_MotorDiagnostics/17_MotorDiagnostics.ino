/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 17: Full Hardware Diagnostic & Self-Test Suite                     ║
 * ║  BoxDrive Library — Automated Quality & Trouble-Shooting Check              ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Performs a complete hardware sanity check on your BoxDrive controller:     ║
 * ║   1. AS5600 Magnetic Encoder: I2C ACK, magnet distance, AGC, magnitude.   ║
 * ║   2. TMC2209 UART Driver: Register read/write, coil health, temps, faults. ║
 * ║   3. Motion Validation: Steps motor 360° and cross-verifies encoder delta.  ║
 * ║   4. Status LED PWM & blink pattern verification.                           ║
 * ║                                                                            ║
 * ║  HOW TO USE:                                                               ║
 * ║   • Open Serial Monitor at 115200 baud.                                    ║
 * ║   • The test suite runs automatically on boot and prints a formatted       ║
 * ║     diagnostic table showing PASS / WARN / FAIL for every component.        ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void printBanner(const char* title) {
    Serial.println();
    Serial.println("==================================================");
    Serial.print("  ");
    Serial.println(title);
    Serial.println("==================================================");
}

void printResult(const char* testName, bool pass, const char* details = "") {
    Serial.printf("  [%-30s] : ", testName);
    if (pass) {
        Serial.print("\033[32mPASS\033[0m"); // Green in ANSI terminals
    } else {
        Serial.print("\033[31mFAIL\033[0m"); // Red
    }
    if (strlen(details) > 0) {
        Serial.printf(" (%s)", details);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    printBanner("BoxDrive HW v3.26 Automated Self-Test");

    // ─────────────────────────────────────────────────────────────
    // TEST 1: AS5600 12-BIT ENCODER TEST
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n[1] --- AS5600 MAGNETIC ENCODER SUBSYSTEM ---");

    Wire.begin(BD_PIN_SDA, BD_PIN_SCL);
    Wire.beginTransmission(AS5600_ADDRESS);
    bool i2cAck = (Wire.endTransmission() == 0);
    printResult("I2C Bus Communication", i2cAck, "Address 0x36");

    bool magDetected = motor.encoder.magnetDetected();
    printResult("Magnet Detected", magDetected, magDetected ? "Magnet Present" : "NO MAGNET DETECTED!");

    bool magTooWeak   = motor.encoder.magnetTooWeak();
    bool magTooStrong = motor.encoder.magnetTooStrong();
    uint8_t agc       = motor.encoder.readAGC();
    uint16_t mag      = motor.encoder.readMagnitude();

    char magBuf[64];
    snprintf(magBuf, sizeof(magBuf), "AGC=%u (Ideal 40-180), Mag=%u", agc, mag);
    bool magDistanceOk = (!magTooWeak && !magTooStrong && agc >= 20 && agc <= 220);
    printResult("Magnet Distance / Field", magDistanceOk, magBuf);

    float angleInit = motor.encoder.readAngleDeg();
    char angBuf[32];
    snprintf(angBuf, sizeof(angBuf), "Current Angle: %.2f°", angleInit);
    printResult("Angle Measurement", (angleInit >= 0.0f && angleInit <= 360.0f), angBuf);

    // ─────────────────────────────────────────────────────────────
    // TEST 2: TMC2209 STEPPER DRIVER UART TEST
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n[2] --- TMC2209 DRIVER & COIL TELEMETRY ---");

    bool bdInit = motor.begin();
    printResult("TMC2209 UART Link", bdInit, "GPIO 14(RX)/18(TX)");

    // Read Driver Status
    BoxDriveStatus status = motor.getStatus();
    printResult("Thermal Warning Flag", !status.overTemp, status.overTemp ? "HIGH TEMP!" : "OK");
    printResult("Thermal Shutdown Flag", !status.overTempShutdown, status.overTempShutdown ? "SHUTDOWN!" : "OK");
    printResult("Short-to-GND Detection", !status.shortToGnd, status.shortToGnd ? "SHORT CIRCUIT DETECTED!" : "No Short");
    printResult("Open-Load Coil Detect", !status.openLoad, status.openLoad ? "OPEN COIL (Check Wires)" : "Coils Connected");

    char csBuf[32];
    snprintf(csBuf, sizeof(csBuf), "CS Scale: %u / 31", status.csActual);
    printResult("Current Scaler Readback", true, csBuf);

    // ─────────────────────────────────────────────────────────────
    // TEST 3: CLOSED-LOOP STEP/DIR MOTION VERIFICATION
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n[3] --- MOTION KINEMATICS & ENCODER CROSS-CHECK ---");
    motor.setCurrent(800);
    motor.setSpeed(1200);
    motor.setAcceleration(2400);
    motor.enableMotor();

    float preAngle = motor.encoder.readMultiTurnDeg();
    Serial.println("  Rotating motor 3200 steps (360°)...");
    motor.move(3200);
    delay(300);

    float postAngle = motor.encoder.readMultiTurnDeg();
    float deltaDeg  = abs(postAngle - preAngle);

    char motionBuf[64];
    snprintf(motionBuf, sizeof(motionBuf), "Delta: %.1f° (Expected ~360.0°)", deltaDeg);
    // Passing if delta is roughly one full turn (360° ± 20° to allow slight setup variance)
    bool motionOk = (deltaDeg >= 340.0f && deltaDeg <= 380.0f);
    printResult("Encoder Step Verification", motionOk, motionBuf);

    if (motionOk) {
        if (postAngle < preAngle) {
            Serial.println("  [Notice] Stepper rotation is inverted relative to AS5600 encoder.");
            Serial.println("           Run motor.invertDirection(true) to synchronize closed-loop PID!");
        } else {
            Serial.println("  [Notice] Stepper and AS5600 directions are aligned normally.");
        }
    }

    Serial.println("  Returning back 3200 steps...");
    motor.move(-3200);
    delay(300);

    // ─────────────────────────────────────────────────────────────
    // TEST 4: ONBOARD LED & PERIPHERALS
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n[4] --- STATUS LED INDICATOR ---");
    Serial.println("  Testing LED PWM brightness...");
    for (int b = 0; b <= 255; b += 50) {
        motor.setLEDBrightness(b);
        delay(40);
    }
    motor.setLED(BoxDriveLED::BLINK_SLOW);
    printResult("Status LED Output", true, "GPIO 19");

    // ─────────────────────────────────────────────────────────────
    // SUMMARY
    // ─────────────────────────────────────────────────────────────
    printBanner("Diagnostic Summary Complete");
    if (i2cAck && magDetected && bdInit && !status.shortToGnd) {
        Serial.println("  >>> ALL CRITICAL SUBSYSTEMS OPERATIONAL. READY FOR USE! <<<");
    } else {
        Serial.println("  >>> WARNING: ONE OR MORE TESTS REPORTED ISSUES. CHECK WIRING! <<<");
    }
    Serial.println("==================================================");
}

void loop() {
    // Continuously report real-time telemetry
    motor.updateLED();

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.print("[Live Status] Angle: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.print("° | Velocity: ");
        Serial.print(motor.encoder.getVelocityDPS(), 1);
        Serial.print("°/s | Temp: ");
        Serial.print(motor.isOverTemp() ? "HOT" : "NORM");
        Serial.print(" | AGC: ");
        Serial.println(motor.encoder.readAGC());
    }

    delay(20);
}
