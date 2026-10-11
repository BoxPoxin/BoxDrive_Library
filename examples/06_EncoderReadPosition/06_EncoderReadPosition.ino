/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 06: Encoder Read Position                                         ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates all AS5600 magnetic encoder capabilities:                    ║
 * ║   • Raw 12-bit angle reading (0–4095)                                      ║
 * ║   • Angle in degrees (0°–360°) and radians (0–2π)                          ║
 * ║   • Multi-turn angle tracking (beyond 360°)                                ║
 * ║   • Angular velocity calculation (°/sec)                                   ║
 * ║   • Magnet status and diagnostics (AGC, magnitude)                         ║
 * ║                                                                            ║
 * ║  THE AS5600 ENCODER:                                                       ║
 * ║   • 12-bit resolution = 4096 positions per revolution                      ║
 * ║   • 0.0879° per LSB                                                        ║
 * ║   • Communicates via I2C (SDA=GPIO23, SCL=GPIO22)                          ║
 * ║   • Needs a diametrically magnetized magnet on the shaft                   ║
 * ║   • Maximum detection distance: ~3mm                                       ║
 * ║                                                                            ║
 * ║  TIP: If magnet is not detected, check the air gap between the             ║
 * ║  magnet and the AS5600 chip on the back of the board.                      ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    motor.begin();

    // We don't need to enable the motor for encoder reading
    // The encoder works independently via I2C

    Serial.println("AS5600 Encoder Demo");
    Serial.println("Turn the motor shaft by hand and watch the readings.");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();

    // Check magnet status
    if (motor.encoder.magnetDetected()) {
        Serial.println("✓ Magnet detected — readings are valid.");
    } else {
        Serial.println("✗ No magnet detected!");
        Serial.println("  Please mount a diametrically magnetized magnet");
        Serial.println("  on the motor shaft, directly above the AS5600.");
    }

    Serial.println();
    Serial.println("AGC  | Raw   | Degrees | Radians | MultiTurn | Velocity");
    Serial.println("-----|-------|---------|---------|-----------|----------");
}

void loop() {
    // Read all encoder values
    uint16_t raw    = motor.encoder.readRawAngle();
    float    deg    = motor.encoder.readAngleDeg();
    float    rad    = motor.encoder.readAngleRad();
    float    multi  = motor.encoder.readMultiTurnDeg();
    float    vel    = motor.encoder.getVelocityDPS();
    uint8_t  agc    = motor.encoder.readAGC();
    uint16_t mag    = motor.encoder.readMagnitude();

    // Print formatted output
    Serial.print(agc);
    Serial.print("\t| ");
    Serial.print(raw);
    Serial.print("\t| ");
    Serial.print(deg, 2);
    Serial.print("°\t| ");
    Serial.print(rad, 4);
    Serial.print("\t| ");
    Serial.print(multi, 2);
    Serial.print("°\t| ");
    Serial.print(vel, 1);
    Serial.println(" °/s");

    // Periodically print magnet diagnostics
    static uint32_t lastDiag = 0;
    if (millis() - lastDiag > 5000) {
        lastDiag = millis();
        Serial.println();
        Serial.println("── Magnet Diagnostics ──");
        Serial.print("  Status: ");
        if (motor.encoder.magnetDetected()) Serial.println("DETECTED ✓");
        else Serial.println("NOT DETECTED ✗");
        Serial.print("  Too strong: ");
        Serial.println(motor.encoder.magnetTooStrong() ? "YES ⚠" : "No");
        Serial.print("  Too weak: ");
        Serial.println(motor.encoder.magnetTooWeak() ? "YES ⚠" : "No");
        Serial.print("  AGC value: ");
        Serial.print(agc);
        Serial.println(" (ideal: 30–200)");
        Serial.print("  Magnitude: ");
        Serial.println(mag);
        Serial.println("────────────────────────");
        Serial.println();
    }

    delay(50); // ~20 Hz update rate
}
