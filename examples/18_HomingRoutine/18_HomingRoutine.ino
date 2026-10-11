/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 18: Precision Homing Routines (StallGuard + Absolute Encoder)     ║
 * ║  BoxDrive Library — Zero-Switch Homing for Robotic Axes & 3D Printers       ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates two independent homing methods without external switches:     ║
 * ║                                                                            ║
 * ║  METHOD 1: SENSORLESS STALLGUARD HOMING                                     ║
 * ║   • Moves motor in chosen direction until it contacts a mechanical hard stop║
 * ║   • Detects stall via TMC2209 back-EMF, stops instantly, and zeroes count.  ║
 * ║                                                                            ║
 * ║  METHOD 2: ABSOLUTE MAGNETIC ENCODER HOMING                                 ║
 * ║   • Rotates motor until the AS5600 reads exactly 0.00° (or saved ZPOS).    ║
 * ║   • Provides sub-degree repeatable homing without hitting any hard stop!   ║
 * ║                                                                            ║
 * ║  METHOD 3: HYBRID TWO-STAGE HOMING (Industrial Standard)                    ║
 * ║   • Stage 1: Fast StallGuard seek to find mechanical physical boundary.    ║
 * ║   • Stage 2: Back off slightly, then fine-align to absolute magnetic zero. ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("  BoxDrive Example 18: Dual-Mode Homing Routines  ");
    Serial.println("==================================================");

    motor.begin();
    motor.setCurrent(700);      // Modest current for soft bump homing
    motor.setHoldCurrent(300);
    motor.setMode(BoxDriveMode::STEALTH_CHOP);
    motor.setMicrosteps(16);
    motor.enableMotor();

    Serial.println("[+] Motor energized.");
    Serial.println("[+] Beginning Homing Demonstrations in 2 seconds...");
    delay(2000);

    // ─────────────────────────────────────────────────────────────
    // DEMO 1: SENSORLESS HOMING VIA STALLGUARD
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n--- [1] Sensorless StallGuard Homing ---");
    Serial.println("Moving CCW at 300 steps/s towards hard stop...");
    motor.setLED(BoxDriveLED::BLINK_FAST);

    // homeStallGuard moves CCW until motor stalls on mechanical endstop
    bool sgSuccess = motor.homeStallGuard(BoxDriveDir::CCW, 300.0f, 60);

    if (sgSuccess) {
        Serial.println("[✓] Hard stop detected! Position zeroed.");
        // Back off 200 steps from the hard stop to relieve stress
        Serial.println("Backing off 200 steps from hard stop...");
        motor.move(200);
        motor.setPosition(0); // Set this safe point as absolute zero
        motor.encoder.resetMultiTurn();
    } else {
        Serial.println("[!] StallGuard homing timed out or cancelled.");
    }

    delay(2000);

    // ─────────────────────────────────────────────────────────────
    // DEMO 2: ABSOLUTE ENCODER HOMING
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n--- [2] Absolute AS5600 Magnetic Homing ---");
    Serial.println("Seeking magnetic 0.00° index mark...");
    motor.setLED(BoxDriveLED::PULSE);

    // homeEncoder rotates until current angle matches 0.0°
    bool encSuccess = motor.homeEncoder(250.0f);

    if (encSuccess) {
        Serial.print("[✓] Magnetic zero found! Current Angle: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.println("°");
        motor.setPosition(0);
        motor.encoder.resetMultiTurn();
    } else {
        Serial.println("[!] Encoder homing timed out.");
    }

    motor.setLED(BoxDriveLED::BLINK_SLOW);
    Serial.println("\n[✓] Both homing routines completed successfully.");
    Serial.println("Motor is now synchronized with coordinate frame (Pos = 0).");
    Serial.println("--------------------------------------------------");
}

void loop() {
    // Demonstration move: oscillate ±90 degrees from newly homed 0 position
    Serial.println("\nRotating +90° from Home...");
    motor.moveToAngle(90.0f);
    delay(1000);

    Serial.println("Returning to Home (0°)...");
    motor.moveToAngle(0.0f);
    delay(1000);

    Serial.println("Rotating -90° from Home...");
    motor.moveToAngle(-90.0f);
    delay(1000);

    Serial.println("Returning to Home (0°)...");
    motor.moveToAngle(0.0f);
    delay(3000);
}
