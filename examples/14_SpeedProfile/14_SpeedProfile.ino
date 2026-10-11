/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 14: Advanced Speed Profiles (Trapezoidal vs S-Curve)              ║
 * ║  BoxDrive Library — Vibration-Free Motion Planning                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates smooth acceleration profiles for high-speed precision motion: ║
 * ║   1. Trapezoidal Profile: Constant acceleration and linear deceleration.    ║
 * ║   2. S-Curve Profile    : Jerk-limited 3rd-order polynomial ramp.           ║
 * ║                                                                            ║
 * ║  WHY S-CURVE MATTERS:                                                      ║
 * ║   • Eliminates instantaneous acceleration jumps (infinite jerk).           ║
 * ║   • Drastically reduces mechanical ringing, resonances, and frame shake.   ║
 * ║   • Prevents liquid sloshing in lab automation & ringing in 3D printers.   ║
 * ║                                                                            ║
 * ║  WIRING:                                                                   ║
 * ║   • Standard BoxDrive board mounted to NEMA 17 motor with XT30 power.      ║
 * ║   • Connect USB-C to view Serial Monitor performance telemetry.           ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("  BoxDrive Example 14: Motion Profiles Comparison ");
    Serial.println("==================================================");

    motor.begin();
    motor.setCurrent(900);
    motor.setHoldCurrent(300);
    motor.setMode(BoxDriveMode::STEALTH_CHOP);
    motor.setMicrosteps(16);
    motor.enableMotor();

    Serial.println("[+] Motor configured and energized.");
    Serial.println("[+] Watch and listen to the difference between Trapezoidal and S-Curve profiles.");
    delay(1000);
}

void loop() {
    const int32_t MOVE_STEPS   = 6400;     // 2 full revolutions (3200 steps/rev @ 16 microsteps)
    const float   CRUISE_SPEED = 2500.0f;  // Peak steps/second
    const float   ACCEL        = 5000.0f;  // Acceleration steps/sec²
    const float   JERK         = 20000.0f; // Jerk steps/sec³

    // ─────────────────────────────────────────────────────────────
    // PROFILE 1: TRAPEZOIDAL PROFILE
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n--- [Profile 1] TRAPEZOIDAL PROFILE ---");
    Serial.print("Parameters: Steps = ");
    Serial.print(MOVE_STEPS);
    Serial.print(" | MaxSpeed = ");
    Serial.print(CRUISE_SPEED);
    Serial.print(" steps/s | Accel = ");
    Serial.print(ACCEL);
    Serial.println(" steps/s²");

    motor.setLED(BoxDriveLED::BLINK_FAST);
    uint32_t tStart = millis();

    // Execute trapezoidal trajectory move
    motor.trapezoidalMove(MOVE_STEPS, CRUISE_SPEED, ACCEL);

    uint32_t durationTrapezoid = millis() - tStart;
    Serial.print("[✓] Trapezoidal move finished in ");
    Serial.print(durationTrapezoid);
    Serial.print(" ms | Final Encoder Angle: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");

    motor.setLED(BoxDriveLED::BLINK_SLOW);
    delay(1500);

    // Return to 0 using Trapezoidal
    Serial.println("Returning to base position...");
    motor.trapezoidalMove(-MOVE_STEPS, CRUISE_SPEED, ACCEL);
    delay(1500);

    // ─────────────────────────────────────────────────────────────
    // PROFILE 2: S-CURVE (JERK-LIMITED) PROFILE
    // ─────────────────────────────────────────────────────────────
    Serial.println("\n--- [Profile 2] S-CURVE (JERK-LIMITED) PROFILE ---");
    Serial.print("Parameters: Steps = ");
    Serial.print(MOVE_STEPS);
    Serial.print(" | MaxSpeed = ");
    Serial.print(CRUISE_SPEED);
    Serial.print(" | Accel = ");
    Serial.print(ACCEL);
    Serial.print(" | Jerk = ");
    Serial.print(JERK);
    Serial.println(" steps/s³");

    motor.setLED(BoxDriveLED::PULSE);
    tStart = millis();

    // Execute S-Curve trajectory move
    motor.sCurveMove(MOVE_STEPS, CRUISE_SPEED, ACCEL, JERK);

    uint32_t durationSCurve = millis() - tStart;
    Serial.print("[✓] S-Curve move finished in ");
    Serial.print(durationSCurve);
    Serial.print(" ms | Final Encoder Angle: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");

    motor.setLED(BoxDriveLED::BLINK_SLOW);
    delay(1500);

    // Return to 0 using S-Curve
    Serial.println("Returning to base position with S-Curve...");
    motor.sCurveMove(-MOVE_STEPS, CRUISE_SPEED, ACCEL, JERK);

    Serial.println("\n[SUMMARY]");
    Serial.print("Trapezoidal Duration: ");
    Serial.print(durationTrapezoid);
    Serial.println(" ms (Punchier start, sharp deceleration)");
    Serial.print("S-Curve Duration    : ");
    Serial.print(durationSCurve);
    Serial.println(" ms (Smooth acceleration ramp, minimal mechanical resonance)");

    Serial.println("\nPausing 4 seconds before next comparison cycle...");
    delay(4000);
}
