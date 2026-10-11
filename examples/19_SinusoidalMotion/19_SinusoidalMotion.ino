/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 19: Harmonic Sinusoidal Motion & Serial Plotter Output             ║
 * ║  BoxDrive Library — Continuous Harmonic Oscillation & Wave Tracking        ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Generates pure mathematical sine-wave position profiles:                  ║
 * ║    θ(t) = A · sin(2π · f · t)                                               ║
 * ║                                                                            ║
 * ║  APPLICATIONS:                                                             ║
 * ║   • Camera pan-tilt gimbals for cinematic sweeps.                          ║
 * ║   • Inverted pendulum balancing & robotic gait generation.                 ║
 * ║   • Accelerated mechanical fatigue & vibration endurance testing.          ║
 * ║                                                                            ║
 * ║  VISUALIZATION:                                                            ║
 * ║   • Open Tools -> 'Serial Plotter' in Arduino IDE to see beautiful,        ║
 * ║     real-time synchronized sine waves of Target Angle vs Measured Angle!   ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initial message for Serial Monitor or Plotter
    Serial.println("TargetDeg,MeasuredDeg,Velocity");

    motor.begin();
    motor.setCurrent(850);
    motor.setHoldCurrent(300);
    motor.setMode(BoxDriveMode::STEALTH_CHOP);
    motor.setMicrosteps(16);
    motor.enableMotor();
    motor.setLED(BoxDriveLED::PULSE);

    // Set motor reference zero
    motor.setPosition(0);
    motor.encoder.resetMultiTurn();
}

void loop() {
    // Wave parameters:
    const float AMPLITUDE_DEG = 45.0f; // ±45 degrees sweep
    const float FREQUENCY_HZ  = 0.5f;  // 0.5 Hz = one cycle every 2 seconds
    const float TWO_PI_F      = 2.0f * PI * FREQUENCY_HZ;

    uint32_t startTime = millis();
    uint32_t testDurationMs = 20000; // Run 10 full cycles (20 seconds)

    while (millis() - startTime < testDurationMs) {
        float t = (millis() - startTime) / 1000.0f; // Seconds

        // Compute instantaneous target angle on the sine curve
        float targetAngle = AMPLITUDE_DEG * sin(TWO_PI_F * t);

        // Move motor to target angle
        motor.moveToAngle(targetAngle);

        // Read actual physical encoder angle
        float actualAngle = motor.encoder.readMultiTurnDeg();
        float velocityDPS = motor.encoder.getVelocityDPS();

        // Print CSV format for Arduino Serial Plotter
        Serial.print(targetAngle, 2);
        Serial.print(",");
        Serial.print(actualAngle, 2);
        Serial.print(",");
        Serial.println(velocityDPS, 1);

        motor.updateLED();
        delay(10);
    }

    // Pause between tests
    motor.moveToAngle(0.0f);
    motor.setLED(BoxDriveLED::BLINK_SLOW);
    delay(3000);

    // Demonstrate higher frequency sweep
    Serial.println("\n[Testing 1.0 Hz higher frequency oscillation for 10 seconds]");
    motor.sinusoidalMotion(30.0f, 1.0f, 10); // 10 cycles at 1 Hz
    motor.moveToAngle(0.0f);
    delay(3000);
}
