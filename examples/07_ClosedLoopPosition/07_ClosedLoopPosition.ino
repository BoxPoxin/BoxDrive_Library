/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 07: Closed-Loop Position Control (PID)                            ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates closed-loop position control using encoder feedback.         ║
 * ║  A PID controller reads the AS5600 encoder and generates step pulses       ║
 * ║  to drive the motor to the exact target angle.                             ║
 * ║                                                                            ║
 * ║  WHY CLOSED-LOOP?                                                          ║
 * ║   • Open-loop steppers can lose steps under load → position drifts.       ║
 * ║   • Closed-loop uses the encoder to detect and correct errors.             ║
 * ║   • Result: guaranteed position accuracy even under varying loads.         ║
 * ║                                                                            ║
 * ║  PID TUNING GUIDE:                                                         ║
 * ║   Kp (Proportional): Higher = faster response, but can overshoot.         ║
 * ║   Ki (Integral):     Eliminates steady-state error, but can oscillate.    ║
 * ║   Kd (Derivative):   Dampens oscillation, but sensitive to noise.          ║
 * ║                                                                            ║
 * ║  START WITH:  Kp=2.0, Ki=0.01, Kd=0.5  and adjust from there.            ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// Target angles to cycle through
const float targets[] = {0, 45, 90, 180, 270, 360, 180, 0};
const uint8_t numTargets = sizeof(targets) / sizeof(targets[0]);
uint8_t targetIndex = 0;
uint32_t lastTargetChange = 0;

void setup() {
    motor.begin();
    motor.setCurrent(800, 400);
    motor.setMicrosteps(16);
    motor.enableMotor();

    // If your motor vibrates or turns opposite to encoder, invert direction:
    // motor.invertDirection(true);

    // Enable smooth closed-loop PID control (Kp, Ki, Kd)
    motor.enableClosedLoop(1.2, 0.005, 0.35);

    // Set position tolerance deadband (0.4° prevents noise buzzing at rest)
    motor.setPositionTolerance(0.4);

    Serial.println("Closed-Loop Position Control Demo");
    Serial.println("The motor will seek to each target angle using PID.");
    Serial.println();

    lastTargetChange = millis();
}

void loop() {
    // CRITICAL: Must call updateClosedLoop() every loop iteration!
    // This reads the encoder, computes PID, and outputs step pulses.
    motor.updateClosedLoop();

    // Change target every 3 seconds
    if (millis() - lastTargetChange > 3000) {
        float target = targets[targetIndex];
        motor.setTargetAngle(target);

        Serial.print("Target: ");
        Serial.print(target, 1);
        Serial.println("°");

        targetIndex = (targetIndex + 1) % numTargets;
        lastTargetChange = millis();
    }

    // Print position info at ~10 Hz
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();

        float current = motor.encoder.readAngleDeg();
        float target  = motor.getTargetAngle();
        float error   = target - current;

        // Handle wrap-around
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        Serial.print("Target: ");
        Serial.print(target, 1);
        Serial.print("° | Current: ");
        Serial.print(current, 2);
        Serial.print("° | Error: ");
        Serial.print(error, 2);
        Serial.println("°");
    }
}
