/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 04: Microstepping Demo                                            ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates all available microstepping resolutions from full-step       ║
 * ║  to 256 microsteps. You can hear and feel the difference!                  ║
 * ║                                                                            ║
 * ║  The TMC2209 supports: 1, 2, 4, 8, 16, 32, 64, 128, 256 µsteps           ║
 * ║  With 256µ interpolation enabled, even 16µ sounds smooth.                  ║
 * ║                                                                            ║
 * ║  STEPS PER REVOLUTION by µstep setting:                                    ║
 * ║   Full step (1):    200 steps/rev                                          ║
 * ║   1/2 step:         400 steps/rev                                          ║
 * ║   1/4 step:         800 steps/rev                                          ║
 * ║   1/8 step:       1,600 steps/rev                                          ║
 * ║   1/16 step:      3,200 steps/rev                                          ║
 * ║   1/32 step:      6,400 steps/rev                                          ║
 * ║   1/64 step:     12,800 steps/rev                                          ║
 * ║   1/128 step:    25,600 steps/rev                                          ║
 * ║   1/256 step:    51,200 steps/rev                                          ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// All supported microstepping values
const uint16_t microstepValues[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
const uint8_t  numSettings = sizeof(microstepValues) / sizeof(microstepValues[0]);

void setup() {
    motor.begin();
    motor.setCurrent(800);
    motor.setSpeed(800);
    motor.setAcceleration(2000);
    motor.enableMotor();

    Serial.println("Microstepping comparison demo");
    Serial.println("Listen to the motor at each setting!");
    Serial.println("Higher microsteps = smoother & quieter.");
    Serial.println();
}

void loop() {
    for (uint8_t i = 0; i < numSettings; i++) {
        uint16_t ms = microstepValues[i];
        uint32_t stepsPerRev = 200 * (uint32_t)ms;

        Serial.print("══════ MICROSTEPS: 1/");
        Serial.print(ms);
        Serial.print(" (");
        Serial.print(stepsPerRev);
        Serial.println(" steps/rev) ══════");

        motor.setMicrosteps(ms);

        // Adjust speed proportionally so angular speed stays constant
        motor.setSpeed(stepsPerRev / 2); // ~0.5 rev/sec

        // Rotate exactly one revolution
        motor.move(stepsPerRev);

        Serial.print("  Encoder angle: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.println("°");
        Serial.println();

        delay(2000);

        // Return to start
        motor.move(-stepsPerRev);
        delay(1000);
    }

    Serial.println("Demo complete! Restarting...\n");
    delay(3000);
}
