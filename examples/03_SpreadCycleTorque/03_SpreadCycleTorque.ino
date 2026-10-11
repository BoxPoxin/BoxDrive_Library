/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 03: SpreadCycle High-Torque Mode                                  ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates SpreadCycle mode for maximum torque and precision at          ║
 * ║  higher speeds. SpreadCycle uses current-mode chopping which provides       ║
 * ║  better torque and more precise current regulation.                         ║
 * ║                                                                            ║
 * ║  USE CASE: CNC milling, heavy loads, high-speed movement,                  ║
 * ║  applications where torque > silence.                                      ║
 * ║                                                                            ║
 * ║  NOTES:                                                                    ║
 * ║   • SpreadCycle is noisier than StealthChop but provides more torque.      ║
 * ║   • Best for speeds above 300–500 RPM.                                    ║
 * ║   • Full 2.0 A RMS current available in this mode.                        ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    motor.begin();

    // Switch to SpreadCycle for maximum torque
    motor.setMode(BoxDriveMode::SPREAD_CYCLE);

    // Use full-rated current for heavy-load applications
    // WARNING: Make sure your motor is rated for this current!
    // Set run current = 1500 mA, hold current = 500 mA
    motor.setCurrent(1500, 500);

    // 16 microsteps is a good balance between smoothness and torque
    motor.setMicrosteps(16);

    // Higher speed — SpreadCycle handles it well
    motor.setSpeed(3000);
    motor.setAcceleration(5000);

    motor.enableMotor();

    Serial.println("SpreadCycle mode — maximum torque enabled!");
}

void loop() {
    // Fast back-and-forth motion
    Serial.println("Fast 5-revolution CW...");
    motor.move(3200 * 5);  // 5 full revolutions
    delay(500);

    Serial.println("Fast 5-revolution CCW...");
    motor.move(-3200 * 5);
    delay(500);

    // Print diagnostics
    Serial.print("StallGuard value: ");
    Serial.println(motor.readStallGuardValue());
    Serial.print("Encoder: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");
    Serial.println();
}
