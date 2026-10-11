/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 02: StealthChop Silent Operation                                  ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates the TMC2209 StealthChop mode for ultra-silent motor          ║
 * ║  operation. StealthChop uses voltage-mode PWM chopping, which              ║
 * ║  eliminates audible motor whine completely.                                ║
 * ║                                                                            ║
 * ║  USE CASE: 3D printer in a bedroom, camera slider, lab equipment,          ║
 * ║  any application where noise is unacceptable.                              ║
 * ║                                                                            ║
 * ║  NOTES:                                                                    ║
 * ║   • StealthChop is the DEFAULT mode on BoxDrive.                           ║
 * ║   • At higher speeds, the driver automatically transitions to              ║
 * ║     SpreadCycle if TPWMTHRS is configured (hybrid mode).                   ║
 * ║   • StealthChop works best at low-to-medium speeds (<500 RPM).            ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    motor.begin();

    // Explicitly set StealthChop mode (this is already the default)
    motor.setMode(BoxDriveMode::STEALTH_CHOP);

    // Use high microstepping for the smoothest, quietest motion
    // 256 microsteps with interpolation = butter-smooth movement
    motor.setMicrosteps(256);

    // Lower current for silent operation (less heat, less vibration)
    motor.setCurrent(600);

    // Moderate speed for StealthChop (it excels at low-medium speeds)
    motor.setSpeed(800);
    motor.setAcceleration(1500);

    motor.enableMotor();

    Serial.println("StealthChop mode active — listen for silence!");
    Serial.println("The motor should be nearly inaudible.");
}

void loop() {
    // Slow, smooth rotation — notice how quiet it is
    Serial.println("Gentle 90° turn...");
    motor.moveToAngle(90);
    delay(2000);

    Serial.println("Back to 0°...");
    motor.moveToAngle(0);
    delay(2000);

    Serial.println("Full 360° rotation...");
    motor.moveToAngle(360);
    delay(2000);

    Serial.println("Return to start...");
    motor.moveToAngle(0);
    delay(2000);

    // Print encoder position to verify accuracy
    Serial.print("Encoder reads: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("° (should be ~0°)");
    Serial.println();
}
