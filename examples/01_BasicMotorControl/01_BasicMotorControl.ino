/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 01: Basic Motor Control                                           ║
 * ║  BoxDrive Library — Getting Started                                        ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  This is the simplest BoxDrive sketch. It initializes the board,           ║
 * ║  enables the motor, sets the current, and moves back and forth.            ║
 * ║                                                                            ║
 * ║  WIRING:                                                                   ║
 * ║   • BoxDrive mounted on NEMA 17 with magnet glued to shaft                ║
 * ║   • XT30 power cable connected (4.75V–24V)                                ║
 * ║   • USB-C connected for programming & serial monitor                       ║
 * ║                                                                            ║
 * ║  EXPECTED OUTPUT:                                                          ║
 * ║   • Motor rotates 360° CW, pauses, then 360° CCW, repeating forever.      ║
 * ║   • Serial monitor shows position and encoder angle.                       ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

// Create a BoxDrive motor object
BoxDrive motor;

void setup() {
    // Initialize BoxDrive (sets up GPIO, UART to TMC2209, I2C to AS5600)
    motor.begin();

    // Set motor current to 800 mA (safe for most NEMA 17 motors)
    // Range: 100–2000 mA. Check your motor's rated current!
    motor.setCurrent(800);

    // Set the speed: 1000 steps per second
    motor.setSpeed(1000);

    // Set acceleration: 2000 steps/sec² (smooth ramp-up and ramp-down)
    motor.setAcceleration(2000);

    // Enable the motor driver (motor holds position)
    motor.enableMotor();

    Serial.println("Setup complete. Motor will rotate back and forth.");
}

void loop() {
    // Move 3200 steps clockwise (= 1 full revolution at 16 microsteps)
    // 200 full steps × 16 microsteps = 3200 steps/revolution
    Serial.println("→ Moving CW 360°...");
    motor.move(3200);

    // Print the current position and encoder angle
    Serial.print("Position: ");
    Serial.print(motor.getPosition());
    Serial.print(" steps | Encoder: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");

    delay(1000);

    // Move 3200 steps counter-clockwise (back to start)
    Serial.println("← Moving CCW 360°...");
    motor.move(-3200);

    Serial.print("Position: ");
    Serial.print(motor.getPosition());
    Serial.print(" steps | Encoder: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");

    delay(1000);
}
