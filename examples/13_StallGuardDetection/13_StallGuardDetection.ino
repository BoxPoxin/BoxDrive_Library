/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 13: StallGuard Sensorless Stall Detection                         ║
 * ║  BoxDrive Library — Advanced TMC2209 Diagnostics                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates sensorless load detection and stall sensing using the TMC2209 ║
 * ║  integrated StallGuard4 technology via the DIAG pin (GPIO 3).               ║
 * ║                                                                            ║
 * ║  HOW IT WORKS:                                                             ║
 * ║   • StallGuard measures the back-EMF of the stepper motor coils.           ║
 * ║   • As mechanical load increases, the StallGuard value decreases toward 0. ║
 * ║   • When it drops below the threshold, the driver asserts DIAG (active HIGH)║
 * ║   • The motor stops instantly to prevent lost steps or mechanical damage.  ║
 * ║                                                                            ║
 * ║  WIRING:                                                                   ║
 * ║   • BoxDrive mounted to NEMA 17 motor with XT30 power (4.75V–24V).         ║
 * ║   • USB-C connected to computer for Serial Monitor.                        ║
 * ║                                                                            ║
 * ║  TUNING GUIDE:                                                             ║
 * ║   • Threshold range: 0 to 255.                                             ║
 * ║   • Higher threshold (e.g. 150-200) = MORE sensitive (triggers easily).    ║
 * ║   • Lower threshold (e.g. 30-70) = LESS sensitive (requires hard stall).   ║
 * ║   • Typical starting value: 80–120.                                        ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// Flag set by stall callback
volatile bool stallOccurred = false;

// Stall interrupt callback function
void handleStall() {
    stallOccurred = true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("  BoxDrive Example 13: StallGuard Load Detection  ");
    Serial.println("==================================================");

    // 1. Initialize BoxDrive
    if (!motor.begin()) {
        Serial.println("[-] Peripherals failed to initialize. Check hardware.");
    }

    // 2. Configure motor current
    // Note: StallGuard accuracy is heavily dependent on current setting.
    // Use consistent current during tuning.
    motor.setCurrent(800);
    motor.setHoldCurrent(200);

    // 3. Set speed & acceleration
    // StallGuard is most effective at moderate-to-high velocities (e.g. 400-2000 steps/s)
    motor.setSpeed(1200);
    motor.setAcceleration(2400);

    // 4. Configure driver mode: StealthChop is required for StallGuard4 on TMC2209
    motor.setMode(BoxDriveMode::STEALTH_CHOP);
    motor.setMicrosteps(16);

    // 5. Enable StallGuard with threshold (default: 90)
    // Adjust threshold if it false-triggers or fails to trigger
    const uint8_t STALL_THRESHOLD = 95;
    motor.enableStallGuard(STALL_THRESHOLD);

    // Register callback for stall events
    motor.onStall(handleStall);

    // 6. Enable motor
    motor.enableMotor();
    motor.setLED(BoxDriveLED::BLINK_SLOW);

    Serial.print("[+] StallGuard enabled with threshold: ");
    Serial.println(STALL_THRESHOLD);
    Serial.println("[+] Lightly grasp or block the motor shaft to trigger stall.");
    Serial.println("--------------------------------------------------");
}

void loop() {
    Serial.println("\n[→] Starting forward rotation test (Target: 6400 steps / 2 revs)...");
    stallOccurred = false;
    motor.setLED(BoxDriveLED::BLINK_FAST);

    // Continuous rotation or incremental step loop while monitoring StallGuard
    const int32_t targetSteps = 6400;
    int32_t stepsTaken = 0;

    while (stepsTaken < targetSteps && !stallOccurred) {
        // Step in small chunks to monitor DIAG pin and SG value in real-time
        motor.move(16);
        stepsTaken += 16;

        // Read real-time StallGuard value (higher = light load, lower = heavy load)
        uint16_t sgVal = motor.readStallGuardValue();

        // Print load meter periodically
        if (stepsTaken % 400 == 0) {
            Serial.print("Step: ");
            Serial.print(stepsTaken);
            Serial.print(" / ");
            Serial.print(targetSteps);
            Serial.print(" | SG Load Metric: ");
            Serial.print(sgVal);
            Serial.print(" | Encoder: ");
            Serial.print(motor.encoder.readAngleDeg(), 1);
            Serial.println("°");
        }

        // Direct check on DIAG pin or callback flag
        if (motor.isStalled() || stallOccurred) {
            stallOccurred = true;
            break;
        }

        delayMicroseconds(50);
    }

    if (stallOccurred) {
        // HALT MOTOR IMMEDIATELY
        motor.stop();
        motor.setLED(BoxDriveLED::ERROR_FLASH);

        Serial.println("\n[!] >>> STALL DETECTED! MOTOR STOPPED SAFELY <<<");
        Serial.print("    Stopped at step: ");
        Serial.print(stepsTaken);
        Serial.print(" | Encoder Angle: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.println("°");

        // Print diagnostic registers
        BoxDriveStatus status = motor.getStatus();
        Serial.print("    SG Result at trip: ");
        Serial.println(status.sgResult);
        Serial.print("    Driver Status: 0x");
        Serial.println(status.drvStatus, HEX);

        // Pause to let user release obstruction
        Serial.println("    Clearing stall in 3 seconds...");
        delay(3000);
        stallOccurred = false;
        motor.setLED(BoxDriveLED::BLINK_SLOW);
    } else {
        Serial.println("[✓] Move completed without stall.");
        delay(1000);
    }

    // Now test reverse direction
    Serial.println("\n[←] Moving reverse (Target: -6400 steps)...");
    stepsTaken = 0;
    while (stepsTaken < targetSteps && !stallOccurred) {
        motor.move(-16);
        stepsTaken += 16;

        if (motor.isStalled() || stallOccurred) {
            stallOccurred = true;
            break;
        }
        delayMicroseconds(50);
    }

    if (stallOccurred) {
        motor.stop();
        motor.setLED(BoxDriveLED::ERROR_FLASH);
        Serial.println("[!] >>> STALL DETECTED DURING REVERSE MOTION <<<");
        delay(3000);
    } else {
        Serial.println("[✓] Reverse move completed smoothly.");
    }

    delay(2000);
}
