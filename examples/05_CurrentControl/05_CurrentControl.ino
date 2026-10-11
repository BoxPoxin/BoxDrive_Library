/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 05: Current Control & Power Management                            ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Shows how to control motor current for different operating conditions:    ║
 * ║   • High current for heavy loads                                           ║
 * ║   • Low current for light loads (saves power, reduces heat)                ║
 * ║   • Hold current for maintaining position at reduced power                 ║
 * ║   • Dynamic current adjustment based on load                               ║
 * ║                                                                            ║
 * ║  CURRENT GUIDELINES for NEMA 17:                                           ║
 * ║   • Light load: 400–600 mA                                                ║
 * ║   • Normal load: 600–1000 mA                                              ║
 * ║   • Heavy load: 1000–1500 mA                                              ║
 * ║   • Max rated: 1500–2000 mA (check your motor!)                           ║
 * ║   • Hold current: typically 30–50% of run current                          ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    motor.begin();

    // Start with a conservative current
    motor.setCurrent(600, 300); // 600 mA run, 300 mA hold
    motor.setSpeed(1000);
    motor.setAcceleration(2000);
    motor.enableMotor();

    Serial.println("Current control demo");
    Serial.println("Watch Serial Monitor for current changes.");
    Serial.println();
}

void loop() {
    // ── Demo 1: Low current (energy saving) ──────────────────
    Serial.println("═══ LOW CURRENT: 400 mA ═══");
    Serial.println("Use case: Light loads, slow motion, power saving");
    motor.setCurrent(400, 200);
    motor.move(3200);
    printCurrentInfo();
    delay(2000);
    motor.move(-3200);
    delay(1000);

    // ── Demo 2: Normal current ───────────────────────────────
    Serial.println("═══ NORMAL CURRENT: 800 mA ═══");
    Serial.println("Use case: General purpose, 3D printing");
    motor.setCurrent(800, 400);
    motor.move(3200);
    printCurrentInfo();
    delay(2000);
    motor.move(-3200);
    delay(1000);

    // ── Demo 3: High current (maximum torque) ────────────────
    Serial.println("═══ HIGH CURRENT: 1500 mA ═══");
    Serial.println("Use case: CNC, heavy loads, fast acceleration");
    motor.setCurrent(1500, 500);
    motor.move(3200);
    printCurrentInfo();
    delay(2000);
    motor.move(-3200);
    delay(1000);

    // ── Demo 4: Dynamic current adjustment ───────────────────
    Serial.println("═══ DYNAMIC CURRENT RAMP ═══");
    Serial.println("Gradually increasing current from 200 to 1500 mA...");
    for (uint16_t ma = 200; ma <= 1500; ma += 100) {
        motor.setCurrent(ma);
        Serial.print("  Current: ");
        Serial.print(ma);
        Serial.println(" mA");
        motor.move(200);
        delay(100);
    }
    Serial.println("Ramp complete.");
    motor.setCurrent(800); // Reset to normal
    delay(3000);

    Serial.println("\n--- Cycle complete ---\n");
}

void printCurrentInfo() {
    BoxDriveStatus status = motor.getStatus();
    Serial.print("  Configured: ");
    Serial.print(motor.getCurrent());
    Serial.println(" mA");
    Serial.print("  CS Actual (scale 0-31): ");
    Serial.println(status.csActual);
    Serial.print("  Over-temp warning: ");
    Serial.println(status.overTemp ? "YES ⚠" : "No");
    Serial.println();
}
