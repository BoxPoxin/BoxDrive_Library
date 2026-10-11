/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 12: CAN Bus Multi-Axis Synchronized Motion                       ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates coordinated motion across multiple BoxDrives using CAN.     ║
 * ║  One BoxDrive acts as the COORDINATOR, sending position commands           ║
 * ║  to multiple axis nodes simultaneously.                                    ║
 * ║                                                                            ║
 * ║  ARCHITECTURE:                                                             ║
 * ║   COORDINATOR (Node 0x00) → sends commands to:                            ║
 * ║     Axis 1 (Node 0x01) — e.g., X axis                                    ║
 * ║     Axis 2 (Node 0x02) — e.g., Y axis                                    ║
 * ║     Axis 3 (Node 0x03) — e.g., Z axis                                    ║
 * ║                                                                            ║
 * ║  Upload with MY_ROLE set appropriately for each BoxDrive.                  ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// ── Configure role for each BoxDrive ─────────────────────────
// Uncomment ONE role per BoxDrive:
#define MY_ROLE_COORDINATOR
// #define MY_ROLE_AXIS

#ifdef MY_ROLE_COORDINATOR
  #define MY_NODE_ID  0x00
#else
  #define MY_NODE_ID  0x01  // Change to 0x02, 0x03 for other axes
#endif

void setup() {
    motor.begin();
    motor.setCurrent(800);
    motor.setMicrosteps(16);
    motor.setSpeed(2000);
    motor.setAcceleration(4000);

    bool canOk = motor.can.begin(BD_CAN_SPEED_500K, MY_NODE_ID);
    Serial.print("CAN init: ");
    Serial.println(canOk ? "OK" : "FAILED");

#ifdef MY_ROLE_COORDINATOR
    Serial.println("═══ COORDINATOR — Sending motion commands ═══");
#else
    motor.enableMotor();
    motor.enableClosedLoop();
    Serial.print("═══ AXIS NODE 0x");
    Serial.print(MY_NODE_ID, HEX);
    Serial.println(" — Awaiting commands ═══");
#endif
}

void loop() {
#ifdef MY_ROLE_COORDINATOR
    coordinatorLoop();
#else
    axisLoop();
#endif
}

#ifdef MY_ROLE_COORDINATOR
void coordinatorLoop() {
    // Sequence: draw a square pattern across 2 axes
    struct Point { float x; float y; };
    const Point square[] = {
        {0, 0}, {90, 0}, {90, 90}, {0, 90}, {0, 0}
    };

    for (int i = 0; i < 5; i++) {
        Serial.print("Moving to (");
        Serial.print(square[i].x, 0);
        Serial.print("°, ");
        Serial.print(square[i].y, 0);
        Serial.println("°)");

        // Send position to Axis 1 (X)
        motor.can.sendPosition(0x01, square[i].x);
        // Send position to Axis 2 (Y)
        motor.can.sendPosition(0x02, square[i].y);

        delay(2000); // Wait for axes to reach position
    }

    Serial.println("Square complete! Repeating...\n");
    delay(1000);
}
#endif

#ifndef MY_ROLE_COORDINATOR
void axisLoop() {
    // Listen for position commands
    uint32_t msgId;
    uint8_t  data[8];
    uint8_t  len;

    while (motor.can.available()) {
        if (motor.can.receive(msgId, data, len)) {
            if (msgId == BD_CAN_MSG_POSITION && len >= 6) {
                uint8_t targetNode = data[0];
                if (targetNode == MY_NODE_ID || targetNode == 0xFF) {
                    float angle;
                    memcpy(&angle, &data[2], sizeof(float));
                    motor.setTargetAngle(angle);
                    Serial.print("→ Target: ");
                    Serial.print(angle, 1);
                    Serial.println("°");
                }
            }
        }
    }

    // Update closed-loop position controller
    motor.updateClosedLoop();
}
#endif
