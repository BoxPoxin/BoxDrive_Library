/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 10: CAN Bus Single-Axis Communication                            ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Demonstrates CAN bus communication on a single BoxDrive.                  ║
 * ║  Sends and receives CAN messages, broadcasts encoder position,             ║
 * ║  and responds to remote position commands.                                 ║
 * ║                                                                            ║
 * ║  CAN BUS ON BOXDRIVE:                                                      ║
 * ║   • Transceiver: SN65HVD230D                                              ║
 * ║   • ESP32-C6 TWAI peripheral (CAN 2.0B compatible)                        ║
 * ║   • TX = GPIO 20, RX = GPIO 21                                            ║
 * ║   • Daisy-chained via XT30 connectors (power + CAN)                       ║
 * ║   • Supports 125K, 250K, 500K, 1M baud rates                             ║
 * ║                                                                            ║
 * ║  WIRING:                                                                   ║
 * ║   • Connect two BoxDrives via their XT30 connectors                        ║
 * ║   • CAN_H and CAN_L are wired through the XT30 daisy-chain cable         ║
 * ║   • 120Ω termination resistor at each end of the bus                       ║
 * ║                                                                            ║
 * ║  Upload this sketch to one BoxDrive. It will:                              ║
 * ║   1. Broadcast its encoder position every 100ms                            ║
 * ║   2. Send heartbeats every 1 second                                        ║
 * ║   3. Listen for incoming position commands                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// This node's CAN ID (change for each BoxDrive on the bus!)
#define MY_NODE_ID  0x01

uint32_t lastBroadcast = 0;
uint32_t lastHeartbeat = 0;

void setup() {
    motor.begin();
    motor.setCurrent(800, 400);
    motor.setMicrosteps(16);
    motor.setSpeed(1000);
    motor.setAcceleration(2000);
    motor.enableMotor();

    // Initialize CAN bus at 500 kbps
    bool canOk = motor.can.begin(BD_CAN_SPEED_500K, MY_NODE_ID);
    Serial.print("[CAN] Initialization: ");
    Serial.println(canOk ? "OK" : "FAILED");

    if (!canOk) {
        Serial.println("Check CAN wiring and termination resistors!");
        motor.setLED(BoxDriveLED::ERROR_FLASH);
        while (1) { motor.updateLED(); delay(10); }
    }

    motor.setLED(BoxDriveLED::BLINK_SLOW);
    Serial.println("CAN bus ready. Node ID: 0x01");
    Serial.println("Broadcasting encoder position...");
    Serial.println();
}

void loop() {
    motor.updateLED();

    // ── Broadcast encoder position every 100 ms ─────────────
    if (millis() - lastBroadcast >= 100) {
        float angle = motor.encoder.readAngleDeg();
        motor.can.broadcastEncoder(angle);
        lastBroadcast = millis();
    }

    // ── Send heartbeat every 1 second ───────────────────────
    if (millis() - lastHeartbeat >= 1000) {
        motor.can.sendHeartbeat();
        lastHeartbeat = millis();

        // Print status
        Serial.print("♥ Heartbeat | Angle: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.print("° | Pos: ");
        Serial.println(motor.getPosition());
    }

    // ── Check for incoming CAN messages ─────────────────────
    uint32_t msgId;
    uint8_t  data[8];
    uint8_t  len;

    while (motor.can.available()) {
        if (motor.can.receive(msgId, data, len)) {
            Serial.print("[CAN RX] ID: 0x");
            Serial.print(msgId, HEX);
            Serial.print(" | Data: ");
            for (uint8_t i = 0; i < len; i++) {
                if (data[i] < 0x10) Serial.print("0");
                Serial.print(data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Handle position commands addressed to this node
            if (msgId == BD_CAN_MSG_POSITION) {
                uint8_t targetNode = data[0];
                if (targetNode == MY_NODE_ID || targetNode == 0xFF) {
                    float angle;
                    memcpy(&angle, &data[2], sizeof(float));
                    Serial.print("[CAN] Received position command: ");
                    Serial.print(angle, 2);
                    Serial.println("°");
                    motor.moveToAngle(angle);
                }
            }

            // Handle heartbeats from other nodes
            if (msgId == BD_CAN_MSG_HEARTBEAT) {
                uint8_t senderId = data[0];
                Serial.print("[CAN] Heartbeat from node 0x");
                Serial.println(senderId, HEX);
            }
        }
    }
}
