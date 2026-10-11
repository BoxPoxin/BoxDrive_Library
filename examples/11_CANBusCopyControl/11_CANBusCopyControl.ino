/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 11: CAN Bus Copy-Control (Leader/Follower)                        ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  COPY-CONTROL: One BoxDrive (LEADER) broadcasts its encoder position       ║
 * ║  over CAN bus, and one or more BoxDrives (FOLLOWERs) mirror that           ║
 * ║  position in real-time.                                                    ║
 * ║                                                                            ║
 * ║  Turn the leader's shaft by hand → all followers replicate the motion!     ║
 * ║                                                                            ║
 * ║  SETUP:                                                                    ║
 * ║   1. Upload THIS SKETCH to all BoxDrives on the CAN bus.                   ║
 * ║   2. Change ROLE to BD_ROLE_LEADER on ONE BoxDrive.                        ║
 * ║   3. Change ROLE to BD_ROLE_FOLLOWER on all other BoxDrives.               ║
 * ║   4. Connect all BoxDrives via XT30 daisy-chain cables.                    ║
 * ║   5. Turn the leader shaft → followers follow!                             ║
 * ║                                                                            ║
 * ║  APPLICATIONS:                                                             ║
 * ║   • Multi-axis synchronized motion (robotic arms, gantries)               ║
 * ║   • Master-slave motion replication                                        ║
 * ║   • Teach pendant: manually position leader, followers copy                ║
 * ║   • Mirror/duplicate machines                                              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// ╔══════════════════════════════════════════╗
// ║  CONFIGURE THESE FOR EACH BOXDRIVE:     ║
// ╚══════════════════════════════════════════╝

// Set role: uncomment ONE of these lines
#define BD_ROLE_LEADER      // This unit broadcasts encoder position
// #define BD_ROLE_FOLLOWER    // This unit mirrors the leader's position

// Node IDs (must be unique for each BoxDrive on the bus)
#define MY_NODE_ID      0x01    // Change this per unit (0x01, 0x02, etc.)
#define LEADER_NODE_ID  0x01    // Followers need to know the leader's ID

void setup() {
    motor.begin();
    motor.setCurrent(800, 400);
    motor.setMicrosteps(16);
    motor.setSpeed(2000);
    motor.setAcceleration(4000);

    // Initialize CAN bus
    bool canOk = motor.can.begin(BD_CAN_SPEED_500K, MY_NODE_ID);
    if (!canOk) {
        Serial.println("CAN init failed! Check wiring.");
        motor.setLED(BoxDriveLED::ERROR_FLASH);
        while (1) { motor.updateLED(); delay(10); }
    }

#ifdef BD_ROLE_LEADER
    // ── LEADER SETUP ─────────────────────────────────────────
    // Disable motor so the shaft can be turned freely by hand
    motor.disableMotor();
    motor.setCANLeader();
    motor.setLED(BoxDriveLED::BLINK_SLOW);

    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║  ROLE: LEADER (Node 0x01)                ║");
    Serial.println("║  Turn the shaft → followers will copy!   ║");
    Serial.println("╚══════════════════════════════════════════╝");
#endif

#ifdef BD_ROLE_FOLLOWER
    // ── FOLLOWER SETUP ───────────────────────────────────────
    motor.enableMotor();
    motor.setCANFollower(LEADER_NODE_ID);
    motor.setLED(BoxDriveLED::BLINK_FAST);

    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║  ROLE: FOLLOWER (mirroring Leader 0x01)  ║");
    Serial.println("║  Motor will copy leader's position.      ║");
    Serial.println("╚══════════════════════════════════════════╝");
#endif

    Serial.println();
}

void loop() {
    // CRITICAL: Update CAN copy-control state machine
    motor.updateCAN();
    motor.updateLED();

    // Print status at 2 Hz
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();

        float myAngle = motor.encoder.readAngleDeg();

#ifdef BD_ROLE_LEADER
        Serial.print("[LEADER] My angle: ");
        Serial.print(myAngle, 2);
        Serial.println("° (broadcasting to followers)");
#endif

#ifdef BD_ROLE_FOLLOWER
        float target = motor.getTargetAngle();
        float error = target - myAngle;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        Serial.print("[FOLLOWER] Target: ");
        Serial.print(target, 1);
        Serial.print("° | Actual: ");
        Serial.print(myAngle, 2);
        Serial.print("° | Error: ");
        Serial.print(error, 2);
        Serial.println("°");
#endif
    }
}
