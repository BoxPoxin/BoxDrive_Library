/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 20: Full System Interactive Showcase                               ║
 * ║  BoxDrive Library — Master Unified Demonstration                             ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  The complete flagship demonstration combining all BoxDrive capabilities:    ║
 * ║                                                                            ║
 * ║  FEATURES DEMONSTRATED CONCURRENTLY:                                       ║
 * ║   [1] Interactive Serial Command Console (CLI)                              ║
 * ║   [2] WiFi Hotspot Web Dashboard (192.168.4.1)                             ║
 * ║   [3] Closed-Loop PID Position Control with AS5600 feedback                ║
 * ║   [4] Hand-Kinesthetic Motion Record & Playback (5-second buffer)           ║
 * ║   [5] CAN Bus Daisy-Chain Broadcast & Copy-Control                          ║
 * ║   [6] TMC2209 StealthChop Silent Driving & StallGuard Safety                ║
 * ║   [7] Real-time RGB/LED Status Indication                                  ║
 * ║                                                                            ║
 * ║  SERIAL CLI COMMANDS:                                                      ║
 * ║   • help          : Print command reference list                           ║
 * ║   • status        : Full system telemetry report                           ║
 * ║   • move <angle>  : Move motor to target angle in degrees                  ║
 * ║   • loop on|off   : Toggle Closed-Loop PID control                         ║
 * ║   • rec start     : Start 5-second hand teaching recording                 ║
 * ║   • rec play [spd]: Replay recorded motion at speed multiplier (e.g. 1.0)   ║
 * ║   • wifi on|off   : Start / stop WiFi Hotspot dashboard                    ║
 * ║   • can leader    : Enable CAN Leader mode (broadcasts encoder)             ║
 * ║   • can follow <id>: Enable CAN Follower mode (mirrors leader ID)          ║
 * ║   • diag          : Run full hardware diagnostic suite                     ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// Serial input buffer
String inputBuffer = "";

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                BOXDRIVE INTERACTIVE COMMAND CLI                ║");
    Serial.println("╠════════════════════════════════════════════════════════════════╣");
    Serial.println("║  move <deg>       : Move motor to absolute angle (e.g. move 90)║");
    Serial.println("║  rel <steps>      : Move relative steps (e.g. rel 3200)        ║");
    Serial.println("║  loop on|off      : Enable or disable Closed-Loop PID          ║");
    Serial.println("║  inv              : Invert motor direction (fixes reverse/shake)║");
    Serial.println("║  rec start        : Record 5 sec of hand encoder motion        ║");
    Serial.println("║  rec play [spd]   : Replay recorded trajectory (e.g. rec play) ║");
    Serial.println("║  wifi on|off      : Toggle WiFi Access Point & Web Dashboard   ║");
    Serial.println("║  can leader       : Become CAN bus master broadcaster          ║");
    Serial.println("║  can follow <id>  : Follow another BoxDrive over CAN           ║");
    Serial.println("║  mode silent|torque : Switch StealthChop vs SpreadCycle        ║");
    Serial.println("║  curr <mA>        : Adjust motor current (100–2000 mA)         ║");
    Serial.println("║  status           : Print real-time status and telemetry       ║");
    Serial.println("║  diag             : Print hardware diagnostics report          ║");
    Serial.println("║  stop             : Emergency stop and hold                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
}

void printStatus() {
    BoxDriveStatus st = motor.getStatus();
    Serial.println("\n--- [BOXDRIVE LIVE STATUS REPORT] ---");
    Serial.printf("  Driver Enabled   : %s\n", st.motorEnabled ? "YES" : "NO");
    Serial.printf("  Direction Invert : %s\n", motor.isDirectionInverted() ? "INVERTED" : "NORMAL");
    Serial.printf("  Driver Mode      : %s\n", motor.getMode() == BoxDriveMode::STEALTH_CHOP ? "StealthChop (Silent)" : "SpreadCycle (High Torque)");
    Serial.printf("  Run Current      : %u mA\n", motor.getCurrent());
    Serial.printf("  Encoder Angle    : %.2f°\n", motor.encoder.readAngleDeg());
    Serial.printf("  Multi-turn Angle : %.2f°\n", motor.encoder.readMultiTurnDeg());
    Serial.printf("  Angular Velocity : %.1f °/s\n", motor.encoder.getVelocityDPS());
    Serial.printf("  Encoder AGC Gain : %u (Magnet %s)\n", motor.encoder.readAGC(), motor.encoder.magnetDetected() ? "Detected" : "MISSING!");
    Serial.printf("  Closed-Loop PID  : %s\n", motor.isClosedLoopEnabled() ? "ACTIVE" : "OFF");
    if (motor.isClosedLoopEnabled()) {
        Serial.printf("  Target Angle     : %.2f° (Error: %.2f°)\n", motor.getTargetAngle(), motor.getPositionError());
    }
    Serial.printf("  WiFi Hotspot     : %s\n", motor.isHotspotActive() ? "ACTIVE (SSID: BoxDrive, IP: 192.168.4.1)" : "OFF");
    Serial.printf("  CAN Bus Role     : %s\n", motor.getCANRole() == BoxDriveCANRole::LEADER ? "LEADER" : (motor.getCANRole() == BoxDriveCANRole::FOLLOWER ? "FOLLOWER" : "STANDALONE"));
    Serial.printf("  Recorded Samples : %u / %u\n", motor.getRecordingSampleCount(), BD_MAX_RECORDING_SAMPLES);
    Serial.printf("  Driver Temp Warn : %s\n", st.overTemp ? "HIGH TEMPERATURE WARNING!" : "Normal");
    Serial.println("-------------------------------------");
}

void processCliCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        printHelp();
    } else if (cmd.equalsIgnoreCase("status")) {
        printStatus();
    } else if (cmd.startsWith("move ")) {
        float angle = cmd.substring(5).toFloat();
        Serial.printf("[CLI] Moving to absolute angle %.2f°\n", angle);
        if (motor.isClosedLoopEnabled()) {
            motor.setTargetAngle(angle);
        } else {
            motor.moveToAngle(angle);
        }
    } else if (cmd.startsWith("rel ")) {
        int32_t steps = cmd.substring(4).toInt();
        Serial.printf("[CLI] Moving %ld relative steps\n", (long)steps);
        motor.move(steps);
    } else if (cmd.equalsIgnoreCase("inv")) {
        motor.invertDirection(!motor.isDirectionInverted());
        Serial.printf("[CLI] Stepper direction inverted: %s\n", motor.isDirectionInverted() ? "YES" : "NO");
    } else if (cmd.equalsIgnoreCase("loop on")) {
        Serial.println("[CLI] Activating smooth Closed-Loop PID position control...");
        motor.enableClosedLoop(1.2f, 0.005f, 0.35f);
        motor.setLED(BoxDriveLED::PULSE);
    } else if (cmd.equalsIgnoreCase("loop off")) {
        Serial.println("[CLI] Disabling Closed-Loop PID control.");
        motor.disableClosedLoop();
        motor.setLED(BoxDriveLED::BLINK_SLOW);
    } else if (cmd.equalsIgnoreCase("rec start")) {
        Serial.println("[CLI] Starting 5-second teaching recording...");
        Serial.println("      Rotate motor shaft by hand now!");
        motor.startRecording(5000);
        motor.setLED(BoxDriveLED::BLINK_FAST);
    } else if (cmd.startsWith("rec play")) {
        float spd = 1.0f;
        if (cmd.length() > 8) {
            spd = cmd.substring(8).toFloat();
            if (spd <= 0.0f) spd = 1.0f;
        }
        Serial.printf("[CLI] Replaying recorded trajectory at %.2fx speed...\n", spd);
        motor.playRecording(spd);
        motor.setLED(BoxDriveLED::BLINK_FAST);
    } else if (cmd.equalsIgnoreCase("wifi on")) {
        Serial.println("[CLI] Starting WiFi Hotspot 'BoxDrive' (192.168.4.1)...");
        motor.startHotspot("BoxDrive", "boxdrive1234");
    } else if (cmd.equalsIgnoreCase("wifi off")) {
        Serial.println("[CLI] Stopping WiFi Hotspot.");
        motor.stopHotspot();
    } else if (cmd.equalsIgnoreCase("can leader")) {
        Serial.println("[CLI] Node configured as CAN LEADER (Broadcasting encoder).");
        motor.can.begin(BD_CAN_SPEED_500K, 0x01);
        motor.setCANLeader();
    } else if (cmd.startsWith("can follow")) {
        uint8_t ldrId = cmd.substring(10).toInt();
        if (ldrId == 0) ldrId = 0x01;
        Serial.printf("[CLI] Node configured as CAN FOLLOWER (Mirroring Leader 0x%02X).\n", ldrId);
        motor.can.begin(BD_CAN_SPEED_500K, 0x02);
        motor.setCANFollower(ldrId);
    } else if (cmd.equalsIgnoreCase("mode silent")) {
        Serial.println("[CLI] Switched to StealthChop (Ultra-Quiet).");
        motor.setMode(BoxDriveMode::STEALTH_CHOP);
    } else if (cmd.equalsIgnoreCase("mode torque")) {
        Serial.println("[CLI] Switched to SpreadCycle (High Speed / Torque).");
        motor.setMode(BoxDriveMode::SPREAD_CYCLE);
    } else if (cmd.startsWith("curr ")) {
        uint16_t ma = cmd.substring(5).toInt();
        motor.setCurrent(ma);
        Serial.printf("[CLI] Motor run current set to %u mA\n", ma);
    } else if (cmd.equalsIgnoreCase("diag")) {
        motor.printDiagnostics();
    } else if (cmd.equalsIgnoreCase("stop")) {
        Serial.println("[CLI] Emergency STOP requested.");
        motor.stop();
        motor.setLED(BoxDriveLED::ERROR_FLASH);
    } else {
        Serial.println("[!] Unknown command. Type 'help' for command listing.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║             BOXDRIVE ALL-IN-ONE FLAGSHIP DEMO                 ║");
    Serial.println("║                    Hardware Revision v3.26                     ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // 1. Initialize BoxDrive
    if (!motor.begin()) {
        Serial.println("[-] Warning: Peripherals init error. Verify hardware!");
    } else {
        Serial.println("[✓] BoxDrive initialized successfully.");
    }

    // 2. Set default parameters
    motor.setCurrent(800, 200);
    motor.setSpeed(1200);
    motor.setAcceleration(2400);
    motor.setMode(BoxDriveMode::STEALTH_CHOP);
    motor.setMicrosteps(16);
    motor.enableMotor();

    // 3. Start WiFi Hotspot Dashboard
    motor.startHotspot("BoxDrive", "boxdrive1234");
    Serial.println("[+] WiFi Hotspot active: Connect to 'BoxDrive' -> Browse http://192.168.4.1");

    // 4. Initialize CAN bus at 500 kbps
    motor.can.begin(BD_CAN_SPEED_500K, 0x01);
    Serial.println("[+] CAN Bus controller initialized at 500 kbps (Node 0x01).");

    // 5. LED Ready state
    motor.setLED(BoxDriveLED::BLINK_SLOW);

    printHelp();
    Serial.println("[✓] System ready. Enter a command in Serial Monitor:");
}

void loop() {
    // 1. Check Serial CLI input
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            processCliCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }

    // 2. Closed-Loop PID State Machine
    if (motor.isClosedLoopEnabled()) {
        motor.updateClosedLoop();
    }

    // 3. Record / Playback State Machine
    if (motor.isRecording() || motor.isPlaying()) {
        motor.updateRecording();
    }

    // 4. CAN Bus Communication & Copy-Control Loop
    motor.updateCAN();

    // 5. WiFi Web Server Dashboard Request Handling
    if (motor.isHotspotActive()) {
        motor.handleHotspot();
    }

    // 6. LED Animation Engine
    motor.updateLED();

    delay(2);
}
