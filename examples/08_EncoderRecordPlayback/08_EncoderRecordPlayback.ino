/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 08: Encoder Record & Playback                                     ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  RECORD the encoder position for 5 seconds while you manually turn         ║
 * ║  the motor shaft, then PLAY it back — the motor replicates your exact      ║
 * ║  motion automatically!                                                     ║
 * ║                                                                            ║
 * ║  HOW IT WORKS:                                                             ║
 * ║   1. Press 'r' in Serial Monitor to START recording.                       ║
 * ║   2. Manually rotate the motor shaft for up to 5 seconds.                  ║
 * ║   3. Recording stops automatically after 5 seconds.                        ║
 * ║   4. Press 'p' to PLAY BACK the recorded motion.                           ║
 * ║   5. Press 'f' to play at 2x speed, 's' for 0.5x speed.                   ║
 * ║                                                                            ║
 * ║  USE CASES:                                                                ║
 * ║   • Teach-and-repeat for robotic arms                                      ║
 * ║   • Recording camera movements for repeatable shots                        ║
 * ║   • Programming motion paths by demonstration                              ║
 * ║   • Art installations with recorded gestures                               ║
 * ║                                                                            ║
 * ║  TECHNICAL DETAILS:                                                        ║
 * ║   • Samples at 1 kHz (1 ms intervals)                                      ║
 * ║   • Max 5000 samples = 5 seconds of recording                             ║
 * ║   • Playback uses closed-loop PID for accurate reproduction                ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

void setup() {
    motor.begin();
    motor.setCurrent(800, 400);
    motor.setMicrosteps(16);
    motor.enableMotor();

    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   ENCODER RECORD & PLAYBACK DEMO        ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.println("║  Commands (type in Serial Monitor):      ║");
    Serial.println("║   r = Start recording (5 sec)            ║");
    Serial.println("║   p = Play back at 1x speed              ║");
    Serial.println("║   f = Play back at 2x speed              ║");
    Serial.println("║   s = Play back at 0.5x speed            ║");
    Serial.println("║   x = Stop playback                      ║");
    Serial.println("║   d = Dump recording data                ║");
    Serial.println("║   i = Print info                         ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
}

void loop() {
    // CRITICAL: Must call this every loop for recording & playback to work
    motor.updateRecording();

    // Also update closed-loop if active (needed for playback)
    if (motor.isClosedLoopEnabled()) {
        motor.updateClosedLoop();
    }

    // Handle Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'r': case 'R':
                // Disable motor so user can freely rotate the shaft
                motor.disableMotor();
                delay(100);

                Serial.println("\n▶ RECORDING STARTED — Turn the shaft now!");
                Serial.println("  Recording will stop after 5 seconds...");
                motor.startRecording(5000); // Record for 5 seconds
                break;

            case 'p': case 'P':
                Serial.println("\n▶ PLAYING BACK at 1.0x speed...");
                motor.enableMotor();
                delay(100);
                motor.playRecording(1.0);
                break;

            case 'f': case 'F':
                Serial.println("\n▶ PLAYING BACK at 2.0x speed (fast)...");
                motor.enableMotor();
                delay(100);
                motor.playRecording(2.0);
                break;

            case 's': case 'S':
                Serial.println("\n▶ PLAYING BACK at 0.5x speed (slow)...");
                motor.enableMotor();
                delay(100);
                motor.playRecording(0.5);
                break;

            case 'x': case 'X':
                motor.stopPlayback();
                motor.stop();
                Serial.println("\n■ Playback stopped.");
                break;

            case 't': case 'T':
                motor.invertDirection(!motor.isDirectionInverted());
                Serial.print("\nDirection inverted: ");
                Serial.println(motor.isDirectionInverted() ? "YES" : "NO");
                break;

            case 'd': case 'D':
                dumpRecording();
                break;

            case 'i': case 'I':
                printInfo();
                break;
        }
    }

    // Status updates
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();

        if (motor.isRecording()) {
            Serial.print("  ● REC  | Samples: ");
            Serial.print(motor.getRecordingSampleCount());
            Serial.print(" | Angle: ");
            Serial.print(motor.encoder.readAngleDeg(), 1);
            Serial.println("°");
        }

        if (motor.isPlaying()) {
            Serial.print("  ▶ PLAY | Angle: ");
            Serial.print(motor.encoder.readAngleDeg(), 1);
            Serial.print("° → Target: ");
            Serial.print(motor.getTargetAngle(), 1);
            Serial.println("°");
        }
    }
}

void dumpRecording() {
    uint16_t count;
    const MotionSample* buffer = motor.getRecordingBuffer(count);

    Serial.print("\nRecording data (");
    Serial.print(count);
    Serial.println(" samples):");
    Serial.println("Time(ms)\tAngle(°)");
    Serial.println("--------\t--------");

    for (uint16_t i = 0; i < count; i += 10) { // Print every 10th sample
        Serial.print(buffer[i].timestampMs);
        Serial.print("\t\t");
        Serial.println(buffer[i].angleDeg, 2);
    }
    Serial.println();
}

void printInfo() {
    Serial.println("\n── Status ──");
    Serial.print("  Recording: ");
    Serial.println(motor.isRecording() ? "YES" : "No");
    Serial.print("  Playing: ");
    Serial.println(motor.isPlaying() ? "YES" : "No");
    Serial.print("  Samples stored: ");
    Serial.println(motor.getRecordingSampleCount());
    Serial.print("  Motor enabled: ");
    Serial.println(motor.isMotorEnabled() ? "YES" : "No");
    Serial.print("  Encoder angle: ");
    Serial.print(motor.encoder.readAngleDeg(), 2);
    Serial.println("°");
    Serial.println();
}
