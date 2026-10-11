/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 09: WiFi Hotspot Motor Control Dashboard                          ║
 * ║  BoxDrive Library                                                          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Creates a WiFi Access Point with a beautiful web dashboard to control     ║
 * ║  the motor from any phone, tablet, or laptop browser.                      ║
 * ║                                                                            ║
 * ║  HOW TO USE:                                                               ║
 * ║   1. Upload this sketch to BoxDrive.                                       ║
 * ║   2. On your phone/laptop, connect to WiFi network "BoxDrive".             ║
 * ║   3. Password: "boxdrive1234"                                              ║
 * ║   4. Open http://192.168.4.1 in your browser.                              ║
 * ║   5. Use the web dashboard to control the motor!                           ║
 * ║                                                                            ║
 * ║  DASHBOARD FEATURES:                                                       ║
 * ║   • Live encoder angle & step position readout                             ║
 * ║   • Enable/Disable motor                                                   ║
 * ║   • Move to specific angle or step count                                   ║
 * ║   • Adjust current, speed, and microstepping                               ║
 * ║   • Switch between StealthChop and SpreadCycle                             ║
 * ║   • Emergency stop button                                                  ║
 * ║                                                                            ║
 * ║  API ENDPOINTS (for custom integrations):                                  ║
 * ║   GET  /api/status            → JSON with all motor data                  ║
 * ║   POST /api/enable            → Enable motor                              ║
 * ║   POST /api/disable           → Disable motor                             ║
 * ║   GET  /api/move?angle=90     → Move to 90°                               ║
 * ║   GET  /api/move?steps=1600   → Move 1600 steps                           ║
 * ║   GET  /api/current?ma=800    → Set current to 800 mA                     ║
 * ║   GET  /api/speed?sps=2000    → Set speed to 2000 steps/sec               ║
 * ║   GET  /api/microsteps?ms=32  → Set microstepping to 1/32                 ║
 * ║   GET  /api/mode?m=stealth    → Set StealthChop mode                      ║
 * ║   GET  /api/mode?m=spread     → Set SpreadCycle mode                      ║
 * ║   POST /api/stop              → Emergency stop                            ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>

BoxDrive motor;

// Customize your WiFi AP settings
const char* WIFI_SSID     = "BoxDrive";
const char* WIFI_PASSWORD = "boxdrive1234"; // Min 8 characters

void setup() {
    motor.begin();
    motor.setCurrent(800, 400);
    motor.setMicrosteps(16);
    motor.setSpeed(1000);
    motor.setAcceleration(2000);
    motor.enableMotor();

    // Start WiFi Access Point with web control dashboard
    motor.startHotspot(WIFI_SSID, WIFI_PASSWORD);

    // Set LED to pulsing pattern to indicate WiFi is active
    motor.setLED(BoxDriveLED::PULSE);

    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║    WiFi HOTSPOT MOTOR CONTROL           ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.print("║  SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("║  Pass: ");
    Serial.println(WIFI_PASSWORD);
    Serial.println("║  URL:  http://192.168.4.1               ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Connect to the WiFi and open the URL in your browser.");
}

void loop() {
    // CRITICAL: Must call these every loop iteration!
    motor.handleHotspot();     // Handle incoming HTTP requests
    motor.updateRecording();   // Update hand kinesthetic motion recording & replay
    motor.updateClosedLoop();  // Update closed-loop if active
    motor.updateLED();         // Update LED animation

    // Also handle serial commands for debugging
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'd': motor.printDiagnostics(); break;
            case 's': motor.stopHotspot(); Serial.println("Hotspot stopped."); break;
            case 'r': motor.startHotspot(WIFI_SSID, WIFI_PASSWORD); break;
        }
    }
}
