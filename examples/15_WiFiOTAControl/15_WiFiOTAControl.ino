/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 15: WiFi OTA (Over-The-Air) & Remote Control                       ║
 * ║  BoxDrive Library — Wireless Firmware Upgrades for Sealed Enclosures       ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  This example demonstrates wireless firmware flashing (ArduinoOTA)         ║
 * ║  combined with real-time motor monitoring on ESP32-C6.                     ║
 * ║                                                                            ║
 * ║  USE CASE:                                                                 ║
 * ║   • BoxDrive installed inside robotic joints, 3D printers, or rovers       ║
 * ║     where plugging in a USB-C cable is impossible or inconvenient.         ║
 * ║   • Safely halts and disables the motor if an OTA update begins!           ║
 * ║                                                                            ║
 * ║  HOW TO FLASH OVER OTA:                                                    ║
 * ║   1. Connect BoxDrive to your local WiFi network (set credentials below).  ║
 * ║   2. In Arduino IDE: Tools -> Port -> Network Ports -> 'boxdrive-motor'    ║
 * ║   3. Hit Upload! The firmware uploads wirelessly in seconds.              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ── WiFi Configuration ──────────────────────────────────────────
// Set your local WiFi router credentials:
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Hostname visible in Arduino IDE Network Ports
const char* OTA_HOSTNAME = "boxdrive-joint-01";

BoxDrive motor;
bool isUpdatingOTA = false;
uint32_t lastTelemetryTime = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("  BoxDrive Example 15: WiFi OTA Wireless Updates  ");
    Serial.println("==================================================");

    // 1. Initialize BoxDrive hardware
    motor.begin();
    motor.setCurrent(800);
    motor.setHoldCurrent(200);
    motor.setSpeed(1000);
    motor.setAcceleration(2000);
    motor.enableMotor();
    motor.setLED(BoxDriveLED::BLINK_SLOW);

    // 2. Connect to WiFi
    Serial.print("[+] Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 12000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[✓] WiFi Connected!");
        Serial.print("    IP Address : ");
        Serial.println(WiFi.localIP());
        Serial.print("    RSSI Signal: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("\n[!] WiFi connection timed out. Falling back to Hotspot AP mode...");
        WiFi.disconnect();
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BoxDrive-OTA-AP", "boxdrive1234");
        Serial.print("    AP IP Address: ");
        Serial.println(WiFi.softAPIP());
    }

    // 3. Configure ArduinoOTA
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    // Optional: Protect with an upload password
    // ArduinoOTA.setPassword("boxdrive-secret");

    ArduinoOTA.onStart([]() {
        isUpdatingOTA = true;
        // CRITICAL SAFETY REQUIREMENT:
        // Disable motor and release coils so motor doesn't jerk or overheat during flash!
        motor.disableMotor();
        motor.setLED(BoxDriveLED::BLINK_FAST);
        Serial.println("\n[!] >>> OTA UPDATE STARTED: MOTOR DISABLED FOR SAFETY <<<");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[✓] OTA Update Complete! Rebooting ESP32-C6...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        unsigned int percent = (progress / (total / 100));
        Serial.printf("[OTA] Progress: %u%%\r", percent);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[!] OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        motor.setLED(BoxDriveLED::ERROR_FLASH);
    });

    ArduinoOTA.begin();
    Serial.println("[+] ArduinoOTA Service Active.");
    Serial.println("    You can now upload code wirelessly from Arduino IDE or PlatformIO!");
    Serial.println("--------------------------------------------------");
}

void loop() {
    // 1. Maintain OTA service
    ArduinoOTA.handle();

    // 2. If an OTA update is actively burning flash, yield immediately
    if (isUpdatingOTA) {
        delay(1);
        return;
    }

    // 3. Normal motor routine (e.g. periodic sweep)
    static uint32_t lastMoveTime = 0;
    static bool direction = true;

    if (millis() - lastMoveTime > 3000) {
        lastMoveTime = millis();
        if (direction) {
            motor.move(1600); // 180° CW
        } else {
            motor.move(-1600); // 180° CCW
        }
        direction = !direction;
    }

    // 4. Print status telemetry
    if (millis() - lastTelemetryTime > 1000) {
        lastTelemetryTime = millis();
        Serial.print("[Telemetry] Encoder: ");
        Serial.print(motor.encoder.readAngleDeg(), 2);
        Serial.print("° | Multi-Turn: ");
        Serial.print(motor.encoder.readMultiTurnDeg(), 1);
        Serial.print("° | WiFi RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    }

    motor.updateLED();
    delay(10);
}
