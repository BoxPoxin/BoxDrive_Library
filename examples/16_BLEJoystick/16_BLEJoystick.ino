/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  EXAMPLE 16: BLE (Bluetooth Low Energy) Remote Control                     ║
 * ║  BoxDrive Library — Smartphone / Gamepad Wireless Stepper Control          ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  Turns the ESP32-C6 on BoxDrive into a BLE GATT peripheral.                ║
 * ║  Control motor velocity, direction, and position directly from any phone   ║
 * ║  using apps like 'nRF Connect', 'Serial Bluetooth Terminal', or Web-BLE!   ║
 * ║                                                                            ║
 * ║  BLE GATT PROFILE:                                                         ║
 * ║   • Service UUID : 4fafc201-1fb5-459e-8fcc-c5c9c331914b                    ║
 * ║   • Command Char : beb5483e-36e1-4688-b7f5-ea07361b26a8 (Write)            ║
 * ║   • Telemetry Char: 1c93d406-4ff5-4847-9c5e-f6da8f773418 (Read / Notify)   ║
 * ║                                                                            ║
 * ║  SUPPORTED COMMANDS (Sent as ASCII text to Command Char):                  ║
 * ║   • "MOVE <steps>"   -> Moves specified steps (e.g. "MOVE 3200")           ║
 * ║   • "ANGLE <deg>"    -> Moves to absolute angle (e.g. "ANGLE 180")         ║
 * ║   • "SPEED <steps/s>"-> Sets speed (e.g. "SPEED 1500")                     ║
 * ║   • "STOP"           -> Halts motor immediately                            ║
 * ║   • "ENABLE"         -> Energizes motor coils                              ║
 * ║   • "DISABLE"        -> De-energizes motor coils                           ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include <BoxDrive.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_CMD     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TELEM   "1c93d406-4ff5-4847-9c5e-f6da8f773418"

BoxDrive motor;

BLEServer*         pServer         = nullptr;
BLECharacteristic* pCmdChar        = nullptr;
BLECharacteristic* pTelemChar      = nullptr;
bool               deviceConnected = false;
bool               oldDeviceConnected = false;

// Pending command buffer
String pendingCmd = "";
bool hasPendingCmd = false;

// BLE Server callbacks for connect/disconnect events
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("[BLE] >>> Client Connected! <<<");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("[BLE] Client Disconnected. Re-advertising...");
    }
};

// BLE Characteristic write callbacks for receiving commands
class CmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            pendingCmd = value;
            hasPendingCmd = true;
        }
    }
};

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("  BoxDrive Example 16: BLE 5.0 Wireless Control   ");
    Serial.println("==================================================");

    // 1. Initialize motor
    motor.begin();
    motor.setCurrent(800);
    motor.setHoldCurrent(250);
    motor.setSpeed(1200);
    motor.setAcceleration(2400);
    motor.enableMotor();

    // 2. Initialize BLE subsystem
    BLEDevice::init("BoxDrive-BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 3. Create GATT Service
    BLEService* pService = pServer->createService(SERVICE_UUID);

    // 4. Create Command Characteristic (Write)
    pCmdChar = pService->createCharacteristic(
        CHARACTERISTIC_CMD,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCmdChar->setCallbacks(new CmdCallbacks());

    // 5. Create Telemetry Characteristic (Read & Notify)
    pTelemChar = pService->createCharacteristic(
        CHARACTERISTIC_TELEM,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pTelemChar->addDescriptor(new BLE2902());

    // 6. Start service & advertising
    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // iPhone connection optimization
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    motor.setLED(BoxDriveLED::PULSE);
    Serial.println("[+] BLE Advertising started as 'BoxDrive-BLE'.");
    Serial.println("[+] Open 'nRF Connect' or any BLE scanner app to connect.");
    Serial.println("--------------------------------------------------");
}

void processCommand(const String& cmd) {
    Serial.print("[BLE Command] Received: ");
    Serial.println(cmd);

    if (cmd.startsWith("MOVE ")) {
        int32_t steps = cmd.substring(5).toInt();
        Serial.printf("→ Executing move %d steps\n", steps);
        motor.move(steps);
    } else if (cmd.startsWith("ANGLE ")) {
        float angle = cmd.substring(6).toFloat();
        Serial.printf("→ Moving to angle %.2f°\n", angle);
        motor.moveToAngle(angle);
    } else if (cmd.startsWith("SPEED ")) {
        float spd = cmd.substring(6).toFloat();
        Serial.printf("→ Setting speed to %.1f steps/s\n", spd);
        motor.setSpeed(spd);
    } else if (cmd.equalsIgnoreCase("STOP")) {
        Serial.println("→ Motor STOP");
        motor.stop();
    } else if (cmd.equalsIgnoreCase("ENABLE")) {
        Serial.println("→ Motor ENABLE");
        motor.enableMotor();
    } else if (cmd.equalsIgnoreCase("DISABLE")) {
        Serial.println("→ Motor DISABLE");
        motor.disableMotor();
    } else {
        Serial.println("[!] Unknown command syntax.");
    }
}

void loop() {
    // 1. Process any pending command from phone
    if (hasPendingCmd) {
        processCommand(pendingCmd);
        hasPendingCmd = false;
    }

    // 2. Stream telemetry notifications at 10 Hz when connected
    static uint32_t lastTelem = 0;
    if (deviceConnected && millis() - lastTelem > 100) {
        lastTelem = millis();

        float angle = motor.encoder.readAngleDeg();
        float vel   = motor.encoder.getVelocityDPS();
        int32_t pos = motor.getPosition();

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "ANG:%.1f,VEL:%.1f,POS:%ld", angle, vel, (long)pos);

        pTelemChar->setValue(buffer);
        pTelemChar->notify();
    }

    // 3. Handle reconnection state changes
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give bluetooth stack a chance to prepare
        pServer->startAdvertising();
        Serial.println("[BLE] Restarted advertising.");
        oldDeviceConnected = deviceConnected;
        motor.setLED(BoxDriveLED::PULSE);
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        motor.setLED(BoxDriveLED::BLINK_FAST);
    }

    motor.updateLED();
    delay(5);
}
