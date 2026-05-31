#include <Arduino.h>
#include "Bluetooth.h"
#include "MGServo.h"
#include "HRController.h"
#include "Params.h"

HardwareSerial RS485(1);
MGServo ServoLW(1, RS485);
MGServo ServoRW(2, RS485);
HRController HR;
WBRBluetooth ble;

void printState() {
    Serial.println("---");
    Serial.printf("isRun  : %s\n", ble.isRun() ? "true" : "false");
    Serial.printf("v_d    : %.3f\n", ble.getV());
    Serial.printf("h_d    : %.3f\n", ble.getH());
    Serial.printf("yaw_d  : %.3f\n", ble.getYaw());
    Serial.printf("phi_d  : %.3f\n", ble.getPhi());
}

void setup() {
    Serial.begin(115200);

    // RS485 최소 초기화 (MGServo 의존성)
    pinMode(RS485_DE_RE, OUTPUT);
    digitalWrite(RS485_DE_RE, LOW);
    RS485.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    // BLE 시작
    ble.begin("ESP32_WBR_BLE", &ServoLW, &ServoRW, &HR);
    Serial.println("BLE ready. Device: ESP32_WBR_BLE");
    Serial.println("Commands: run / stop / reset / v 0.3 / h 0.15 / yaw 0.2 / roll 0.1");
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last > 1000) {
        last = millis();
        printState();
    }
}