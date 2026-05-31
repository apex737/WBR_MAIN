#include <Arduino.h>
#include <ArduinoEigen.h>
#include "HRController.h"
#include "Params.h"

HRController HR;

void setHip(float deg) {
    float rad = deg * M_PI / 180.0f;
    Eigen::Vector2f target;
    target << rad, rad;
    HR.controlHipServos(target);
    Serial.printf(">> %.1f deg → ", deg);
    HR.printStatus();
}

void setup() {
    Serial.begin(115200);
    HR.attachServos(LH_PIN, RH_PIN);
    Serial.println("ready. input: 0 / 55 / -55");
}

void loop() {
    if (!Serial.available()) return;
    float deg = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    setHip(deg);
}