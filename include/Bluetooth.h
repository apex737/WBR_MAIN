#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MGServo.h"
#include "HRController.h"

// ===== BLE Configuration =====
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class WBRBluetooth
{
private:
  BLECharacteristic *pTxCharacteristic = nullptr;
  bool connected = false;

  // 하드웨어 주소 보관 (비상 정지 및 직접 제어용)
  MGServo *sLW, *sRW;
  HRController *hr;

  // 제어 상태 변수
  bool _isRun = false;
  bool _isReset = false;
  float _v_d = 0.0f;
  float _dpsi_d = 0.0f;
  float _phi_d = 0.0f;
  float _h_d = 0.18f; // Params.h의 HEIGHT_MAX 등 기본값에 맞춤

public:
  void begin(const char *name, MGServo *lw, MGServo *rw, HRController *h)
  {
    sLW = lw;
    sRW = rw;
    hr = h;

    BLEDevice::init(name);
    BLEServer *pServer = BLEDevice::createServer();
    class ServerCB : public BLEServerCallbacks
    {
      WBRBluetooth *p;

    public:
      ServerCB(WBRBluetooth *_p) : p(_p) {}
      void onConnect(BLEServer *s) { p->connected = true; }
      void onDisconnect(BLEServer *s)
      {
        p->connected = false;
        BLEDevice::startAdvertising();
      }
    };
    pServer->setCallbacks(new ServerCB(this));

    BLEService *pSvc = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pSvc->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    class RxCB : public BLECharacteristicCallbacks
    {
      WBRBluetooth *p;

    public:
      RxCB(WBRBluetooth *_p) : p(_p) {}
      void onWrite(BLECharacteristic *c) { p->handle(c->getValue().c_str()); }
    };
    BLECharacteristic *pRx = pSvc->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRx->setCallbacks(new RxCB(this));

    pSvc->start();
    pServer->getAdvertising()->start();
  }

  void handle(String cmd)
  {
    cmd.trim();
    if (cmd.length() == 0)
      return;

    // 1. 상태 및 목표값 업데이트 (Main Loop의 Getter에서 참조)
    if (cmd == "run")
    {
      _isRun = true;
      notify("[MODE] RUN");
    }
    else if (cmd == "stop")
    {
      _isRun = false;
      sLW->sendTorqueControlCommand(0);
      sRW->sendTorqueControlCommand(0);
      notify("[MODE] STOP & EMERGENCY BRAKE");
    }
    else if (cmd == "reset")
    {
      _isReset = true;
      notify("[SYSTEM] Estimator Reset");
    }
    else if (cmd.startsWith("v "))
    {
      _v_d = cmd.substring(2).toFloat();
    }
    else if (cmd.startsWith("h "))
    {
      _h_d = cmd.substring(2).toFloat();
    }
    else if (cmd.startsWith("yaw "))
    {
      _dpsi_d = cmd.substring(4).toFloat();
    }
    else if (cmd.startsWith("roll "))
    {
      _phi_d = cmd.substring(5).toFloat(); // Roll 추가
    }

    // 2. 직접 하드웨어 명령 (테스트용)
    char side[10] = {0};
    char mode;
    int val;
    if (sscanf(cmd.c_str(), "%s %c %d", side, &mode, &val) == 3)
    {
      String s = String(side);
      if (s == "lw" || s == "both")
      {
        if (mode == 't')
          sLW->sendTorqueControlCommand(val);
        else if (mode == 's')
          sLW->sendSpeedControlCommand(val);
      }
      if (s == "rw" || s == "both")
      {
        if (mode == 't')
          sRW->sendTorqueControlCommand(val);
        else if (mode == 's')
          sRW->sendSpeedControlCommand(val);
      }
    }
  }

  void notify(String m)
  {
    if (connected)
    {
      pTxCharacteristic->setValue(m.c_str());
      pTxCharacteristic->notify();
    }
  }

  // === Getter 함수들 ===
  bool isRun() const { return _isRun; }
  bool isReset()
  {
    bool r = _isReset;
    _isReset = false;
    return r;
  }
  float getV() const { return _v_d; }
  float getYaw() const { return _dpsi_d; }
  float getH() const { return _h_d; }
  float getPhi() const { return _phi_d; }
};
#endif