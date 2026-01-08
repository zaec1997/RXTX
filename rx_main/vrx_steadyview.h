#pragma once
#include <Arduino.h>
#include "vrx.h"

/*
  VRX_SteadyView
  ---------------
  - UART control
  - постійний poll
  - ACTIVE detection + timeout
  - resend last channel after ACTIVE (resync)
*/

class VRX_SteadyView : public VRX {
public:
  VRX_SteadyView(HardwareSerial &s, int rxPin, int txPin)
    : serial(s), rx(rxPin), tx(txPin) {}

  void begin() override {
    serial.begin(115200, SERIAL_8N1, rx, tx);
    lastPoll = millis();
    active = false;
    needResync = false;
    lastActiveSeen = 0;
  }

  void loop() override {
    // ---- check active timeout ----
    if (active && lastActiveSeen != 0 && (millis() - lastActiveSeen > ACTIVE_TIMEOUT_MS)) {
      active = false;
      Serial.println("[SVX] ACTIVE lost");
    }

    // ---- poll ----
    if (millis() - lastPoll > 820) {
      serial.write(pollPkt, 6);
      serial.flush();
      lastPoll = millis();
    }

    // ---- RX ----
    while (serial.available()) {
      uint8_t b = serial.read();

      // ACTIVE frame detected
      if (b == 0xB3) {
        lastActiveSeen = millis();
        if (!active) {
          active = true;
          Serial.println("[SVX] ACTIVE detected");
          // resync last channel
          if (needResync) {
            sendSet(lastIdx);
            needResync = false;
          }
        }
      }
    }
  }

  bool isActive() override {
    return active;
  }

  void setChannel(uint8_t idx) override {
    lastIdx = idx;

    if (!active) {
      // remember and send later
      needResync = true;
      Serial.println("[SVX] channel saved for resync");
      return;
    }

    sendSet(idx);
  }

private:
  void sendSet(uint8_t idx) {
    uint8_t pkt[6];
    pkt[0] = 0x02;
    pkt[1] = 0x06;
    pkt[2] = 0x31;
    pkt[3] = idx;
    pkt[4] = pkt[1] ^ pkt[2] ^ pkt[3];
    pkt[5] = 0x03;

    // exactly as goggles do
    for (int i = 0; i < 3; i++) {
      serial.write(pkt, 6);
      serial.flush();
      delayMicroseconds(1000);
    }
  }

private:
  HardwareSerial &serial;
  int rx, tx;

  bool active = false;
  bool needResync = false;
  uint8_t lastIdx = 0;

  unsigned long lastPoll = 0;
  unsigned long lastActiveSeen = 0;
  // Timeout value tuned for SteadyView X hardware polling rate (820ms)
  // Should be > 2x poll interval to avoid false timeouts
  static constexpr unsigned long ACTIVE_TIMEOUT_MS = 2000;

  const uint8_t pollPkt[6] = {
    0x02, 0x06, 0x33, 0x80, 0xB5, 0x03
  };
};
