#pragma once
#include <Arduino.h>
#include "vrx.h"

/*
  VRX_SteadyView
  ---------------
  - UART control (1-в-1 як у окулярів)
  - постійний poll
  - ACTIVE detection
  - resend last channel after ACTIVE (resync)
*/

class VRX_SteadyView : public VRX {
public:
  VRX_SteadyView(HardwareSerial &s, int rxPin, int txPin)
    : serial(s), rx(rxPin), tx(txPin) {}

  // --------------------
  // Init
  // --------------------
  void begin() override {
    serial.begin(115200, SERIAL_8N1, rx, tx);
    lastPoll = millis();
    active = false;
    needResync = false;
  }

  // --------------------
  // Main loop
  // --------------------
  void loop() override {

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
      if (b == 0xB3 && !active) {
        active = true;
        Serial.println("[VRX_SteadyView] ACTIVE detected");

        // resync last channel
        if (needResync) {
          Serial.print("[VRX_SteadyView] resync idx=");
          Serial.println(lastIdx);
          sendSet(lastIdx);
          needResync = false;
        }
      }
    }
  }

  // --------------------
  // State
  // --------------------
  bool isActive() override {
    return active;
  }

  // --------------------
  // Set channel
  // --------------------
  void setChannel(uint8_t idx) override {
    lastIdx = idx;

    if (!active) {
      // remember and send later
      needResync = true;
      Serial.print("[VRX_SteadyView] setChannel NOT ACTIVE, queue resync idx=");
      Serial.println(idx);
      return;
    }

    Serial.print("[VRX_SteadyView] setChannel ACTIVE, send idx=");
    Serial.println(idx);
    sendSet(idx);
  }

private:
  // --------------------
  // Low-level send
  // --------------------
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

  const uint8_t pollPkt[6] = {
    0x02, 0x06, 0x33, 0x80, 0xB5, 0x03
  };
};
