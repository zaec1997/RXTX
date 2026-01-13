#pragma once
#include <Arduino.h>
#include "vrx.h"

// Set to 1 to enable detailed frame logging
#ifndef ENABLE_SVX_FRAME_LOG
#define ENABLE_SVX_FRAME_LOG 0
#endif

/*
  VRX_SteadyView
  ---------------
  - UART control (1-в-1 як у окулярів)
  - постійний poll
  - ACTIVE detection
  - resend last channel after ACTIVE (resync)
  - Frame assembly and B3 status decode
  - RSSI extraction (A and B channels in centi-dBm)
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
    
    // Send initial wake polls
    for (int i = 0; i < 3; i++) {
      serial.write(pollPkt, 6);
      serial.flush();
      delay(50);
    }
  }

  // --------------------
  // Main loop
  // --------------------
  void loop() override {

    // ---- poll ----
    if (millis() - lastPoll > 300) {
      serial.write(pollPkt, 6);
      serial.flush();
      lastPoll = millis();
    }

    // ---- RX ----
    while (serial.available()) {
      uint8_t b = serial.read();

      // Frame assembly: collect bytes between 0x02 and 0x03
      if (b == 0x02) {
        frameLen = 0;
        inFrame = true;
      } else if (b == 0x03 && inFrame) {
        inFrame = false;
        // Process complete frame
        processFrame();
      } else if (inFrame && frameLen < sizeof(frameBuf)) {
        frameBuf[frameLen++] = b;
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
    needResync = false;
    
    // Always send immediately (blind/optimistic behavior)
    sendSet(idx);
  }

  // --------------------
  // RSSI getters
  // --------------------
  bool hasRssi() const {
    return rssiValid;
  }

  float getLastRssiDbmA() const {
    return lastRssiAcenti / 100.0f;
  }

  float getLastRssiDbmB() const {
    return lastRssiBcenti / 100.0f;
  }

  bool consumeRssiUpdated() {
    bool ret = rssiUpdated;
    rssiUpdated = false;
    return ret;
  }

private:
  // --------------------
  // Frame processing
  // --------------------
  void processFrame() {
    if (frameLen < 1) return;

    uint8_t cmd = frameBuf[0];

    // B3 status frame
    if (cmd == 0xB3) {
      if (!active) {
        active = true;
#if ENABLE_SVX_FRAME_LOG
        Serial.println("[SVX] ACTIVE detected");
#endif
        // resync last channel after ACTIVE
        if (needResync) {
          sendSet(lastIdx);
          needResync = false;
        }
      }

      // Decode RSSI if frame is long enough
      // Expected format: 0x06 B3 ... <rssiA_lo> <rssiA_hi> <rssiB_lo> <rssiB_hi> ... checksum
      // RSSI values are typically at bytes 3-6 (indices 2-5 in frameBuf after cmd)
      if (frameLen >= 6) {
        // Extract RSSI A (signed int16 LE)
        int16_t rssiA = (int16_t)((frameBuf[3] << 8) | frameBuf[2]);
        // Extract RSSI B (signed int16 LE)
        int16_t rssiB = (int16_t)((frameBuf[5] << 8) | frameBuf[4]);

        lastRssiAcenti = rssiA;
        lastRssiBcenti = rssiB;
        rssiValid = true;
        rssiUpdated = true;

#if ENABLE_SVX_FRAME_LOG
        Serial.print("[SVX] RSSI A: ");
        Serial.print(getLastRssiDbmA());
        Serial.print(" dBm, B: ");
        Serial.print(getLastRssiDbmB());
        Serial.println(" dBm");
#endif
      }
    }

#if ENABLE_SVX_FRAME_LOG
    // Log frame in concise format
    Serial.print("[SVX] Frame[");
    Serial.print(frameLen);
    Serial.print("]: ");
    for (int i = 0; i < frameLen && i < 16; i++) {
      if (frameBuf[i] < 0x10) Serial.print("0");
      Serial.print(frameBuf[i], HEX);
      Serial.print(" ");
    }
    if (frameLen > 16) Serial.print("...");
    Serial.println();
#endif
  }

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

#if ENABLE_SVX_FRAME_LOG
    Serial.print("[SVX] SET channel ");
    Serial.println(idx);
#endif
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

  // Frame assembly
  bool inFrame = false;
  uint8_t frameBuf[64];
  uint8_t frameLen = 0;

  // RSSI state
  int16_t lastRssiAcenti = 0;
  int16_t lastRssiBcenti = 0;
  bool rssiValid = false;
  bool rssiUpdated = false;
};
