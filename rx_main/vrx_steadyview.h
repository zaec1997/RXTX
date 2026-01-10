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

      // State machine for B3 frame parsing
      if (rxState == RX_IDLE) {
        if (b == 0xB3) {
          rxState = RX_HEADER;
          rxBuf[0] = b;
          rxPos = 1;
          
          // Mark as active when B3 frame detected
          if (!active) {
            active = true;
            
            // resync last channel
            if (needResync) {
              sendSet(lastIdx);
              needResync = false;
            }
          }
        }
      } else if (rxState == RX_HEADER) {
        rxBuf[rxPos++] = b;
        
        // B3 frame structure: B3 + payload length + data...
        // Assuming frame format: 0xB3 [len] [data...] [checksum]
        // For RSSI, we need at least: 0xB3 + len + RSSI1(2 bytes) + RSSI2(2 bytes) + ...
        if (rxPos >= 2) {
          rxLen = rxBuf[1];  // length byte
          if (rxLen > 0 && rxLen < 64) {
            rxState = RX_DATA;
          } else {
            rxState = RX_IDLE;
          }
        }
      } else if (rxState == RX_DATA) {
        rxBuf[rxPos++] = b;
        
        // Check if we have complete frame (header + len + data)
        if (rxPos >= (2 + rxLen)) {
          // Parse complete B3 frame
          parseB3Frame();
          rxState = RX_IDLE;
        }
        
        // Safety: prevent buffer overflow
        if (rxPos >= 64) {
          rxState = RX_IDLE;
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
  // RSSI getters (in centi-dBm)
  // --------------------
  int16_t getRSSI1() {
    return rssi1_cdBm;
  }

  int16_t getRSSI2() {
    return rssi2_cdBm;
  }

  // --------------------
  // Set channel
  // --------------------
  void setChannel(uint8_t idx) override {
    lastIdx = idx;

    // Always send SET command immediately ("blind" send)
    sendSet(idx);

    // If not active yet, mark for resync when ACTIVE detected
    if (!active) {
      needResync = true;
    }
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

  // --------------------
  // Parse B3 frame and extract RSSI
  // --------------------
  void parseB3Frame() {
    // B3 frame structure (based on typical VRX protocol):
    // [0] = 0xB3 (header)
    // [1] = length
    // [2..] = data
    // Data typically contains RSSI values for antenna 1 and antenna 2
    // RSSI format: signed 16-bit little-endian in centi-dBm
    
    if (rxPos < 6) return; // Need at least: B3 + len + 2*RSSI (2 bytes each)
    
    // Extract RSSI values (assuming they start at byte 2)
    // RSSI1: bytes [2-3], RSSI2: bytes [4-5]
    rssi1_cdBm = (int16_t)(rxBuf[2] | (rxBuf[3] << 8));  // Little Endian
    rssi2_cdBm = (int16_t)(rxBuf[4] | (rxBuf[5] << 8));  // Little Endian
  }

private:
  HardwareSerial &serial;
  int rx, tx;

  bool active = false;
  bool needResync = false;
  uint8_t lastIdx = 0;

  unsigned long lastPoll = 0;

  // RSSI values in centi-dBm (signed int16, little-endian)
  int16_t rssi1_cdBm = 0;
  int16_t rssi2_cdBm = 0;

  // RX state machine for B3 frame parsing
  enum RxState {
    RX_IDLE,
    RX_HEADER,
    RX_DATA
  };
  RxState rxState = RX_IDLE;
  uint8_t rxBuf[64];
  uint8_t rxPos = 0;
  uint8_t rxLen = 0;

  const uint8_t pollPkt[6] = {
    0x02, 0x06, 0x33, 0x80, 0xB5, 0x03
  };
};
