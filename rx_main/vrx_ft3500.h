#pragma once
#include "vrx.h"

class VRX_FT3500 : public VRX {
public:
  VRX_FT3500(
    int cs1, int cs2, int cs3,
    int s1,  int s2,  int s3
  ) : CS1(cs1), CS2(cs2), CS3(cs3),
      S1(s1),  S2(s2),  S3(s3) {}

  void begin() override {
    pinMode(CS1, OUTPUT); pinMode(CS2, OUTPUT); pinMode(CS3, OUTPUT);
    pinMode(S1,  OUTPUT); pinMode(S2,  OUTPUT); pinMode(S3,  OUTPUT);
    active = true;
  }

  void loop() override {}

  bool isActive() override {
    return active;
  }

  void setChannel(uint8_t idx) override {
    // idx: 0..63 (8 bands x 8 channels)

    uint8_t band = idx / 8;   // 0..7
    uint8_t ch   = idx % 8;   // 0..7

    // Band → CS (explicit HIGH/LOW)
    digitalWrite(CS1, (band & 0x01) ? HIGH : LOW);
    digitalWrite(CS2, (band & 0x02) ? HIGH : LOW);
    digitalWrite(CS3, (band & 0x04) ? HIGH : LOW);

    // Channel → S (explicit HIGH/LOW)
    digitalWrite(S1, (ch & 0x01) ? HIGH : LOW);
    digitalWrite(S2, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(S3, (ch & 0x04) ? HIGH : LOW);
  }

private:
  int CS1, CS2, CS3;
  int S1, S2, S3;
  bool active = false;
};
