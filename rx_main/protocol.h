#pragma once
#include <Arduino.h>

enum CmdType : uint8_t {
  CMD_SET_CHANNEL = 0x01,
  CMD_SET_VRX     = 0x02,
  CMD_PING        = 0x03,
  CMD_ACK         = 0x80
};

struct Packet {
  uint8_t cmd;
  uint8_t arg1;
  uint8_t arg2;
  uint8_t crc;
};

inline uint8_t calcCRC(const Packet &p) {
  return p.cmd ^ p.arg1 ^ p.arg2;
}

inline bool checkCRC(const Packet &p) {
  return p.crc == calcCRC(p);
}
