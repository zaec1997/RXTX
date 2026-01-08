#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "protocol.h"

class ChannelController {
public:
  void begin(LoRaClass *lora);

  // UI / encoder calls this
  void setChannel(uint8_t idx);

  // call every loop()
  void loop();

  bool isWaitingAck() const { return waitingAck; }
  uint8_t currentChannel() const { return currentIdx; }

private:
  void sendPacket();

  LoRaClass *lora = nullptr;

  uint8_t currentIdx = 0;
  bool pendingSend = false;
  bool waitingAck  = false;

  unsigned long waitStart = 0;
  static constexpr unsigned long ACK_TIMEOUT = 800;

  Packet pkt;
};
