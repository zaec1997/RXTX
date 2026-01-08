#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "protocol.h"

class VrxController {
public:
  void begin(LoRaClass *l) { lora = l; }

  void setVrx(uint8_t id) {
    currentId = id;
    pendingSend = true;
  }

  void loop() {
    if (!lora) return;

    if (pendingSend && !waitingAck) {
      Packet pkt;
      pkt.cmd = CMD_SET_VRX;
      pkt.arg1 = currentId;
      pkt.arg2 = 0;
      pkt.crc = calcCRC(pkt);

      lora->idle();
      lora->beginPacket();
      lora->write((uint8_t*)&pkt, sizeof(pkt));
      lora->endPacket();
      lora->receive();

      waitingAck = true;
      pendingSend = false;
      waitStart = millis();

      Serial.print("[VRX] SEND id=");
      Serial.println(currentId);
    }

    if (waitingAck && millis() - waitStart > ACK_TIMEOUT) {
      waitingAck = false;
      pendingSend = true;
      Serial.println("[VRX] ACK TIMEOUT");
    }
  }

  void handleAck() {
    if (waitingAck) {
      waitingAck = false;
      Serial.println("[VRX] GOT ACK");
    }
  }

  bool isWaitingAck() const { return waitingAck; }

private:
  LoRaClass *lora = nullptr;
  uint8_t currentId = 0;
  bool pendingSend = false;
  bool waitingAck = false;
  unsigned long waitStart = 0;
  static constexpr unsigned long ACK_TIMEOUT = 800;
};
