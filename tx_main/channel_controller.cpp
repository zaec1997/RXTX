#include "channel_controller.h"

void ChannelController::begin(LoRaClass *l) {
  lora = l;
}

void ChannelController::setChannel(uint8_t idx) {
  currentIdx = idx;
  pendingSend = true;
}

void ChannelController::sendPacket() {
  pkt.cmd  = CMD_SET_CHANNEL;
  pkt.arg1 = currentIdx;
  pkt.arg2 = 0;
  pkt.crc  = calcCRC(pkt);

  lora->idle();
  lora->beginPacket();
  lora->write((uint8_t*)&pkt, sizeof(pkt));
  lora->endPacket();
  lora->receive();

  waitingAck = true;
  pendingSend = false;
  waitStart = millis();

  Serial.print("[CH] SEND IDX=");
  Serial.println(currentIdx);
}

void ChannelController::loop() {
  // ---- send if allowed ----
  if (pendingSend && !waitingAck) {
    sendPacket();
  }

  // ---- ACK timeout ----
  if (waitingAck && millis() - waitStart > ACK_TIMEOUT) {
    waitingAck = false;
    pendingSend = true;
    Serial.println("[CH] ACK TIMEOUT");
  }
}

void ChannelController::handleAck() {
  if (waitingAck) {
    waitingAck = false;
    Serial.println("[CH] GOT ACK");
  }
}
