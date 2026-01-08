#include <SPI.h>
#include <LoRa.h>

#include "vrx.h"
#include "vrx_manager.h"
#include "vrx_steadyview.h"
#include "vrx_ft3500.h"
#include "video_switch.h"
#include "protocol.h"

// ---------- LoRa ----------
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26

// ---------- Video switch ----------
#define VIDEO_PWM_PIN 27

// ---------- VRX ----------
VRX_SteadyView vrx0(Serial2, 16, 17);

VRX_FT3500 vrx1(
  13, 25, 33,   // CS1 CS2 CS3
  32, 21, 22    // S1  S2  S3
);

VRX_Manager vrxMgr;
VideoSwitch videoSwitch;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("FPV RX START");

  // --- VRX registration ---
  vrxMgr.add(&vrx0);
  vrxMgr.add(&vrx1);
  vrxMgr.begin();

  // --- Video switch ---
  videoSwitch.begin(VIDEO_PWM_PIN);

  // --- LoRa ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa FAIL");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println("RX READY");
}

void sendAck() {
  Packet ack;
  ack.cmd = CMD_ACK;
  ack.arg1 = 0;
  ack.arg2 = 0;
  ack.crc = calcCRC(ack);

  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ack, sizeof(ack));
  LoRa.endPacket();
  LoRa.receive();
}

void handlePacket(const Packet &p) {
  if (!checkCRC(p)) {
    Serial.println("[RX] bad CRC");
    return;
  }

  switch (p.cmd) {
    case CMD_SET_CHANNEL:
      Serial.print("[RX] SET_CHANNEL idx=");
      Serial.println(p.arg1);
      vrxMgr.setChannel(p.arg1);
      sendAck();
      break;

    case CMD_SET_VRX:
      Serial.print("[RX] SET_VRX id=");
      Serial.println(p.arg1);
      vrxMgr.setActive(p.arg1);
      videoSwitch.select(p.arg1);
      sendAck();
      break;

    case CMD_PING:
      // optional: reply
      break;

    default:
      Serial.print("[RX] unknown cmd=");
      Serial.println(p.cmd, HEX);
      break;
  }
}

void loop() {
  vrxMgr.loop();

  int ps = LoRa.parsePacket();
  if (!ps) return;

  // if packet size equals Packet, read binary
  if (ps == sizeof(Packet)) {
    Packet p;
    LoRa.readBytes((uint8_t*)&p, sizeof(p));
    handlePacket(p);
    LoRa.receive();
    return;
  }

  // fallback: textual handling for compatibility
  String msg;
  while (LoRa.available()) msg += (char)LoRa.read();

  Serial.print("[RX][TEXT] ");
  Serial.println(msg);

  if (msg.startsWith("SET,")) {
    uint8_t idx = msg.substring(4).toInt();
    vrxMgr.setChannel(idx);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.print("ACK");
    LoRa.endPacket();
    LoRa.receive();
    return;
  }

  if (msg.startsWith("VRX,")) {
    uint8_t id = msg.substring(4).toInt();

    vrxMgr.setActive(id);
    videoSwitch.select(id);

    Serial.print("[RX] ACTIVE VRX = ");
    Serial.println(id);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.print("ACK");
    LoRa.endPacket();
    LoRa.receive();
    return;
  }
}
