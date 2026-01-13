#include <SPI.h>
#include <LoRa.h>

#include "vrx.h"
#include "vrx_manager.h"
#include "vrx_steadyview.h"
#include "vrx_ft3500.h"
#include "video_switch.h"

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


videoSwitch.begin(27);

videoSwitch.select(0);
delay(2000);

videoSwitch.select(1);
delay(2000);

videoSwitch.select(2);
delay(2000);

  // --- VRX ---
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

void loop() {
  vrxMgr.loop();

  // ---- Check for RSSI updates and send telemetry ----
  if (vrx0.consumeRssiUpdated()) {
    float rssiA = vrx0.getLastRssiDbmA();
    float rssiB = vrx0.getLastRssiDbmB();
    
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.print("RSSI,");
    LoRa.print(rssiA, 1);
    LoRa.print(",");
    LoRa.print(rssiB, 1);
    LoRa.endPacket();
    LoRa.receive();
  }

  int ps = LoRa.parsePacket();
  if (!ps) return;

  String msg;
  while (LoRa.available()) msg += (char)LoRa.read();

  Serial.print("[RX] ");
  Serial.println(msg);

  // ---- Channel ----
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

  // ---- VRX select ----
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
