#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "channel_controller.h"
#include "vrx_controller.h"
#include "protocol.h"

// ================= OLED =================
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ================= Encoder =================
#define ENC_CLK 34
#define ENC_DT  35
#define ENC_SW  32

int lastCLK;
int rawSteps = 0;
const int STEPS_PER_CLICK = 2;

// ================= Button =================
unsigned long btnPressTime = 0;
bool btnPrev = HIGH;

// ================= LoRa =================
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26

// ================= Controllers =================
ChannelController channelCtrl;
VrxController vrxCtrl;

// ================= Channels =================
struct Channel {
  char band;
  uint8_t ch;
  uint16_t freq;
  uint8_t idx;
};

const Channel channels[] = {
  // A
  {'A',1,5865,0x00},{'A',2,5845,0x01},{'A',3,5825,0x02},{'A',4,5805,0x03},
  {'A',5,5785,0x04},{'A',6,5765,0x05},{'A',7,5745,0x06},{'A',8,5725,0x07},
  // B
  {'B',1,5733,0x08},{'B',2,5752,0x09},{'B',3,5771,0x0A},{'B',4,5790,0x0B},
  {'B',5,5809,0x0C},{'B',6,5828,0x0D},{'B',7,5847,0x0E},{'B',8,5866,0x0F},
  // E
  {'E',1,5705,0x10},{'E',2,5685,0x11},{'E',3,5665,0x12},{'E',4,5645,0x13},
  {'E',5,5885,0x14},{'E',6,5905,0x15},{'E',7,5925,0x16},{'E',8,5945,0x17},
  // F
  {'F',1,5740,0x18},{'F',2,5760,0x19},{'F',3,5780,0x1A},{'F',4,5800,0x1B},
  {'F',5,5820,0x1C},{'F',6,5840,0x1D},{'F',7,5860,0x1E},{'F',8,5880,0x1F},
  // R
  {'R',1,5658,0x20},{'R',2,5695,0x21},{'R',3,5732,0x22},{'R',4,5769,0x23},
  {'R',5,5806,0x24},{'R',6,5843,0x25},{'R',7,5880,0x26},{'R',8,5917,0x27},
  // L
  {'L',1,5333,0x28},{'L',2,5373,0x29},{'L',3,5413,0x2A},{'L',4,5453,0x2B},
  {'L',5,5493,0x2C},{'L',6,5533,0x2D},{'L',7,5573,0x2E},{'L',8,5613,0x2F},
  // X
  {'X',1,4990,0x30},{'X',2,5020,0x31},{'X',3,5050,0x32},{'X',4,5080,0x33},
  {'X',5,5110,0x34},{'X',6,5140,0x35},{'X',7,5170,0x36},{'X',8,5200,0x37},
};

const int CHANNEL_COUNT = sizeof(channels) / sizeof(channels[0]);
int chIndex = 0;

// ================= VRX =================
enum VrxType {
  VRX_STEADYVIEW_X = 0,
  VRX_RX3364,
  VRX_RX5808,
  VRX_12G_DIVERSITY,
  VRX_RUSHFPV_33G,
  VRX_COUNT
};

const char* vrxNames[VRX_COUNT] = {
  "SteadyView X",
  "RX3364",
  "RX5808",
  "1.2G Diversity",
  "RushFPV 3.3G"
};

int vrxIndex = 0;

// ================= UI =================
enum UiState {
  UI_MAIN,
  UI_MENU,
  UI_MENU_VRX
};

UiState uiState = UI_MAIN;
bool uiDirty = true;

// ================= UI draw =================
void drawUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (uiState == UI_MAIN) {
    const Channel &c = channels[chIndex];
    display.setCursor(0,0);
    display.println("FPV Station");

    display.setCursor(0,16);
    display.print("CH: ");
    display.print(c.band);
    display.print(c.ch);

    display.setCursor(0,28);
    display.print("Freq: ");
    if (c.freq) display.print(c.freq);
    else display.print("CUST");

    display.setCursor(0,40);
    display.print("VRX: ");
    display.print(vrxNames[vrxIndex]);

    display.setCursor(0,56);
    if (channelCtrl.isWaitingAck() || vrxCtrl.isWaitingAck()) {
      display.print("WAIT ACK");
    } else {
      display.print("READY");
    }
  }

  if (uiState == UI_MENU) {
    display.setCursor(0,0);
    display.println("MENU");
    display.setCursor(0,20);
    display.print("> VRX Select");
  }

  if (uiState == UI_MENU_VRX) {
    display.setCursor(0,0);
    display.println("Select VRX");

    for (int i = 0; i < VRX_COUNT; i++) {
      display.setCursor(0, 14 + i * 10);
      if (i == vrxIndex) display.print("> ");
      else display.print("  ");
      display.print(vrxNames[i]);
    }
  }

  display.display();
  uiDirty = false;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastCLK = digitalRead(ENC_CLK);

  Wire.begin(21,22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.begin(433E6);
  LoRa.enableCrc();
  LoRa.receive();

  channelCtrl.begin(&LoRa);
  vrxCtrl.begin(&LoRa);

  uiDirty = true;
}

// ================= LOOP =================
void loop() {
  // ---------- Controllers ----------
  channelCtrl.loop();
  vrxCtrl.loop();

  // ---------- Encoder ----------
  int clk = digitalRead(ENC_CLK);
  if (clk != lastCLK) {
    int dt = digitalRead(ENC_DT);
    rawSteps += (dt != clk) ? 1 : -1;

    if (rawSteps >= STEPS_PER_CLICK) {
      rawSteps = 0;
      if (uiState == UI_MAIN) {
        chIndex = (chIndex + 1) % CHANNEL_COUNT;
        channelCtrl.setChannel(channels[chIndex].idx);
      } else if (uiState == UI_MENU_VRX) {
        vrxIndex = (vrxIndex + 1) % VRX_COUNT;
      }
      uiDirty = true;
    }

    if (rawSteps <= -STEPS_PER_CLICK) {
      rawSteps = 0;
      if (uiState == UI_MAIN) {
        chIndex = (chIndex + CHANNEL_COUNT - 1) % CHANNEL_COUNT;
        channelCtrl.setChannel(channels[chIndex].idx);
      } else if (uiState == UI_MENU_VRX) {
        vrxIndex = (vrxIndex + VRX_COUNT - 1) % VRX_COUNT;
      }
      uiDirty = true;
    }
  }
  lastCLK = clk;

  // ---------- Button ----------
  bool btn = digitalRead(ENC_SW);

  if (btn == LOW && btnPrev == HIGH) {
    btnPressTime = millis();
  }

  if (btn == HIGH && btnPrev == LOW) {
    unsigned long dur = millis() - btnPressTime;

    if (dur > 700) { // long
      if (uiState == UI_MENU_VRX) uiState = UI_MENU;
      else if (uiState == UI_MENU) uiState = UI_MAIN;
    } else { // short
      if (uiState == UI_MAIN) uiState = UI_MENU;
      else if (uiState == UI_MENU) uiState = UI_MENU_VRX;
      else if (uiState == UI_MENU_VRX) {
        vrxCtrl.setVrx(vrxIndex);
        uiState = UI_MAIN;
      }
    }
    uiDirty = true;
  }

  btnPrev = btn;

  // ---------- LoRa RX (ACK handling) ----------
  int ps = LoRa.parsePacket();
  if (ps == sizeof(Packet)) {
    Packet p;
    LoRa.readBytes((uint8_t*)&p, sizeof(p));
    if (checkCRC(p) && p.cmd == CMD_ACK) {
      channelCtrl.handleAck();
      vrxCtrl.handleAck(p);
    }
    LoRa.receive();
  }

  // Update UI if anything changed
  if (uiDirty || channelCtrl.isWaitingAck() != vrxCtrl.isWaitingAck()) {
    drawUI();
  }
}
