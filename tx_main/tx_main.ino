#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

bool waitingAck = false;
bool pendingSend = false;
unsigned long waitStart = 0;
const unsigned long ACK_TIMEOUT = 800;

// ================= Channels =================
struct Channel {
  char band;
  uint8_t ch;
  uint16_t freq;
  uint8_t idx;
};


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

// ================= LoRa helpers =================
void sendChannel(uint8_t idx) {
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print("SET,");
  LoRa.print(idx);
  LoRa.endPacket();
  LoRa.receive();

  waitingAck = true;
  waitStart = millis();
}

void sendVrx(uint8_t id) {
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print("VRX,");
  LoRa.print(id);
  LoRa.endPacket();
  LoRa.receive();
}

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
    display.print(waitingAck ? "WAIT ACK" : "READY");
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

// ================= Channel table =================
struct Channel {
  char band;
  uint8_t ch;
  uint16_t freq;   // 0 = CUST
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
  // U
  {'U',1,5325,0x38},{'U',2,5348,0x39},{'U',3,5366,0x3A},{'U',4,5384,0x3B},
  {'U',5,5402,0x3C},{'U',6,5420,0x3D},{'U',7,5438,0x3E},{'U',8,5456,0x3F},
  // O
  {'O',1,5474,0x40},{'O',2,5492,0x41},{'O',3,5510,0x42},{'O',4,5528,0x43},
  {'O',5,5546,0x44},{'O',6,5564,0x45},{'O',7,5582,0x46},{'O',8,5600,0x47},
  // H
  {'H',1,5653,0x48},{'H',2,5693,0x49},{'H',3,5733,0x4A},{'H',4,5773,0x4B},
  {'H',5,5813,0x4C},{'H',6,5853,0x4D},{'H',7,5893,0x4E},{'H',8,5933,0x4F},
  // D
  {'D',1,5362,0x50},{'D',2,5399,0x51},{'D',3,5436,0x52},{'D',4,5473,0x53},
  {'D',5,5510,0x54},{'D',6,5547,0x55},{'D',7,5584,0x56},{'D',8,5621,0x57},
  // C
  {'C',1,5658,0x58},{'C',2,5695,0x59},{'C',3,5732,0x5A},{'C',4,5769,0x5B},
  {'C',5,5806,0x5C},{'C',6,5843,0x5D},{'C',7,5880,0x5E},{'C',8,5917,0x5F},
};

const int CHANNEL_COUNT = sizeof(channels) / sizeof(channels[0]);

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

  uiDirty = true;
}

// ================= LOOP =================
void loop() {
  // ---------- Encoder ----------
  int clk = digitalRead(ENC_CLK);
  if (clk != lastCLK) {
    int dt = digitalRead(ENC_DT);
    rawSteps += (dt != clk) ? 1 : -1;

    if (rawSteps >= STEPS_PER_CLICK) {
      rawSteps = 0;
      if (uiState == UI_MAIN) {
        chIndex = (chIndex + 1) % CHANNEL_COUNT;
        pendingSend = true;
      } else if (uiState == UI_MENU_VRX) {
        vrxIndex = (vrxIndex + 1) % VRX_COUNT;
      }
      uiDirty = true;
    }

    if (rawSteps <= -STEPS_PER_CLICK) {
      rawSteps = 0;
      if (uiState == UI_MAIN) {
        chIndex = (chIndex + CHANNEL_COUNT - 1) % CHANNEL_COUNT;
        pendingSend = true;
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
        sendVrx(vrxIndex);
        uiState = UI_MAIN;
      }
    }
    uiDirty = true;
  }

  btnPrev = btn;

  // ---------- Channel send ----------
  if (pendingSend && !waitingAck) {
    sendChannel(channels[chIndex].idx);
    pendingSend = false;
    uiDirty = true;
  }

  if (waitingAck && millis() - waitStart > ACK_TIMEOUT) {
    waitingAck = false;
    pendingSend = true;
  }

  int ps = LoRa.parsePacket();
  if (ps) {
    String msg;
    while (LoRa.available()) msg += (char)LoRa.read();
    if (msg.startsWith("ACK")) {
      waitingAck = false;
      uiDirty = true;
    }
    LoRa.receive();
  }

  if (uiDirty) drawUI();
}
