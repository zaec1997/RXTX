#pragma once
#include "vrx.h"

class VRX_Manager {
public:
  // maxSlots задає розмір внутрішнього масиву, якщо потрібно — збільшити
  static constexpr int MAX_SLOTS = 6;

  VRX_Manager() : count(0), active(0) {}

  void add(VRX *v) {
    if (count >= MAX_SLOTS) {
      // перевищення — ігноруємо або логируемо
      Serial.println("[VRX_MGR] add() overflow");
      return;
    }
    vrx[count++] = v;
  }

  void begin() {
    for (int i = 0; i < count; i++) if (vrx[i]) vrx[i]->begin();
  }

  void loop() {
    for (int i = 0; i < count; i++) if (vrx[i]) vrx[i]->loop();
  }

  void setActive(uint8_t id) {
    if (id < count) active = id;
  }

  void setChannel(uint8_t idx) {
    if (count == 0) return;
    if (active >= count) return;
    if (!vrx[active]) return;
    if (vrx[active]->isActive())
      vrx[active]->setChannel(idx);
  }

private:
  VRX* vrx[MAX_SLOTS];
  uint8_t count;
  uint8_t active;
};
