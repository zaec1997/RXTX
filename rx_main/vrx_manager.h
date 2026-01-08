#pragma once
#include "vrx.h"

class VRX_Manager {
public:
  void add(VRX *v) {
    vrx[count++] = v;
  }

  void begin() {
    for (int i = 0; i < count; i++) vrx[i]->begin();
  }

  void loop() {
    for (int i = 0; i < count; i++) vrx[i]->loop();
  }

  void setActive(uint8_t id) {
    if (id < count) active = id;
  }

  void setChannel(uint8_t idx) {
    if (vrx[active]->isActive())
      vrx[active]->setChannel(idx);
  }

  

private:
  VRX* vrx[3];
  uint8_t count = 0;
  uint8_t active = 0;
};
