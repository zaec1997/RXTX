#pragma once
#include <Arduino.h>

class VRX {
public:
  virtual void begin() = 0;
  virtual void loop() = 0;

  virtual bool isActive() = 0;
  virtual void setChannel(uint8_t idx) = 0;
};
