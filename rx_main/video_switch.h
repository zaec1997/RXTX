#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

/*
  VideoSwitch (RC PWM, 100% stable)
  --------------------------------
  1000us -> VRX 0
  1500us -> VRX 1
  2000us -> VRX 2
*/

class VideoSwitch {
public:
  void begin(int pin) {
    pwmPin = pin;

    servo.setPeriodHertz(50);          // 50 Hz
    servo.attach(pwmPin, 1000, 2000);  // min/max us

    select(0);
  }

  void select(uint8_t id) {
    int us;

    if (id == 0)      us = 1000;
    else if (id == 1) us = 1500;
    else              us = 2000;

    servo.writeMicroseconds(us);
  }

private:
  int pwmPin = -1;
  Servo servo;
};
