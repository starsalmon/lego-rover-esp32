#pragma once

#include <Arduino.h>

#include "rover_button_events.h"

// Onboard BOOT (GPIO0): held = emergency stop. Long hold = shutdown request.
class EStop {
 public:
  void begin(int pin);
  void poll(uint32_t now_ms);
  bool active() const { return _active; }
  RoverButtonEvent take_event();

 private:
  int _pin = -1;
  bool _active = false;
  bool _long_fired = false;
  RoverButtonEvent _pending = kBtnNone;
  uint32_t _press_ms = 0;
};
