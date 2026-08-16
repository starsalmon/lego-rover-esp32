#pragma once

#include <Arduino.h>

#include "rover_button_events.h"

// Onboard KEY (GPIO14): tap / hold / double-tap (standby).
class GoButton {
 public:
  void begin(int pin);
  void poll(uint32_t now_ms);
  void set_multi_tap_enabled(bool enabled) { _multi_tap = enabled; }

  RoverButtonEvent take_event();
  uint8_t active_tap_count() const { return _tap_count; }
  bool pressed() const { return _pressed; }

 private:
  void finalize_taps();

  int _pin = -1;
  bool _last_high = true;
  bool _pressed = false;
  bool _long_fired = false;
  bool _multi_tap = true;
  uint8_t _tap_count = 0;
  RoverButtonEvent _pending = kBtnNone;
  uint32_t _press_ms = 0;
  uint32_t _last_edge_ms = 0;
  uint32_t _last_tap_ms = 0;
};
