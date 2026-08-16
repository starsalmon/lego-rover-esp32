#include "estop.h"

void EStop::begin(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT_PULLUP);
  _active = false;
  _long_fired = false;
  _pending = kBtnNone;
  _press_ms = 0;
}

void EStop::poll(uint32_t now_ms) {
  if (_pin < 0) return;

  const bool pressed = digitalRead(_pin) == LOW;
  if (pressed && !_active) {
    _press_ms = now_ms;
    _long_fired = false;
  } else if (!pressed) {
    _long_fired = false;
  } else if (pressed && !_long_fired && (now_ms - _press_ms) >= ROVER_ESTOP_LONG_MS) {
    _long_fired = true;
    _pending = kBtnEstopLong;
  }

  _active = pressed;
}

RoverButtonEvent EStop::take_event() {
  const RoverButtonEvent ev = _pending;
  _pending = kBtnNone;
  return ev;
}
