#include "rover_go_button.h"

void GoButton::begin(int pin) {
  _pin = pin;
  if (_pin < 0) return;
  pinMode(_pin, INPUT_PULLUP);
  _last_high = digitalRead(_pin) == HIGH;
  _pressed = !_last_high;
  _long_fired = false;
  _multi_tap = true;
  _tap_count = 0;
  _pending = kBtnNone;
  _press_ms = 0;
  _last_edge_ms = 0;
  _last_tap_ms = 0;
}

void GoButton::finalize_taps() {
  if (_tap_count >= ROVER_GO_MULTI_TAPS) {
    _pending = kBtnGoTriple;
  } else if (_tap_count == 1) {
    _pending = kBtnGoShort;
  }
  _tap_count = 0;
  _last_tap_ms = 0;
}

void GoButton::poll(uint32_t now_ms) {
  if (_pin < 0) return;

  const bool high = digitalRead(_pin) == HIGH;

  // Idle timeout — only commit after user stops tapping for SETTLE_MS.
  if (_tap_count > 0 && _last_tap_ms > 0 &&
      (now_ms - _last_tap_ms) >= ROVER_GO_MULTI_SETTLE_MS) {
    finalize_taps();
  }

  if (_last_high && !high) {
    if (now_ms - _last_edge_ms > ROVER_GO_DEBOUNCE_MS) {
      _pressed = true;
      _long_fired = false;
      _press_ms = now_ms;
      _last_edge_ms = now_ms;
    }
  } else if (!_last_high && high) {
    if (_pressed && now_ms - _last_edge_ms > ROVER_GO_DEBOUNCE_MS) {
      if (!_long_fired) {
        const uint32_t held = now_ms - _press_ms;
        if (held >= ROVER_GO_DEBOUNCE_MS) {
          if (_multi_tap) {
            if (_tap_count > 0 && (now_ms - _last_tap_ms) > ROVER_GO_MULTI_GAP_MS) {
              finalize_taps();
            }
            _tap_count++;
            _last_tap_ms = now_ms;
            Serial.printf("Go tap %u\n", (unsigned)_tap_count);
            if (_tap_count >= ROVER_GO_MULTI_TAPS) {
              _pending = kBtnGoTriple;
              _tap_count = 0;
              _last_tap_ms = 0;
            }
          } else {
            _pending = kBtnGoShort;
          }
        }
      }
      _pressed = false;
      _long_fired = false;
      _last_edge_ms = now_ms;
    }
  } else if (_pressed && !_long_fired && (now_ms - _press_ms) >= ROVER_GO_LONG_MS) {
    _long_fired = true;
    _tap_count = 0;
    _last_tap_ms = 0;
    _pending = kBtnGoLong;
  }

  _last_high = high;
}

RoverButtonEvent GoButton::take_event() {
  const RoverButtonEvent ev = _pending;
  _pending = kBtnNone;
  return ev;
}
