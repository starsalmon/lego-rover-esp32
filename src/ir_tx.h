#pragma once

#include <Arduino.h>

// 38 kHz carrier for TSOP4138 / TSOP4838 receivers (chase beacon).
// TSOP AGC ignores a steady carrier — tick() gates 560 µs bursts when modulating.
class IrTx {
 public:
  bool begin(int pin, uint32_t hz = 38000, int8_t ledc_channel = 4);
  bool ok() const { return _ok; }
  void set_enabled(bool on);
  void tick();
  bool enabled() const { return _armed; }

 private:
  void apply_carrier(bool on);

  int _pin = -1;
  int8_t _ch = 4;
  bool _ok = false;
  bool _armed = false;
  uint32_t _duty_on = 128;
};
