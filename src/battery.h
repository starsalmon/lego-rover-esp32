#pragma once

#include <Arduino.h>

class BatteryMonitor {
 public:
  void begin(int adc_pin, float scale, float low_v, float critical_v);
  void update(uint32_t now_ms);
  float voltage() const { return _volts; }
  bool low() const { return _low; }
  bool critical() const { return _critical; }
  uint16_t pin_millivolts() const { return _pin_mv; }

 private:
  int _pin = -1;
  float _scale = 2.0f;
  float _low_v = 6.4f;
  float _critical_v = 6.0f;
  float _volts = 0.0f;
  uint16_t _pin_mv = 0;
  bool _primed = false;
  bool _low = false;
  bool _critical = false;
};
