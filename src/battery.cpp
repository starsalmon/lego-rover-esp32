#include "battery.h"

void BatteryMonitor::begin(int adc_pin, float scale, float low_v, float critical_v) {
  _pin = adc_pin;
  _scale = scale;
  _low_v = low_v;
  _critical_v = critical_v;
  _primed = false;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < 4; i++) {
    (void)analogReadMilliVolts(_pin);
    delay(2);
  }
  _volts = 0.0f;
  _low = false;
  _critical = false;
}

void BatteryMonitor::update(uint32_t now_ms) {
  (void)now_ms;
  if (_pin < 0) return;

  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) {
    mv += analogReadMilliVolts(_pin);
  }
  mv /= 8;
  _pin_mv = (uint16_t)mv;

  const float v = (mv / 1000.0f) * _scale;
  if (!_primed) {
    _volts = v;
    _primed = true;
  } else {
    _volts = 0.85f * _volts + 0.15f * v;
  }
  _critical = _volts > 0.5f && _volts < _critical_v;
  _low = _critical || (_volts > 0.5f && _volts < _low_v);
}
