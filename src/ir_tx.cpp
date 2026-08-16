#include "ir_tx.h"

namespace {

constexpr uint8_t kDutyBits = 8;
constexpr uint8_t kDutyMax = (1u << kDutyBits) - 1u;

}  // namespace

bool IrTx::begin(int pin, uint32_t hz, int8_t ledc_channel) {
  _pin = pin;
  _ch = ledc_channel;
  _duty_on = kDutyMax / 2;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(static_cast<uint8_t>(_pin));
  if (!ledcAttachChannel(static_cast<uint8_t>(_pin), hz, kDutyBits, ledc_channel)) {
    _ok = false;
    return false;
  }
  ledcWrite(static_cast<uint8_t>(_pin), 0);
#else
  ledcDetachPin(static_cast<uint8_t>(_pin));
  ledcSetup(static_cast<uint8_t>(ledc_channel), hz, kDutyBits);
  ledcAttachPin(static_cast<uint8_t>(_pin), static_cast<uint8_t>(ledc_channel));
  ledcWrite(static_cast<uint8_t>(ledc_channel), 0);
#endif

  _ok = true;
  _armed = false;
  return true;
}

void IrTx::apply_carrier(bool on) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(static_cast<uint8_t>(_pin), on ? _duty_on : 0);
#else
  ledcWrite(static_cast<uint8_t>(_ch), on ? _duty_on : 0);
#endif
}

void IrTx::set_enabled(bool on) {
  if (!_ok) {
    return;
  }
  _armed = on;
  if (!on) {
    apply_carrier(false);
    return;
  }
#if IR_TX_MODULATE
  tick();
#else
  apply_carrier(true);
#endif
}

void IrTx::tick() {
  if (!_ok || !_armed) {
    return;
  }
#if IR_TX_MODULATE
  const uint32_t period = IR_TX_BURST_US + IR_TX_GAP_US;
  const bool burst = (micros() % period) < IR_TX_BURST_US;
  apply_carrier(burst);
#else
  apply_carrier(true);
#endif
}
