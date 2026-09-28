#include "ir_tx.h"

#include "fleet_ir_proto.h"

namespace {

constexpr uint8_t kDutyBits = 8;
constexpr uint8_t kDutyMax = (1u << kDutyBits) - 1u;

enum TxPhase : uint8_t {
  kTxFrameGap = 0,
  kTxSync,
  kTxData,
};

uint32_t g_frame_start_us = 0;
uint8_t g_tx_phase = kTxFrameGap;
uint8_t g_burst_idx = 0;
uint32_t g_burst_start_us = 0;

bool modulated_on(uint32_t now_us, uint32_t start_us) {
  const uint32_t t = now_us - start_us;
  const uint32_t period = FLEET_IR_BURST_US + FLEET_IR_BIT_GAP_US;
  return (t % period) < FLEET_IR_BURST_US;
}

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
  g_frame_start_us = micros();
  g_tx_phase = kTxFrameGap;
  g_burst_idx = 0;
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
  tick();
}

bool IrTx::in_tx_frame() const {
  if (!_ok || !_armed) {
    return false;
  }
#if !IR_TX_MODULATE
  return true;
#else
  return g_tx_phase != kTxFrameGap;
#endif
}

void IrTx::tick() {
  if (!_ok || !_armed) {
    return;
  }

#if !IR_TX_MODULATE
  apply_carrier(true);
  return;
#endif

  const uint32_t now = micros();
  const uint8_t data_bursts = static_cast<uint8_t>(FLEET_IR_ID + 1);

  if (g_tx_phase == kTxFrameGap) {
    apply_carrier(false);
    if ((now - g_frame_start_us) >= FLEET_IR_FRAME_US) {
      g_tx_phase = kTxSync;
      g_burst_start_us = now;
      g_frame_start_us = now;
    }
    return;
  }

  if (g_tx_phase == kTxSync) {
    apply_carrier(modulated_on(now, g_burst_start_us));
    if ((now - g_burst_start_us) >= FLEET_IR_SYNC_US) {
      g_tx_phase = kTxData;
      g_burst_idx = 0;
      g_burst_start_us = now;
    }
    return;
  }

  // Data bursts: (fleet_id + 1) marked pulses.
  const uint32_t burst_period = FLEET_IR_BURST_US + FLEET_IR_BIT_GAP_US;
  const uint32_t elapsed = now - g_burst_start_us;
  const uint32_t slot = elapsed / burst_period;
  const uint32_t in_slot = elapsed % burst_period;

  if (slot >= data_bursts) {
    g_tx_phase = kTxFrameGap;
    g_frame_start_us = now;
    apply_carrier(false);
    return;
  }

  apply_carrier(in_slot < FLEET_IR_BURST_US);
}
