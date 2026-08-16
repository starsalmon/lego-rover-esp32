#include "rover_shift_ir.h"

void ShiftIr::begin(int ser_pin, int clk_pin, int miso_pin, int latch_pin) {
  _ser = ser_pin;
  _clk = clk_pin;
  _miso = miso_pin;
  _latch = latch_pin;
  _ok = _ser >= 0 && _clk >= 0 && _miso >= 0 && _latch >= 0;
  if (!_ok) return;

  pinMode(_ser, OUTPUT);
  pinMode(_clk, OUTPUT);
  pinMode(_latch, OUTPUT);
  pinMode(_miso, INPUT);
  digitalWrite(_clk, LOW);
  digitalWrite(_ser, LOW);
  digitalWrite(_latch, HIGH);
  _aux_led_on = 0;
  transfer(0);
  _front_phase = FrontPhase::kOff;
  _front_phase_until = millis() + IR_FRONT_OFF_MS;
}

void ShiftIr::pulse_clk() {
  digitalWrite(_clk, HIGH);
  delayMicroseconds(2);
  digitalWrite(_clk, LOW);
  delayMicroseconds(2);
}

uint8_t ShiftIr::_outputs_front_on() const {
  uint8_t out = _aux_led_on;
  out |= IR_OUT_FRONT;
  return out;
}

uint8_t ShiftIr::_outputs_front_off() const {
  return _aux_led_on;
}

bool ShiftIr::_hit(uint8_t inputs, uint8_t mask) const {
  return (inputs & mask) != 0;
}

uint8_t ShiftIr::transfer(uint8_t outputs) {
  if (!_ok) return 0;

  digitalWrite(_latch, HIGH);
  for (int i = 0; i < 8; i++) {
    digitalWrite(_ser, (outputs >> i) & 1);
    pulse_clk();
  }

  digitalWrite(_latch, LOW);
  delayMicroseconds(5);
  digitalWrite(_latch, HIGH);
  delayMicroseconds(5);

  uint8_t inputs = 0;
  for (int i = 0; i < 8; i++) {
    if (digitalRead(_miso)) {
      inputs |= static_cast<uint8_t>(1u << i);
    }
    pulse_clk();
  }
  return inputs;
}

void ShiftIr::set_aux_led(bool on) {
  _aux_led_on = on ? IR_OUT_AUX : 0;
  if (_ok) {
    transfer(_outputs_front_off());
  }
}

bool ShiftIr::read_aux_raw() {
  if (!_ok) return false;
  return _hit(transfer(_outputs_front_off()), IR_IN_AUX);
}

void ShiftIr::tick(uint32_t now_ms) {
  if (!_ok) return;

  if ((int32_t)(now_ms - _front_phase_until) < 0) {
    return;
  }

  if (_front_phase == FrontPhase::kOff) {
    _front_off_hit = _hit(transfer(_outputs_front_off()), IR_IN_FRONT);
    transfer(_outputs_front_on());
    _front_phase = FrontPhase::kOn;
    _front_phase_until = now_ms + IR_FRONT_ON_MS;
    return;
  }

  _front_on_hit = _hit(transfer(_outputs_front_on()), IR_IN_FRONT);
  transfer(_outputs_front_off());
  _front_delta_hit = _front_on_hit && !_front_off_hit;
  _front_phase = FrontPhase::kOff;
  _front_phase_until = now_ms + IR_FRONT_OFF_MS;
}

void ShiftIr::sample_aux(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit) {
  off_hit = false;
  on_hit = false;
  if (!_ok) return;

  const uint8_t saved_aux = _aux_led_on;
  _aux_led_on = 0;

  transfer(_outputs_front_off());
  delay(off_ms);
  off_hit = _hit(transfer(_outputs_front_off()), IR_IN_AUX);

  transfer(_outputs_front_off() | IR_OUT_AUX);
  delay(on_ms);
  on_hit = _hit(transfer(_outputs_front_off() | IR_OUT_AUX), IR_IN_AUX);

  _aux_led_on = saved_aux;
  transfer(_outputs_front_off());
}
