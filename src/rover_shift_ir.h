#pragma once

#include <Arduino.h>

// 74HC595 outputs (emitter LED gates via MOSFET) — LSB = Q0.
constexpr uint8_t IR_OUT_FRONT = 1u << 0;
constexpr uint8_t IR_OUT_AUX = 1u << 1;

// 74HC165 inputs (LM358 comparator OUT) — LSB = Q0.
constexpr uint8_t IR_IN_FRONT = 1u << 0;
constexpr uint8_t IR_IN_AUX = 1u << 1;

// Match lego-rover-ros2 rover_ir.py defaults (ms).
constexpr uint16_t IR_FRONT_OFF_MS = 50;
constexpr uint16_t IR_FRONT_ON_MS = 10;
constexpr uint16_t IR_AUX_OFF_MS = 10;
constexpr uint16_t IR_AUX_ON_MS = 100;

class ShiftIr {
 public:
  void begin(int ser_pin, int clk_pin, int miso_pin, int latch_pin);
  bool ok() const { return _ok; }

  // Background front pulser (~17 Hz) — call every loop().
  void tick(uint32_t now_ms);

  bool front_hit() const { return _front_delta_hit; }
  bool front_off_hit() const { return _front_off_hit; }
  bool front_on_hit() const { return _front_on_hit; }

  void set_aux_led(bool on);
  bool read_aux_raw();

  // Blocking aux pulse sample (for radar sweep).
  void sample_aux(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit);

  uint8_t transfer(uint8_t outputs);

 private:
  bool _hit(uint8_t inputs, uint8_t mask) const;
  void pulse_clk();
  uint8_t _outputs_front_on() const;
  uint8_t _outputs_front_off() const;

  int _ser = -1;
  int _clk = -1;
  int _miso = -1;
  int _latch = -1;
  bool _ok = false;
  uint8_t _aux_led_on = 0;

  enum class FrontPhase : uint8_t { kOff, kOn };
  FrontPhase _front_phase = FrontPhase::kOff;
  uint32_t _front_phase_until = 0;
  bool _front_off_hit = false;
  bool _front_on_hit = false;
  bool _front_delta_hit = false;
};
