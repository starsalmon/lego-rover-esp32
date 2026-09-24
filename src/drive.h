#pragma once

#include <Arduino.h>

#ifndef PWM_FREQ_HZ
#define PWM_FREQ_HZ 20000
#endif

// Dead-zone remap: logical cmd 0..1 → duty dead_min..100% (MakerVerse @ 20 kHz).
#ifndef MIN_PWM_FLOOR
#define MIN_PWM_FLOOR 0.58f
#endif

#ifndef MIN_PWM_FLOOR_L
#define MIN_PWM_FLOOR_L MIN_PWM_FLOOR
#endif

#ifndef MIN_PWM_FLOOR_R
#define MIN_PWM_FLOOR_R 0.57f
#endif

// Software motor direction (no rewire): set to 1 in platformio.ini, re-flash only.
#ifndef MOTOR_INVERT_L
#define MOTOR_INVERT_L 0
#endif
#ifndef MOTOR_INVERT_R
#define MOTOR_INVERT_R 0
#endif
#ifndef MOTOR_SWAP_LR
#define MOTOR_SWAP_LR 0
#endif

class SmoothDrive {
 public:
  bool begin(int dir_l, int pwm_l, int dir_r, int pwm_r);
  void set_target(float left, float right);
  void tick();
  /** Instant PWM zero — E-stop, low battery, stall only. Everything else ramps. */
  void hard_stop();
  void stop() { hard_stop(); }

  // Characterization: linear duty 0..1 per side, no ramp / MIN_PWM floor.
  // Sign = direction (+ = forward per DIR wiring, - = reverse).
  void apply_direct(float left, float right);
  // Set LEDC duty 0..255 directly (motor cal).
  void apply_duty_lr(uint8_t duty_l, bool rev_l, uint8_t duty_r, bool rev_r);
  bool set_pwm_freq(uint32_t hz);
  // Call before setup() finishes — pins idle low so motors don't run during boot.
  static void gpio_safe_idle(int dir_l, int pwm_l, int dir_r, int pwm_r);

  bool ok() const { return _pwm_ok; }
  float cur_left() const { return _cur_l; }
  float cur_right() const { return _cur_r; }
  float tgt_left() const { return _tgt_l; }
  float tgt_right() const { return _tgt_r; }

 private:
  int _dir_l = -1, _pwm_l = -1, _dir_r = -1, _pwm_r = -1;
  float _cur_l = 0, _cur_r = 0;
  float _tgt_l = 0, _tgt_r = 0;
  uint32_t _last_ms = 0;
  bool _pwm_ok = false;
  uint8_t _pwm_bits = 8;
  uint32_t _pwm_freq = PWM_FREQ_HZ;
  uint8_t _ch_l = 0;
  uint8_t _ch_r = 1;
  // Wall-clock rates so loop jitter cannot make motion snappy.
  // ~0.15 cruise takes ~0.3 s to reach; reverse always bleeds through 0.
  static constexpr float ACCEL_PER_S = 0.50f;
  static constexpr float BRAKE_PER_S = 0.42f;
  static constexpr float MIN_PWM_L = MIN_PWM_FLOOR_L;
  static constexpr float MIN_PWM_R = MIN_PWM_FLOOR_R;
  static constexpr float MOVE_EPS = 0.03f;
  static constexpr uint32_t PWM_FREQ = PWM_FREQ_HZ;

  static float _cmd_to_scale(float a, float min_pwm);
  void _apply(float l, float r);
  void _apply_direct(float l, float r);
};
