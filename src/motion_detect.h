#pragma once

#include <Arduino.h>

#include "rover_pins_s3.h"

class MotionDetect {
 public:
  void begin();
  // ax/ay/az in g; gx/gy/gz in rad/s. motor_l/r are applied wheel commands [-1, 1].
  void update(float ax, float ay, float az, float gx, float gy, float gz, float motor_l,
              float motor_r, uint32_t wheel_l_ticks, uint32_t wheel_r_ticks, uint32_t now_ms);
  bool stall() const { return _stall; }
  void clear_events();

 private:
  static uint32_t wheel_window_ms(float motor);

  uint32_t _drive_since = 0;
  uint32_t _last_stall_ms = 0;
  uint32_t _wheel_ref_ticks = 0;
  uint32_t _wheel_ref_ms = 0;
  uint32_t _last_ms = 0;
  float _yaw_accum_rad = 0.0f;
  float _lp_ax = 0.0f;
  float _lp_ay = 0.0f;
  float _lp_az = 1.0f;
  bool _stall = false;

  static constexpr float STALL_MOTOR = 0.04f;
  static constexpr float WHEEL_STALL_MOTOR = 0.05f;
  static constexpr uint32_t STALL_ARM_MS = 350;
  static constexpr uint32_t STALL_COOLDOWN_MS = 1500;
  // Wheels turning but body barely rotates (slip / push against wall).
  static constexpr float ANCHOR_YAW_MIN_RAD = 0.10f;
};
