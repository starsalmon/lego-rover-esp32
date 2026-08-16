#include "motion_detect.h"

#include <cmath>

void MotionDetect::begin() {
  _lp_ax = 0.0f;
  _lp_ay = 0.0f;
  _lp_az = 1.0f;
  _drive_since = 0;
  _last_stall_ms = 0;
  _wheel_ref_ticks = 0;
  _wheel_ref_ms = 0;
  _yaw_accum_rad = 0.0f;
  _last_ms = 0;
  _stall = false;
}

uint32_t MotionDetect::wheel_window_ms(float motor) {
  const float m = fmaxf(motor, 0.05f);
  const float scaled =
      static_cast<float>(STALL_TICK_WINDOW_CRUISE_MS) * (ROVER_CRUISE_MAX_LIN / m);
  const uint32_t win = static_cast<uint32_t>(scaled);
  return win < 500u ? 500u : win;
}

void MotionDetect::update(float ax, float ay, float az, float gx, float gy, float gz,
                          float motor_l, float motor_r, uint32_t wheel_l_ticks,
                          uint32_t wheel_r_ticks, uint32_t now_ms) {
  (void)ax;
  (void)ay;
  (void)az;
  (void)gx;
  (void)gy;

  _stall = false;

  const float motor = fmaxf(fabsf(motor_l), fabsf(motor_r));
  float dt = 0.0f;
  if (_last_ms != 0 && now_ms > _last_ms) {
    dt = (now_ms - _last_ms) / 1000.0f;
    if (dt > 0.08f) {
      dt = 0.08f;
    }
  }
  _last_ms = now_ms;

  if (motor < STALL_MOTOR) {
    _drive_since = 0;
    _wheel_ref_ticks = wheel_l_ticks + wheel_r_ticks;
    _wheel_ref_ms = now_ms;
    _yaw_accum_rad = 0.0f;
    return;
  }

  _yaw_accum_rad += fabsf(gz) * dt;

  if (_drive_since == 0) {
    _drive_since = now_ms;
    _wheel_ref_ticks = wheel_l_ticks + wheel_r_ticks;
    _wheel_ref_ms = now_ms;
    _yaw_accum_rad = 0.0f;
    return;
  }

  if (now_ms - _drive_since < STALL_ARM_MS) {
    return;
  }

  if (now_ms - _last_stall_ms <= STALL_COOLDOWN_MS) {
    return;
  }

  const uint32_t combined_ticks = wheel_l_ticks + wheel_r_ticks;
  const uint32_t tick_delta = combined_ticks - _wheel_ref_ticks;
  const uint32_t window_ms = wheel_window_ms(motor);
  const bool window_elapsed = (now_ms - _wheel_ref_ms) >= window_ms;

  if (!window_elapsed) {
    return;
  }

  const bool low_ticks =
      motor >= WHEEL_STALL_MOTOR &&
      tick_delta < static_cast<uint32_t>(STALL_TICKS_REQUIRED);

  const bool anchor_stall = motor >= WHEEL_STALL_MOTOR &&
                            tick_delta >= static_cast<uint32_t>(STALL_TICKS_REQUIRED) &&
                            _yaw_accum_rad < ANCHOR_YAW_MIN_RAD;

  if (low_ticks || anchor_stall) {
    _stall = true;
    _last_stall_ms = now_ms;
    _drive_since = now_ms;
    _wheel_ref_ticks = combined_ticks;
    _wheel_ref_ms = now_ms;
    _yaw_accum_rad = 0.0f;
  } else {
    _wheel_ref_ticks = combined_ticks;
    _wheel_ref_ms = now_ms;
    _yaw_accum_rad = 0.0f;
  }
}

void MotionDetect::clear_events() {
  _stall = false;
}
