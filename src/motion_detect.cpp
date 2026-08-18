#include "motion_detect.h"

#include <cmath>

#ifndef WHEEL_STALL_MOTOR
#define WHEEL_STALL_MOTOR 0.05f
#endif
#ifndef STALL_ARM_MS
#define STALL_ARM_MS 350
#endif
#ifndef STALL_COOLDOWN_MS
#define STALL_COOLDOWN_MS 1500
#endif

void MotionDetect::begin() {
  _lp_ax = 0.0f;
  _lp_ay = 0.0f;
  _lp_az = 1.0f;
  _drive_since = 0;
  _last_stall_ms = 0;
  _wheel_ref_ticks = 0;
  _wheel_ref_ms = 0;
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
  (void)gz;

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
  (void)dt;

  if (motor < STALL_MOTOR) {
    _drive_since = 0;
    _wheel_ref_ticks = wheel_l_ticks + wheel_r_ticks;
    _wheel_ref_ms = now_ms;
    return;
  }

  if (_drive_since == 0) {
    _drive_since = now_ms;
    _wheel_ref_ticks = wheel_l_ticks + wheel_r_ticks;
    _wheel_ref_ms = now_ms;
    return;
  }

  if (now_ms - _drive_since < static_cast<uint32_t>(STALL_ARM_MS)) {
    return;
  }

  if (now_ms - _last_stall_ms <= static_cast<uint32_t>(STALL_COOLDOWN_MS)) {
    return;
  }

  const uint32_t combined_ticks = wheel_l_ticks + wheel_r_ticks;
  const uint32_t tick_delta = combined_ticks - _wheel_ref_ticks;
  const uint32_t window_ms = wheel_window_ms(motor);
  const bool window_elapsed = (now_ms - _wheel_ref_ms) >= window_ms;

  if (!window_elapsed) {
    return;
  }

  // Stall = motors commanded but wheel IR saw no rotation at all in the window.
  // Do NOT use yaw/IMU here — straight driving has ~0 yaw rate and the old
  // "anchor_stall" path false-triggered constantly while cruising a hallway.
  const bool no_wheel_motion =
      motor >= WHEEL_STALL_MOTOR && tick_delta < static_cast<uint32_t>(STALL_TICKS_REQUIRED);

  if (no_wheel_motion) {
    _stall = true;
    _last_stall_ms = now_ms;
    _drive_since = now_ms;
    Serial.printf("STALL no_ticks delta=%u win=%ums motor=%.2f\n",
                  (unsigned)tick_delta, (unsigned)window_ms, motor);
  }

  _wheel_ref_ticks = combined_ticks;
  _wheel_ref_ms = now_ms;
}

void MotionDetect::clear_events() {
  _stall = false;
}
