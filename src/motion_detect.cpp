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
#ifndef STALL_EXPECTED_TICKS_CRUISE
#define STALL_EXPECTED_TICKS_CRUISE 10
#endif
#ifndef STALL_SLIP_RATIO
#define STALL_SLIP_RATIO 0.12f
#endif
#ifndef ROVER_CRUISE_MAX_LIN
#define ROVER_CRUISE_MAX_LIN 0.15f
#endif

void MotionDetect::begin() {
  _lp_ax = 0.0f;
  _lp_ay = 0.0f;
  _lp_az = 1.0f;
  _drive_since = 0;
  _last_stall_ms = 0;
  _last_bump_ms = 0;
  _wheel_ref_ticks = 0;
  _wheel_ref_ms = 0;
  _last_ms = 0;
  _ticks_per_motor_s = 0.0f;
  _cal_windows = 0;
  _stall = false;
  _bump = false;
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
  _stall = false;
  _bump = false;

  const float motor = fmaxf(fabsf(motor_l), fabsf(motor_r));
  float dt = 0.0f;
  if (_last_ms != 0 && now_ms > _last_ms) {
    dt = (now_ms - _last_ms) / 1000.0f;
    if (dt > 0.08f) {
      dt = 0.08f;
    }
  }
  _last_ms = now_ms;

  if (dt > 0.0f) {
    const float a = BUMP_LP_ALPHA;
    const float b = 1.0f - a;
    _lp_ax = _lp_ax * a + ax * b;
    _lp_ay = _lp_ay * a + ay * b;
    _lp_az = _lp_az * a + az * b;
  }

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

  if (now_ms - _last_bump_ms > static_cast<uint32_t>(BUMP_COOLDOWN_MS)) {
    const float dx = ax - _lp_ax;
    const float dy = ay - _lp_ay;
    const float jerk = sqrtf(dx * dx + dy * dy);
    const float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (jerk >= BUMP_JERK_G && gyro_mag <= BUMP_GYRO_MAX) {
      _bump = true;
      _last_bump_ms = now_ms;
      Serial.printf("BUMP IMU jerk=%.2f g gyro=%.2f motor=%.2f\n", jerk, gyro_mag, motor);
    }
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

  // Stall only when the wheels are not turning. Expected tick count is learned
  // from real motion at this motor command (6 strips/rev), not a fixed guess.
  // A low-but-nonzero rate is slow driving, not a stall.
  const float window_s = static_cast<float>(window_ms) * 0.001f;
  if (tick_delta >= 2u && motor > 0.02f && window_s > 0.2f) {
    const float observed = static_cast<float>(tick_delta) / (window_s * motor);
    if (_cal_windows == 0) {
      _ticks_per_motor_s = observed;
    } else {
      _ticks_per_motor_s = 0.8f * _ticks_per_motor_s + 0.2f * observed;
    }
    if (_cal_windows < 255) {
      _cal_windows++;
    }
  }

  const bool no_wheel_motion =
      motor >= WHEEL_STALL_MOTOR && tick_delta == 0u;

  bool under_rate = false;
  uint32_t need = 0;
  if (_cal_windows >= 3 && _ticks_per_motor_s > 0.5f) {
    const float expected = _ticks_per_motor_s * motor * window_s;
    need = static_cast<uint32_t>(expected * 0.25f);
    if (need < 2u) {
      need = 2u;
    }
    under_rate = motor >= WHEEL_STALL_MOTOR && tick_delta < need && expected >= 4.0f;
  }

  if (no_wheel_motion || under_rate) {
    _stall = true;
    _last_stall_ms = now_ms;
    _drive_since = now_ms;
    Serial.printf("STALL %s delta=%u need>=%u win=%ums motor=%.2f cal=%.1f t/s\n",
                  no_wheel_motion ? "no_ticks" : "under_rate", (unsigned)tick_delta,
                  (unsigned)(no_wheel_motion ? 1u : need), (unsigned)window_ms, motor,
                  _ticks_per_motor_s);
  }

  _wheel_ref_ticks = combined_ticks;
  _wheel_ref_ms = now_ms;
}

void MotionDetect::clear_events() {
  _stall = false;
  _bump = false;
}
