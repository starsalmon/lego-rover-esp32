#include "heading_hold.h"
#include <math.h>

static constexpr float RAD2DEG = 57.2957795f;

void HeadingHold::begin() {
  _bias = 0;
  reset_angle();
}

void HeadingHold::reset_angle() {
  _angle_z_deg = 0;
  _lock_angle_deg = 0;
  _was_steering = false;
  _steering = false;
  _lock_valid = false;
}

void HeadingHold::calibrate_bias(float gz) {
  _bias = gz;
}

void HeadingHold::disarm() {
  _was_steering = false;
  _steering = false;
  _lock_valid = false;
}

void HeadingHold::integrate_gyro(float yaw_rate, float dt) {
  if (dt <= 0) {
    return;
  }
  const float rate_dps = (yaw_rate - _bias) * RAD2DEG;
  _angle_z_deg += rate_dps * dt;
}

float HeadingHold::correct(float gz, float dt, float lin, float ang) {
  (void)gz;
  (void)dt;

  if (fabsf(ang) > ANG_ON) {
    _steering = true;
  } else if (fabsf(ang) < ANG_OFF) {
    _steering = false;
  }

  if (fabsf(lin) < LIN_THRESH) {
    disarm();
    return 0;
  }

  if (_steering) {
    _was_steering = true;
    return 0;
  }

  if (_was_steering || !_lock_valid) {
    _lock_angle_deg = _angle_z_deg;
    _lock_valid = true;
    _was_steering = false;
  }

  const float err_deg = _angle_z_deg - _lock_angle_deg;
  const float trim = (err_deg / ADJ_MAP_DEG) * ADJ_OUT_NORM;
  return constrain(trim, -TRIM_MAX, TRIM_MAX);
}
