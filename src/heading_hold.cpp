#include "heading_hold.h"

#include <math.h>

void HeadingHold::begin() {
  _bias = 0;
  _bias_n = 0;
  _cal_elapsed_s = 0;
  _ready = false;
  _last_rate_rad_s = 0;
  reset_angle();
}

void HeadingHold::reset_angle() {
  _angle_z_deg = 0;
  _lock_angle_deg = 0;
  _was_steering = false;
  _steering = false;
  _lock_valid = false;
}

void HeadingHold::seed_bias(float yaw_rate_rad_s) {
  _bias = yaw_rate_rad_s;
  _bias_n = 1;
}

void HeadingHold::disarm() {
  _was_steering = false;
  _steering = false;
  _lock_valid = false;
}

float HeadingHold::wrap_deg(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

float HeadingHold::angle_deg_wrapped() const { return wrap_deg(_angle_z_deg); }

float HeadingHold::cal_progress() const {
  if (_ready) {
    return 1.0f;
  }
  const float sample_frac =
      static_cast<float>(_bias_n) / static_cast<float>(BIAS_MIN_SAMPLES > 0 ? BIAS_MIN_SAMPLES : 1);
  const float time_frac = BIAS_CAL_S > 0.0f ? (_cal_elapsed_s / BIAS_CAL_S) : 1.0f;
  const float p = fminf(sample_frac, time_frac);
  return p > 1.0f ? 1.0f : p;
}

void HeadingHold::update(float yaw_rate_rad_s, float dt_s, bool stationary) {
  if (dt_s > 0.0f) {
    _cal_elapsed_s += dt_s;
  }

  const float raw_abs = fabsf(yaw_rate_rad_s);
  if (!_ready) {
    if (raw_abs > BIAS_MAX_GZ) {
      _cal_elapsed_s = 0;
      if (_bias_n > 15) {
        _bias_n -= 15;
      } else {
        _bias_n = 0;
      }
    } else if (stationary && raw_abs < BIAS_STATIONARY_GZ) {
      ++_bias_n;
      _bias += (yaw_rate_rad_s - _bias) / static_cast<float>(_bias_n);
    }
    if (_cal_elapsed_s >= BIAS_CAL_S && _bias_n >= BIAS_MIN_SAMPLES) {
      _ready = true;
      _lock_angle_deg = _angle_z_deg;
      _lock_valid = true;
    }
  } else if (stationary && raw_abs < BIAS_STATIONARY_GZ) {
    ++_bias_n;
    _bias += (yaw_rate_rad_s - _bias) / static_cast<float>(_bias_n);
  }

  _last_rate_rad_s = yaw_rate_rad_s - _bias;
  if (dt_s > 0.0f) {
    _angle_z_deg += _last_rate_rad_s * 57.2957795f * dt_s;
  }
}

float HeadingHold::correct(float yaw_rate_rad_s, float dt_s, float lin, float ang) {
  (void)yaw_rate_rad_s;
  (void)dt_s;

  if (!_ready) {
    return 0;
  }

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

  const float err_deg = wrap_deg(_angle_z_deg - _lock_angle_deg);
  const float trim = (err_deg / ADJ_MAP_DEG) * ADJ_OUT_NORM;
  return constrain(trim, -TRIM_MAX, TRIM_MAX);
}
