#include "rover_sonar.h"

#include <math.h>

#include "rover_pca9685.h"
#include "rover_pins_s3.h"
#include "ultrasonic.h"

#ifndef SONAR_STOP_M
#define SONAR_STOP_M 0.20f
#endif
#ifndef SONAR_AVOID_M
#define SONAR_AVOID_M 0.55f
#endif
#ifndef SONAR_CLOSE_TREND_M
#define SONAR_CLOSE_TREND_M 0.04f
#endif
#ifndef SONAR_FORWARD_READ_MS
#define SONAR_FORWARD_READ_MS 80
#endif
#ifndef SONAR_GLANCE_MAG_DEG
#define SONAR_GLANCE_MAG_DEG 12.0f
#endif
#ifndef SONAR_GLANCE_PERIOD_MS
#define SONAR_GLANCE_PERIOD_MS 2800
#endif
#ifndef SONAR_BOOT_PAN_MS
#define SONAR_BOOT_PAN_MS 200
#endif

namespace {

float clamp_pan(float deg) {
  if (deg < static_cast<float>(SONAR_PAN_MIN_DEG)) {
    return static_cast<float>(SONAR_PAN_MIN_DEG);
  }
  if (deg > static_cast<float>(SONAR_PAN_MAX_DEG)) {
    return static_cast<float>(SONAR_PAN_MAX_DEG);
  }
  return deg;
}

float center_pan() { return static_cast<float>(SONAR_PAN_CENTER_DEG); }

float hw_pan(float logical_deg) {
#if SONAR_PAN_INVERT
  logical_deg = 2.0f * static_cast<float>(SONAR_PAN_CENTER_DEG) - logical_deg;
#endif
  logical_deg += static_cast<float>(SONAR_PAN_TRIM_DEG);
  return clamp_pan(logical_deg);
}

}  // namespace

bool RoverSonar::begin(Ultrasonic* sonar, RoverPca9685* pca) {
  _sonar = sonar;
  _pca = pca;
  _pan_deg = center_pan();
  _wiggle_t0_ms = millis();
  if (_pca) {
    set_pan(_pan_deg);
  }
  return _sonar != nullptr;
}

void RoverSonar::set_aux_scan(RoverMcpIr* ir) { _mcp_ir = ir; }

void RoverSonar::set_pan(float deg) {
  _pan_deg = clamp_pan(deg);
  if (_pca) {
    _pca->setAngle(SONAR_SERVO_CHANNEL, hw_pan(_pan_deg), static_cast<float>(SONAR_PAN_MIN_DEG),
                   static_cast<float>(SONAR_PAN_MAX_DEG), 500, 2500);
  }
}

float RoverSonar::read_range() {
  if (!_sonar) {
    return -1.0f;
  }
  const float m = _sonar->read_range_m();
  return (m > 0.01f && m < 4.0f) ? m : -1.0f;
}

void RoverSonar::update_pan_wiggle(uint32_t now_ms) {
  if (!_pca) {
    return;
  }
  const float center = center_pan();
  const float t = static_cast<float>((now_ms - _wiggle_t0_ms) % SONAR_GLANCE_PERIOD_MS) /
                  static_cast<float>(SONAR_GLANCE_PERIOD_MS);
  const float wiggle = sinf(t * 2.0f * static_cast<float>(M_PI)) * SONAR_GLANCE_MAG_DEG;
  const float target = center + wiggle;
  set_pan(target);
}

void RoverSonar::push_range_sample(float range_m) {
  if (range_m <= 0.0f) {
    return;
  }
  for (uint8_t i = 2; i > 0; --i) {
    _range_ring[i] = _range_ring[i - 1];
  }
  _range_ring[0] = range_m;
  if (_range_ring_n < 3) {
    ++_range_ring_n;
  }
}

bool RoverSonar::closing_trend() const {
  if (_range_ring_n < 3) {
    return false;
  }
  return (_range_ring[0] + SONAR_CLOSE_TREND_M < _range_ring[1]) &&
         (_range_ring[1] + SONAR_CLOSE_TREND_M < _range_ring[2]);
}

void RoverSonar::note_glance_sample(float pan_deg, float range_m) {
  if (range_m <= 0.0f) {
    return;
  }
  const float center = center_pan();
  if (pan_deg <= center - SONAR_GLANCE_MAG_DEG * 0.5f) {
    _glance_left_m = range_m;
  } else if (pan_deg >= center + SONAR_GLANCE_MAG_DEG * 0.5f) {
    _glance_right_m = range_m;
  }
}

void RoverSonar::boot_full_sweep() {
  if (!_pca) {
    return;
  }
  const float center = center_pan();
  set_pan(static_cast<float>(SONAR_PAN_MIN_DEG));
  delay(SONAR_BOOT_PAN_MS);
  set_pan(static_cast<float>(SONAR_PAN_MAX_DEG));
  delay(SONAR_BOOT_PAN_MS);
  set_pan(center);
  delay(SONAR_BOOT_PAN_MS);
}

bool RoverSonar::tick(uint32_t now_ms, float cmd_lin, float cmd_ang, bool body_moving,
                      float yaw_deg, float* out_lin, float* out_ang) {
  (void)body_moving;
  (void)yaw_deg;

  update_pan_wiggle(now_ms);

  if (now_ms - _last_read_ms >= SONAR_FORWARD_READ_MS) {
    _last_read_ms = now_ms;
    const float rng = read_range();
    if (rng > 0.0f) {
      _last_range_m = rng;
      push_range_sample(rng);
      note_glance_sample(_pan_deg, rng);
    }
  }

  const bool forward = cmd_lin > 0.04f;
  _braking = false;
  if (forward && _last_range_m > 0.0f) {
    if (_last_range_m < SONAR_STOP_M ||
        (closing_trend() && _last_range_m < SONAR_AVOID_M)) {
      _braking = true;
    }
  }

  if (_braking) {
    *out_lin = 0.0f;
    *out_ang = cmd_ang;
    return true;
  }

  *out_lin = cmd_lin;
  *out_ang = cmd_ang;
  return false;
}

bool RoverSonar::take_escape_event(uint8_t* out_id) {
  (void)out_id;
  return false;
}

void RoverSonar::abort_escape(uint32_t now_ms) {
  (void)now_ms;
  _braking = false;
}

void RoverSonar::trigger_escape(uint32_t now_ms) {
  (void)now_ms;
}
