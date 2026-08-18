#include "rover_sonar.h"

#include <math.h>

#include "rover_pca9685.h"
#include "rover_pins_s3.h"
#include "rover_diag.h"
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
#define SONAR_GLANCE_PERIOD_MS 2200
#endif
#ifndef SONAR_BOOT_PAN_MS
#define SONAR_BOOT_PAN_MS 200
#endif
// PCA9685 outputs a 50 Hz (20 ms) PWM cycle in hardware. Writing a new
// ON/OFF register value more than once per cycle risks landing the write
// right as the chip's internal counter wraps, tearing that cycle's pulse —
// a real, constant glitch, not a "cheap chip" limitation. Throttle well
// clear of that: 33 ms (~30 Hz) never lands twice in the same 20 ms window,
// and at the wiggle's peak angular rate this is still a <1° step — smooth
// to the eye.
#ifndef SONAR_PAN_WRITE_MS
#define SONAR_PAN_WRITE_MS 33
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

void RoverSonar::set_pan_scan_enabled(bool enabled) {
  if (enabled == _pan_scan_enabled) {
    return;
  }
  _pan_scan_enabled = enabled;
  if (!enabled) {
    _wiggle_t0_ms = millis();
  }
}

float RoverSonar::cal_snap_pan(uint8_t idx) const {
  return (idx < _cal_snap_count) ? _cal_snap_pan[idx] : -1.0f;
}

float RoverSonar::cal_snap_range(uint8_t idx) const {
  return (idx < _cal_snap_count) ? _cal_snap_range[idx] : -1.0f;
}

void RoverSonar::service_cal_capture() {
  if (!_cal_capture_pending) {
    return;
  }
  _cal_capture_pending = false;
  if (_cal_snap_count >= 4) {
    return;
  }
  const float rng = read_range();
  if (rng > 0.0f) {
    _last_range_m = rng;
  }
  _cal_snap_pan[_cal_snap_count] = _cal_capture_pan;
  _cal_snap_range[_cal_snap_count] = (rng > 0.0f) ? rng : _last_range_m;
  ++_cal_snap_count;
}

void RoverSonar::set_cal_sweep(bool active) {
  if (!active) {
    _cal_sweep_active = false;
    _cal_step = 0;
    _cal_step_ms = 0;
    _cal_capture_pending = false;
    return;
  }
  if (_cal_sweep_active) {
    return;
  }
  _cal_sweep_active = true;
  _cal_step = 0;
  _cal_step_ms = 0;
  _cal_snap_count = 0;
  _cal_capture_pending = false;
  for (uint8_t i = 0; i < 4; i++) {
    _cal_snap_pan[i] = center_pan();
    _cal_snap_range[i] = -1.0f;
  }
  rover_diag_ckpt(kCkptCalStart);
}

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
  return (m > 0.01f && m < 4.35f) ? m : -1.0f;
}

void RoverSonar::update_pan_wiggle(uint32_t now_ms) {
  if (!_pca) {
    return;
  }

  if (_cal_sweep_active) {
    static constexpr float kCalAngles[] = {90.0f, 180.0f, 0.0f, 90.0f};
    static constexpr uint8_t kCalCount = 4;
#ifndef SONAR_CAL_DWELL_MS
#define SONAR_CAL_DWELL_MS 900
#endif
    if (_cal_step >= kCalCount) {
      _cal_sweep_active = false;
      rover_diag_ckpt(kCkptPanCalDone, _cal_snap_count);
      return;
    }
    if (_cal_step_ms == 0) {
      if (now_ms - _last_pan_write_ms < SONAR_PAN_WRITE_MS) {
        return;
      }
      _last_pan_write_ms = now_ms;
      set_pan(kCalAngles[_cal_step]);
      _cal_step_ms = now_ms;
      rover_diag_ckpt(kCkptCalPanMove, static_cast<uint32_t>(kCalAngles[_cal_step]));
      return;
    }
    if (now_ms - _cal_step_ms < static_cast<uint32_t>(SONAR_CAL_DWELL_MS)) {
      return;
    }
    _cal_capture_pan = kCalAngles[_cal_step];
    _cal_capture_pending = true;
    rover_diag_ckpt(kCkptCalDwellEnd, static_cast<uint32_t>(kCalAngles[_cal_step]));
    _cal_step++;
    _cal_step_ms = 0;
    return;
  }

  if (now_ms - _last_pan_write_ms < SONAR_PAN_WRITE_MS) {
    return;
  }

  if (!_pan_scan_enabled) {
    const float center = center_pan();
    if (fabsf(_pan_deg - center) > 0.5f) {
      _last_pan_write_ms = now_ms;
      set_pan(center);
    }
    return;
  }

  _last_pan_write_ms = now_ms;
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

  // Pan wiggle is driven by a dedicated task now (see update_pan_wiggle
  // doc comment) so it keeps a steady cadence regardless of how long this
  // tick() call (and the rest of the main loop) takes.

  if (now_ms - _last_read_ms >= static_cast<uint32_t>(SONAR_FORWARD_READ_MS)) {
    _last_read_ms = now_ms;
    const float rng = read_range();
    if (rng > 0.0f) {
      _last_range_m = rng;
      note_glance_sample(_pan_deg, rng);
      const float center = center_pan();
      if (fabsf(_pan_deg - center) <= SONAR_GLANCE_MAG_DEG * 0.45f) {
        _last_forward_range_m = rng;
        push_range_sample(rng);
      }
    }
  }
  if (_cal_capture_pending) {
    const uint8_t before = _cal_snap_count;
    service_cal_capture();
    if (_cal_snap_count > before) {
      rover_diag_ckpt(kCkptCalCapture, static_cast<uint32_t>(_cal_snap_pan[_cal_snap_count - 1]));
    }
  } else {
    service_cal_capture();
  }

  const bool forward = cmd_lin > 0.04f;
  _braking = false;
  const float fwd_rng = _last_forward_range_m;
  if (forward && fwd_rng > 0.0f) {
    if (fwd_rng < SONAR_STOP_M || (closing_trend() && fwd_rng < SONAR_AVOID_M)) {
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
