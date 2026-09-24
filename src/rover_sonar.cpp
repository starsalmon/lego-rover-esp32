#include "rover_sonar.h"

#include <math.h>

#include "rover_pca9685.h"
#include "rover_pins_s3.h"
#include "rover_diag.h"
#include "ultrasonic.h"

#ifndef SONAR_BOOT_PAN_MS
#define SONAR_BOOT_PAN_MS 200
#endif
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
  if (_cal_snap_count >= 12) {
    return;
  }
  const float rng = read_range();
  if (rng > 0.0f) {
    _last_range_m = rng;
  }
  _cal_snap_pan[_cal_snap_count] = _cal_capture_pan;
  _cal_snap_range[_cal_snap_count] = (rng > 0.0f) ? rng : _last_range_m;
  ++_cal_snap_count;

  // Advance sweep step ONLY once a capture has been stored, so the pan task
  // cannot move to the next angle before the reading is taken.
  _cal_step++;
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
  for (uint8_t i = 0; i < 12; i++) {
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

void RoverSonar::set_side_hint(float left_m, float right_m) {
  _hint_left_m = left_m;
  _hint_right_m = right_m;
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
    // Multi-angle scan (not just left/right/center) to help pick a safe drive-out.
    static constexpr float kCalAngles[] = {90.0f, 135.0f, 180.0f, 135.0f, 90.0f, 45.0f, 0.0f, 45.0f, 90.0f};
    static constexpr uint8_t kCalCount = sizeof(kCalAngles) / sizeof(kCalAngles[0]);
#ifndef SONAR_CAL_DWELL_MS
#define SONAR_CAL_DWELL_MS 160
#endif
    if (_cal_step >= kCalCount) {
      _cal_sweep_active = false;
      rover_diag_ckpt(kCkptPanCalDone, _cal_snap_count);
      return;
    }

    // Critical: do not advance pan while the main loop is capturing a sample for
    // the current angle. Otherwise range reads get attributed to the wrong pan.
    if (_cal_capture_pending) {
      return;
    }

    if (_cal_step_ms == 0) {
      if (now_ms - _last_pan_write_ms < SONAR_PAN_WRITE_MS) {
        return;
      }
      _last_pan_write_ms = now_ms;

      // Servo is not instantaneous. Dwell must cover travel + settle.
      // Use a bench-calibrated ms/deg model (and always enforce a minimum).
      const float delta = fabsf(kCalAngles[_cal_step] - _pan_deg);
      const float dwell = fmaxf(static_cast<float>(SONAR_CAL_DWELL_MS), delta * _cal_ms_per_deg);
      _cal_dwell_ms = static_cast<uint16_t>(
          constrain(static_cast<int>(lroundf(dwell)), static_cast<int>(SONAR_CAL_DWELL_MS), 360));

      set_pan(kCalAngles[_cal_step]);
      _cal_step_ms = now_ms;
      rover_diag_ckpt(kCkptCalPanMove, static_cast<uint32_t>(kCalAngles[_cal_step]));
      return;
    }
    if (now_ms - _cal_step_ms < static_cast<uint32_t>(_cal_dwell_ms)) {
      return;
    }
    _cal_capture_pan = kCalAngles[_cal_step];
    _cal_capture_pending = true;
    rover_diag_ckpt(kCkptCalDwellEnd, static_cast<uint32_t>(kCalAngles[_cal_step]));
    // Step advance happens only after capture completes in service_cal_capture().
    // This keeps pan fixed at the requested angle until a sample is taken.
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
  if (hold_pan_wiggle()) {
    if (fabsf(_pan_deg - center) > 0.5f) {
      set_pan(center);
    }
    return;
  }
  const float t = static_cast<float>((now_ms - _wiggle_t0_ms) % SONAR_GLANCE_PERIOD_MS) /
                  static_cast<float>(SONAR_GLANCE_PERIOD_MS);
  const float wiggle = sinf(t * 2.0f * static_cast<float>(M_PI)) * SONAR_GLANCE_MAG_DEG;
  // Look a bit toward the closer side ToF so the cone isn't staring at empty
  // space while he slides into a wall.
  float bias = 0.0f;
  const float hl = _hint_left_m;
  const float hr = _hint_right_m;
  if (hl > 0.05f && hr > 0.05f) {
    bias = (hr - hl) * 40.0f;
    if (bias > 18.0f) {
      bias = 18.0f;
    } else if (bias < -18.0f) {
      bias = -18.0f;
    }
  } else if (hl > 0.05f && hl < 0.35f && hr < 0.0f) {
    bias = 12.0f;
  } else if (hr > 0.05f && hr < 0.35f && hl < 0.0f) {
    bias = -12.0f;
  }
  const float target = center + bias + wiggle;
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
  // pan 0 = chassis right, 90 = aft, 180 = chassis left (after invert).
  if (pan_deg <= center - SONAR_GLANCE_MAG_DEG * 0.5f) {
    _glance_right_m = range_m;
  } else if (pan_deg >= center + SONAR_GLANCE_MAG_DEG * 0.5f) {
    _glance_left_m = range_m;
  } else {
    _last_forward_range_m = range_m;
    push_range_sample(range_m);
  }
}

float RoverSonar::effective_forward_m() const {
  float best = _last_forward_range_m;
  if (_glance_left_m > 0.0f) {
    best = (best > 0.0f) ? fminf(best, _glance_left_m) : _glance_left_m;
  }
  if (_glance_right_m > 0.0f) {
    best = (best > 0.0f) ? fminf(best, _glance_right_m) : _glance_right_m;
  }
  if (_last_range_m > 0.0f) {
    best = (best > 0.0f) ? fminf(best, _last_range_m) : _last_range_m;
  }
  return best;
}

float RoverSonar::brake_range_m() const {
  // Only trust live aft cone for reverse clearance.
  float r = _last_forward_range_m;
  const float center = center_pan();
  const bool at_center = fabsf(_pan_deg - center) <= SONAR_GLANCE_MAG_DEG * 0.40f;
  if (at_center && _last_range_m > 0.0f) {
    r = (r > 0.0f) ? fminf(r, _last_range_m) : _last_range_m;
  }
  return r;
}

bool RoverSonar::hold_pan_wiggle() const {
  if (_braking) {
    return true;
  }
  const float r = brake_range_m();
  return r > 0.0f && r < SONAR_WIGGLE_HOLD_M;
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

  const bool reverse = cmd_lin < -0.04f;
  if (_cal_sweep_active || reverse) {
    if (now_ms - _last_read_ms >= static_cast<uint32_t>(SONAR_FORWARD_READ_MS)) {
      _last_read_ms = now_ms;
      const float rng = read_range();
      if (rng > 0.0f) {
        _last_range_m = rng;
        note_glance_sample(_pan_deg, rng);
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

  const bool turning = fabsf(cmd_ang) > 0.03f;
  _braking = false;
  const float aft_rng = _last_forward_range_m;
  if (reverse && aft_rng > 0.0f) {
    if (aft_rng < SONAR_STOP_M || (closing_trend() && aft_rng < SONAR_AVOID_M)) {
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
  if (turning && reverse && aft_rng > 0.0f && aft_rng < SONAR_SPIN_STOP_M) {
    *out_lin = 0.0f;
    return true;
  }
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
