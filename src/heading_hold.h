#pragma once

#include <Arduino.h>

// Gyro yaw integrator + straight-line trim. Bias learning matches
// fleet_steering.py HeadingHold (stationary EMA, boot cal gate).

class HeadingHold {
 public:
  void begin();
  void reset_angle();
  /** Seed bias from boot average (optional — update() refines while still). */
  void seed_bias(float yaw_rate_rad_s);
  void disarm();

  /** Call every IMU sample: learns bias while stationary, integrates debiased yaw. */
  void update(float yaw_rate_rad_s, float dt_s, bool stationary);
  float correct(float yaw_rate_rad_s, float dt_s, float lin, float ang);

  bool ready() const { return _ready; }
  float cal_progress() const;
  float bias_rad_s() const { return _bias; }
  /** Debiased yaw rate (rad/s) from last update(). */
  float rate_rad_s() const { return _last_rate_rad_s; }
  float rate_dps() const { return _last_rate_rad_s * 57.2957795f; }
  float angle_deg() const { return _angle_z_deg; }
  float angle_deg_wrapped() const;

 private:
  static float wrap_deg(float deg);

  float _bias = 0;
  float _angle_z_deg = 0;
  float _lock_angle_deg = 0;
  float _last_rate_rad_s = 0;
  float _cal_elapsed_s = 0;
  uint32_t _bias_n = 0;
  bool _was_steering = false;
  bool _steering = false;
  bool _lock_valid = false;
  bool _ready = false;

#ifndef HEADING_BIAS_CAL_S
#define HEADING_BIAS_CAL_S 2.5f
#endif
#ifndef HEADING_BIAS_MIN_SAMPLES
#define HEADING_BIAS_MIN_SAMPLES 50
#endif
#ifndef HEADING_BIAS_MAX_GZ
#define HEADING_BIAS_MAX_GZ 0.35f
#endif
#ifndef HEADING_BIAS_STATIONARY_GZ
#define HEADING_BIAS_STATIONARY_GZ 0.8f
#endif

#ifndef HEADING_ADJ_MAP_DEG
#define HEADING_ADJ_MAP_DEG 3.0f
#endif
#ifndef HEADING_ADJ_OUT_NORM
#define HEADING_ADJ_OUT_NORM 0.012f
#endif
#ifndef HEADING_TRIM_MAX
#define HEADING_TRIM_MAX 0.012f
#endif

  static constexpr float BIAS_CAL_S = HEADING_BIAS_CAL_S;
  static constexpr uint32_t BIAS_MIN_SAMPLES = HEADING_BIAS_MIN_SAMPLES;
  static constexpr float BIAS_MAX_GZ = HEADING_BIAS_MAX_GZ;
  static constexpr float BIAS_STATIONARY_GZ = HEADING_BIAS_STATIONARY_GZ;
  static constexpr float ADJ_MAP_DEG = HEADING_ADJ_MAP_DEG;
  static constexpr float ADJ_OUT_NORM = HEADING_ADJ_OUT_NORM;
  static constexpr float TRIM_MAX = HEADING_TRIM_MAX;
  static constexpr float ANG_ON = 0.04f;
  static constexpr float ANG_OFF = 0.025f;
  static constexpr float LIN_THRESH = 0.05f;
};
