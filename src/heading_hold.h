#pragma once

#include <Arduino.h>

// IMU heading hold — Arduino P-only (adj_map / adj_out), lock heading on steer release.
class HeadingHold {
 public:
  void begin();
  void reset_angle();
  void calibrate_bias(float gz);
  void disarm();
  void integrate_gyro(float yaw_rate, float dt);
  float correct(float gz, float dt, float lin, float ang);
  float angle_deg() const { return _angle_z_deg; }

 private:
  float _bias = 0;
  float _angle_z_deg = 0;
  float _lock_angle_deg = 0;
  bool _was_steering = false;
  bool _steering = false;
  bool _lock_valid = false;

#ifndef HEADING_ADJ_MAP_DEG
#define HEADING_ADJ_MAP_DEG 3.0f
#endif
#ifndef HEADING_ADJ_OUT_NORM
#define HEADING_ADJ_OUT_NORM 0.156f
#endif
#ifndef HEADING_TRIM_MAX
#define HEADING_TRIM_MAX 0.156f
#endif

  static constexpr float ADJ_MAP_DEG = HEADING_ADJ_MAP_DEG;
  static constexpr float ADJ_OUT_NORM = HEADING_ADJ_OUT_NORM;
  static constexpr float TRIM_MAX = HEADING_TRIM_MAX;
  static constexpr float ANG_ON = 0.04f;
  static constexpr float ANG_OFF = 0.025f;
  static constexpr float LIN_THRESH = 0.05f;
};
