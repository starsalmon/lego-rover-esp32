#pragma once

#include <Arduino.h>

/** ESP-local explore (no Pi / ROS cmd_vel). Sonar escape still overrides in main loop. */
class RoverWander {
 public:
  void begin();
  void reset();
  bool tick(uint32_t now_ms, float* out_lin, float* out_ang);

  float cmd_lin() const { return _lin; }
  float cmd_ang() const { return _ang; }

 private:
  uint32_t _mode_until_ms = 0;
  float _lin = 0.0f;
  float _ang = 0.0f;
  void pick_next(uint32_t now_ms);
};
