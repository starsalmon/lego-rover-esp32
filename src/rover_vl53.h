#pragma once

#include <stdint.h>

/** Dual VL53L0X/L1X side rangefinders on shared I2C (XSHUT on GPIO 17/18). */
class RoverVl53 {
 public:
  bool begin(int xshut_left, int xshut_right);
  void poll();

  bool ok() const { return _ok; }
  bool left_ok() const { return _left_ok; }
  bool right_ok() const { return _right_ok; }
  float range_left_m() const { return _left_m; }
  float range_right_m() const { return _right_m; }

 private:
  void set_xshut(int pin, bool on);

  bool _ok = false;
  bool _left_ok = false;
  bool _right_ok = false;
  int _xshut_l = -1;
  int _xshut_r = -1;
  float _left_m = -1.0f;
  float _right_m = -1.0f;
};
