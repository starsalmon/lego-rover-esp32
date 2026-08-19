#pragma once

#include <stdint.h>

class RoverMcpIr;

/** Dual VL53L0X/L1X side rangefinders on shared I2C (XSHUT on body MCP GPA6/GPA7). */
class RoverVl53 {
 public:
  bool begin(RoverMcpIr* mcp);
  void poll();

  bool ok() const { return _ok; }
  bool left_ok() const { return _left_ok; }
  bool right_ok() const { return _right_ok; }
  float range_left_m() const { return _left_m; }
  float range_right_m() const { return _right_m; }

 private:
  void set_xshut(bool left_on, bool right_on);

  RoverMcpIr* _mcp = nullptr;
  bool _ok = false;
  bool _left_ok = false;
  bool _right_ok = false;
  float _left_m = -1.0f;
  float _right_m = -1.0f;
};
