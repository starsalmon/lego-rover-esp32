#pragma once

#include <stdint.h>

/** VL53L8CX 8×8 nose ToF. LPn is body MCP GPA3 (not an ESP GPIO). */
class RoverVl53L8 {
 public:
  static constexpr uint8_t kCols = 8;

  bool begin();
  void poll();

  bool ok() const { return _ok; }
  float range_center_m() const { return _center_m; }
  float range_left_m() const { return _left_m; }
  float range_right_m() const { return _right_m; }
  /**
   * Closest thing that is not the floor, per column. Index 0 = robot-left.
   * Clear floor (or nothing closer than the floor) is reported as a long range.
   * <0 only when that column has no reading at all.
   */
  float column_m(uint8_t col) const {
    return (col < kCols) ? _col_m[col] : -1.0f;
  }
  bool close_ahead() const;

 private:
  bool _ok = false;
  float _center_m = -1.0f;
  float _left_m = -1.0f;
  float _right_m = -1.0f;
  float _col_m[kCols] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
};
