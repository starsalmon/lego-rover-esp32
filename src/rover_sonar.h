#pragma once

#include <Arduino.h>

class RoverMcpIr;
class RoverPca9685;
class Ultrasonic;

/** Pan sonar: smooth cruise wiggle, publish range, emergency brake only.
 *  Wander / escape lives on the Pi (autonomous_explore.py), not here. */
class RoverSonar {
 public:
  bool begin(Ultrasonic* sonar, RoverPca9685* pca);

  /** True when hard-braking forward motion (Pi still owns steering). */
  bool tick(uint32_t now_ms, float cmd_lin, float cmd_ang, bool body_moving, float yaw_deg,
            float* out_lin, float* out_ang);

  float pan_deg() const { return _pan_deg; }
  float range_m() const { return _last_range_m; }
  bool avoid_active() const { return _braking; }
  int phase_code() const { return _braking ? 1 : 0; }
  bool owns_pan() const { return false; }
  float glance_left_m() const { return _glance_left_m; }
  float glance_right_m() const { return _glance_right_m; }
  uint8_t glance_state() const { return 0; }

  float escape_turn_sign() const { return 0.0f; }
  float escape_best_deg() const { return _pan_deg; }
  float escape_best_m() const { return _last_range_m; }
  float escape_worst_deg() const { return _pan_deg; }
  float escape_worst_m() const { return _last_range_m; }
  uint8_t escape_event_id() const { return 0; }
  bool take_escape_event(uint8_t* out_id);
  void abort_escape(uint32_t now_ms);
  void trigger_escape(uint32_t now_ms);
  void set_aux_scan(RoverMcpIr* ir);
  void boot_full_sweep();

 private:
  void set_pan(float deg);
  float read_range();
  void update_pan_wiggle(uint32_t now_ms);
  void push_range_sample(float range_m);
  bool closing_trend() const;
  void note_glance_sample(float pan_deg, float range_m);

  Ultrasonic* _sonar = nullptr;
  RoverPca9685* _pca = nullptr;
  RoverMcpIr* _mcp_ir = nullptr;
  float _pan_deg = 90.0f;
  float _last_range_m = -1.0f;
  float _glance_left_m = -1.0f;
  float _glance_right_m = -1.0f;
  bool _braking = false;
  uint32_t _last_read_ms = 0;
  uint32_t _wiggle_t0_ms = 0;
  uint32_t _last_pan_write_ms = 0;
  float _range_ring[3] = {-1.0f, -1.0f, -1.0f};
  uint8_t _range_ring_n = 0;
};
