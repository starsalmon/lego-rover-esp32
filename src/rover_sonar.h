#pragma once

#include <Arduino.h>

class RoverMcpIr;
class RoverPca9685;
class Ultrasonic;

/** Pan sonar (aft + sides): cruise sweep, publish range, reverse hard-brake.
 *  Nose depth is VL53L8CX. Wander / escape lives on dockerhost. */
class RoverSonar {
 public:
  bool begin(Ultrasonic* sonar, RoverPca9685* pca);

  /** True when hard-braking reverse (brain still owns steering). */
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

  /** Scan sweep: multi-angle sample across 0..180°, then return to center. */
  void set_cal_sweep(bool active);
  bool cal_sweep_active() const { return _cal_sweep_active; }
  uint8_t cal_step() const { return _cal_step; }
  uint8_t cal_snap_count() const { return _cal_snap_count; }
  float cal_snap_pan(uint8_t idx) const;
  float cal_snap_range(uint8_t idx) const;
  /** Main loop: sample range when a cal dwell step completes. */
  void service_cal_capture();

  /** When enabled (explore / bench cal), run the cruise pan wiggle.
   *  When disabled (standby idle), hold center — independent of wheel motion. */
  void set_pan_scan_enabled(bool enabled);
  /** Directly command the pan (bench/calibration). */
  void set_pan_deg(float deg) { set_pan(deg); }
  /** Cal-sweep dwell model (ms per degree), inferred from bench calibration. */
  void set_cal_ms_per_deg(float ms_per_deg) { _cal_ms_per_deg = ms_per_deg; }
  float cal_ms_per_deg() const { return _cal_ms_per_deg; }
  void set_side_hint(float left_m, float right_m);

  /** Pause cruise wiggle near obstacles — hold pan at center. */
  bool hold_pan_wiggle() const;

  /** Drives the smooth sine cruise-wiggle. Called from a dedicated
   *  FreeRTOS task on its own fixed schedule, NOT from the main loop —
   *  the main loop's period varies with display/network work (10-150ms),
   *  and driving the servo target off that jitter is what caused visible
   *  jerkiness even though the PCA9685 write itself was atomic. */
  void update_pan_wiggle(uint32_t now_ms);

 private:
  void set_pan(float deg);
  float read_range();
  void push_range_sample(float range_m);
  bool closing_trend() const;
  void note_glance_sample(float pan_deg, float range_m);
  float effective_forward_m() const;
  /** Conservative forward range for braking — ignores off-center "open" glances. */
  float brake_range_m() const;

  Ultrasonic* _sonar = nullptr;
  RoverPca9685* _pca = nullptr;
  RoverMcpIr* _mcp_ir = nullptr;
  float _pan_deg = 90.0f;
  float _last_range_m = -1.0f;
  float _last_forward_range_m = -1.0f;
  float _glance_left_m = -1.0f;
  float _glance_right_m = -1.0f;
  volatile float _hint_left_m = -1.0f;
  volatile float _hint_right_m = -1.0f;
  bool _braking = false;
  uint32_t _last_read_ms = 0;
  uint32_t _wiggle_t0_ms = 0;
  uint32_t _last_pan_write_ms = 0;
  volatile bool _pan_scan_enabled = false;
  volatile bool _cal_sweep_active = false;
  uint8_t _cal_step = 0;
  uint32_t _cal_step_ms = 0;
  volatile bool _cal_capture_pending = false;
  volatile float _cal_capture_pan = 90.0f;
  uint16_t _cal_dwell_ms = 160;
  float _cal_ms_per_deg = 2.4f;  // conservative default; bench-calibrate for your servo
  uint8_t _cal_snap_count = 0;
  float _cal_snap_pan[12] = {90.0f};
  float _cal_snap_range[12] = {-1.0f};
  float _range_ring[3] = {-1.0f, -1.0f, -1.0f};
  uint8_t _range_ring_n = 0;
};
