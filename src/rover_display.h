#pragma once

#include <Arduino.h>

struct RoverUiData {
  float bat_v = 0.0f;
  bool bat_low = false;
  bool bat_critical = false;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float yaw_rate = 0.0f;
  float cmd_lin = 0.0f;
  float cmd_ang = 0.0f;
  float motor_l = 0.0f;
  float motor_r = 0.0f;
  int link_state = 0;  // 0 wait, 1 connect, 2 up, 3 lost
  bool estop = false;
  bool stall = false;
  bool bump = false;
  bool session_active = false;  // Pi session running (from /rover/session)
  bool pi_bridge_live = false;  // recent /rover/session heartbeat from rover-main
  bool mode_menu = false;
  bool power_menu = false;
  uint8_t drive_mode = 0;  // RoverDriveMode
  uint8_t power_action = 0;  // RoverPowerAction
  uint32_t menu_hint_until_ms = 0;  // flash "tap Go" after closing menu
  uint32_t key_flash_until_ms = 0;  // brief border flash on any key
  uint8_t tap_progress = 0;         // 0-N while counting multi-tap (power menu)
  const char *mode = "boot";
  int ota_state = 0;       // OtaState: 0 off, 1 connecting, 2 ready, 3 failed
  const char *wifi_ip = "";
  bool ir_enabled = false;   // MCP IR build + attempted init
  bool ir_ok = false;        // both MCP23008 responded
  bool ir_in_ok = false;     // front bumper MCP @0x20
  bool ir_out_ok = false;    // body MCP @0x21 (aux + wheels)
  uint8_t ir_inputs = 0;     // GPA0-4 snapshot (bit = HIGH)
  bool ir_front_hit = false; // recent delta detect (held ~0.5s for display)
  bool ir_wheels_on = false;
  bool ir_front_emit = false;  // front emitter gates enabled (555 carrier on PCB)
  bool ir_body_aux_emit = false;  // body aux gate LOW (BC557 active-low on)
  uint8_t ir_body_iodir = 0;
  uint8_t ir_body_olat = 0;
  uint8_t ir_body_gpio = 0;
  uint8_t ir_front_iodir = 0;
  uint8_t ir_front_olat = 0;
  uint8_t ir_front_gpio = 0;
  uint8_t ir_i2c_count = 0;
  uint8_t ir_i2c_addrs[8] = {};  // devices seen on last bus scan
  int ir_sda_pin = -1;
  int ir_scl_pin = -1;
  bool sonar_enabled = false;
  float sonar_pan_deg = 90.0f;
  float sonar_range_m = -1.0f;
  bool sonar_cal_active = false;
  uint8_t sonar_cal_snap_count = 0;
  float sonar_cal_snap_pan[4] = {};
  float sonar_cal_snap_range[4] = {};
  uint32_t sonar_cal_summary_until_ms = 0;
  bool tof_enabled = false;
  bool tof_left_ok = false;
  bool tof_right_ok = false;
  float tof_left_m = -1.0f;
  float tof_right_m = -1.0f;
  const char *boot_reason = nullptr;
  uint32_t boot_reason_until_ms = 0;
};

class RoverDisplay {
 public:
  bool begin();
  void draw(const RoverUiData &data, uint32_t now_ms);

 private:
  bool _ok = false;
  bool _sprite_ok = false;
  uint32_t _last_draw = 0;
  RoverUiData _last{};
  char _last_mode[24] = {};

  bool _changed(const RoverUiData &data) const;

  uint8_t _last_face = 0xFF;
};
