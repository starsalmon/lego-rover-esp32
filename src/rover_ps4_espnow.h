#pragma once

#include <stdint.h>

struct RoverPs4DriveOut {
  bool override_ros = false;
  bool failsafe = false;
  float linear_x = 0.0f;
  float angular_z = 0.0f;
};

struct RoverPs4LinkStatus {
  bool espnow_ready = false;
  bool linked = false;
  bool manual = false;
  bool ctrl_ok = false;
  bool speed_low = false;
  bool steer_low = false;
  uint8_t channel = 0;
  uint32_t last_pkt_age_ms = 0;
  uint32_t ack_tx = 0;
  uint32_t rx_count = 0;
};

struct RoverPs4Hooks {
  void (*apply_twist)(float linear_x, float angular_z);
  void (*motor_stop)(void);
  bool (*drive_cut)(void);
};

#if defined(ROVER_MICROROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

void rover_ps4_set_hooks(const RoverPs4Hooks* hooks);
void rover_ps4_espnow_begin();
void rover_ps4_espnow_tick(uint32_t now_ms);
RoverPs4DriveOut rover_ps4_espnow_poll(uint32_t now_ms);
bool rover_ps4_manual_active();
bool rover_ps4_straight_eligible(float* linear_out);
void rover_ps4_link_status(RoverPs4LinkStatus* out);

#else

inline void rover_ps4_set_hooks(const RoverPs4Hooks*) {}
inline void rover_ps4_espnow_begin() {}
inline void rover_ps4_espnow_tick(uint32_t) {}
inline RoverPs4DriveOut rover_ps4_espnow_poll(uint32_t) { return {}; }
inline bool rover_ps4_manual_active() { return false; }
inline bool rover_ps4_straight_eligible(float*) { return false; }
inline void rover_ps4_link_status(RoverPs4LinkStatus* out) {
  if (out) {
    *out = {};
  }
}

#endif