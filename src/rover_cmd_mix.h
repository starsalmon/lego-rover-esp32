#pragma once

#include <Arduino.h>

/** Shared cmd_vel twist -> normalized wheel targets (heading trim + mix). */
struct RoverCmdMixPort {
  bool (*mpu_ok)();
  bool (*heading_lock)();
  void (*heading_unlock)();
  float (*heading_correct)(float yaw_rate, float dt, float lin, float ang);
  void (*heading_disarm)();
#if defined(ROVER_VL53L8)
  bool (*front_tof_ok)();
  float (*front_range_m)();
#endif
};

void rover_cmd_mix_set_port(const RoverCmdMixPort* port);
void rover_cmd_mix_reset_trim();
bool rover_cmd_mix_twist(float lin, float ang, float yaw_rate, float dt, bool sonar_override, float* out_l,
                         float* out_r);
float rover_cmd_mix_trim_out();