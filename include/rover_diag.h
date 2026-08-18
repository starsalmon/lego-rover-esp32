#pragma once

#include <stdint.h>

/** Crash breadcrumbs in RTC RAM (survives watchdog/panic reboot, not power-off). */
enum RoverCkpt : uint32_t {
  kCkptNone = 0,
  kCkptBoot = 1,
  kCkptLoop = 10,
  kCkptCalStart = 100,
  kCkptCalPanMove = 110,
  kCkptCalDwellEnd = 120,
  kCkptCalDone = 130,
  kCkptCalCapture = 140,
  kCkptApplyDrive = 200,
  kCkptDisplayDraw = 210,
  kCkptPanTaskLoop = 300,
  kCkptPanCalDone = 390,
};

struct RoverDiagSample {
  uint32_t now_ms = 0;
  int link_state = 0;
  bool session = false;
  bool cal_active = false;
  uint8_t cal_step = 0;
  uint8_t cal_snaps = 0;
  float pan_deg = 0.0f;
  float range_m = -1.0f;
  float cmd_lin = 0.0f;
  float cmd_ang = 0.0f;
  float motor_l = 0.0f;
  float motor_r = 0.0f;
  float bat_v = 0.0f;
  float yaw_deg = 0.0f;
  float gz_dps = 0.0f;
  float htrim = 0.0f;
  float az_g = 0.0f;
  float tof_l_m = -1.0f;
  float tof_r_m = -1.0f;
  uint32_t pan_stack_words = 0;
};

/** Call once from setup() after Serial.begin. Logs prior crash checkpoint + reset reason. */
void rover_diag_begin(const char* reset_reason);

/** Call after WiFi is up so BOOT/prev_ckpt reaches UDP telem on dockerhost. */
void rover_diag_wifi_ready();

/** Safe from any task — no Serial, only RTC store. */
void rover_diag_ckpt(RoverCkpt id, uint32_t arg = 0);

/** Main loop only — structured 1 Hz line + optional events. */
void rover_diag_tick(const RoverDiagSample& sample);

/** Main loop only — immediate EVENT line (cal transitions, etc.). */
void rover_diag_event(const char* fmt, ...);

/** Optional: pan FreeRTOS task for stack watermark in DIAG line. */
void rover_diag_set_pan_task(void* task_handle);
