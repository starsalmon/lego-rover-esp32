#pragma once

#include <stdint.h>

#ifndef ROVER_DIAG_PERIOD_MS
#define ROVER_DIAG_PERIOD_MS 100
#endif

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
/** After WiFi up — includes reset reason (BOOT line is often pre-WiFi / lost). */
void rover_diag_wifi_ready(const char* reset_reason = nullptr);

/**
 * After WiFi up: if last reset was panic/WDT and a flash coredump exists,
 * UDP/Serial-emit PANIC_CORE + PANIC_BT lines (PC list) then erase the dump.
 * Safe no-op if no dump / not a panic reset. Called from rover_diag_wifi_ready().
 */
void rover_diag_emit_panic_coredump(const char* reset_reason);

/** Safe from any task — no Serial, only RTC store. */
void rover_diag_ckpt(RoverCkpt id, uint32_t arg = 0);

/** Structured DIAG line (target ~10 Hz; safe from imu_diag FreeRTOS task). */
void rover_diag_tick(const RoverDiagSample& sample);

/** Immediate EVENT line (cal transitions, etc.). Prefer main/connected contexts. */
void rover_diag_event(const char* fmt, ...);

/** Optional: pan FreeRTOS task for stack watermark in DIAG line. */
void rover_diag_set_pan_task(void* task_handle);

/**
 * Global WiFiUDP / lwIP serialization for DIAG + micro-ROS transport.
 * microros_main wires these to the recursive uros_mu. Transport read/write and
 * DIAG UDP must both take this lock — 15ms fail-fast was not enough.
 */
using RoverDiagNetTake = bool (*)(uint32_t timeout_ms);
using RoverDiagNetGive = void (*)();
void rover_diag_set_net_guard(RoverDiagNetTake take, RoverDiagNetGive give);

/** C API for micro_ros_transport.cpp (and DIAG). timeout_ms=0 → no wait. */
extern "C" bool rover_net_lock(uint32_t timeout_ms);
extern "C" void rover_net_unlock(void);
