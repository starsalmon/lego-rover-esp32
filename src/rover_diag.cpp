#include "rover_diag.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(ROVER_MICROROS_WIFI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include "rover_pins_s3.h"
#endif

namespace {

RTC_NOINIT_ATTR uint32_t g_prev_ckpt;
RTC_NOINIT_ATTR uint32_t g_prev_ckpt_arg;
RTC_NOINIT_ATTR uint32_t g_prev_ckpt_ms;

TaskHandle_t g_pan_task = nullptr;
uint32_t g_last_diag_ms = 0;

#ifndef ROVER_DIAG_PERIOD_MS
#define ROVER_DIAG_PERIOD_MS 1000
#endif

#if defined(ROVER_MICROROS_WIFI)
WiFiUDP g_diag_udp;
bool g_diag_udp_host_ok = false;
IPAddress g_diag_udp_host;

void diag_udp_mirror(const char* line) {
  if (line == nullptr || WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (!g_diag_udp_host_ok) {
    g_diag_udp_host_ok = g_diag_udp_host.fromString(ROVER_TELEM_HOST);
  }
  if (!g_diag_udp_host_ok) {
    return;
  }
  g_diag_udp.beginPacket(g_diag_udp_host, ROVER_TELEM_PORT);
  g_diag_udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
  g_diag_udp.endPacket();
}
#else
void diag_udp_mirror(const char*) {}
#endif

void diag_emit(const char* line) {
  if (line == nullptr) {
    return;
  }
  Serial.print(line);
  if (line[strlen(line) - 1] != '\n') {
    Serial.println();
  }
  diag_udp_mirror(line);
}

}  // namespace

void rover_diag_set_pan_task(void* task_handle) { g_pan_task = static_cast<TaskHandle_t>(task_handle); }

void rover_diag_ckpt(RoverCkpt id, uint32_t arg) {
  g_prev_ckpt = static_cast<uint32_t>(id);
  g_prev_ckpt_arg = arg;
  g_prev_ckpt_ms = millis();
}

void rover_diag_begin(const char* reset_reason) {
  char buf[120];
  snprintf(buf, sizeof(buf), "BOOT reset=%s prev_ckpt=%lu arg=%lu at_ms=%lu\n",
           reset_reason ? reset_reason : "?",
           static_cast<unsigned long>(g_prev_ckpt),
           static_cast<unsigned long>(g_prev_ckpt_arg),
           static_cast<unsigned long>(g_prev_ckpt_ms));
  diag_emit(buf);
  rover_diag_ckpt(kCkptBoot);
}

void rover_diag_wifi_ready() {
  char buf[120];
  snprintf(buf, sizeof(buf), "BOOT_WIFI ckpt=%lu arg=%lu\n",
           static_cast<unsigned long>(g_prev_ckpt),
           static_cast<unsigned long>(g_prev_ckpt_arg));
  diag_emit(buf);
}

void rover_diag_event(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  char line[200];
  snprintf(line, sizeof(line), "EVENT %lu %s\n", static_cast<unsigned long>(millis()), buf);
  diag_emit(line);
}

void rover_diag_tick(const RoverDiagSample& s) {
  if (s.now_ms - g_last_diag_ms < ROVER_DIAG_PERIOD_MS) {
    return;
  }
  g_last_diag_ms = s.now_ms;

  uint32_t heap = ESP.getFreeHeap();
  uint32_t min_heap = ESP.getMinFreeHeap();
  uint32_t pan_stack = 0;
  if (g_pan_task != nullptr) {
    pan_stack = uxTaskGetStackHighWaterMark(g_pan_task);
  }

  char line[360];
  snprintf(line, sizeof(line),
           "DIAG t=%lu link=%d sess=%d cal=%d step=%u snaps=%u pan=%.0f rng=%.2f "
           "cmd=%.2f,%.2f mot=%.2f,%.2f bat=%.2f yaw=%.1f gz=%.1f trim=%.3f az=%.2f "
           "tof=%.2f,%.2f heap=%lu min_heap=%lu pan_stk=%lu ckpt=%lu\n",
           static_cast<unsigned long>(s.now_ms), s.link_state, s.session ? 1 : 0,
           s.cal_active ? 1 : 0, static_cast<unsigned>(s.cal_step),
           static_cast<unsigned>(s.cal_snaps), s.pan_deg, s.range_m, s.cmd_lin, s.cmd_ang,
           s.motor_l, s.motor_r, s.bat_v, s.yaw_deg, s.gz_dps, s.htrim, s.az_g, s.tof_l_m,
           s.tof_r_m,
           static_cast<unsigned long>(heap), static_cast<unsigned long>(min_heap),
           static_cast<unsigned long>(pan_stack), static_cast<unsigned long>(g_prev_ckpt));
  diag_emit(line);
}
