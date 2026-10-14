#include "rover_diag.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <esp_system.h>
#include <esp_core_dump.h>

#if defined(ROVER_MICROROS_WIFI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include "rover_pins_s3.h"
#endif

namespace {

RTC_NOINIT_ATTR uint32_t g_prev_ckpt;
RTC_NOINIT_ATTR uint32_t g_prev_ckpt_arg;
RTC_NOINIT_ATTR uint32_t g_prev_ckpt_ms;

// Last panic backtrace mirror in RTC (survives panic reboot; cleared on power-on
// if RTC was lost). Magic verifies validity. Filled from flash coredump summary
// before UDP emit; also kept if coredump erase fails.
static constexpr uint32_t kPanicBtMagic = 0x50414E49u;  // 'PANI'
#ifndef ROVER_PANIC_BT_MAX
#define ROVER_PANIC_BT_MAX 16
#endif
RTC_NOINIT_ATTR uint32_t g_panic_bt_magic;
RTC_NOINIT_ATTR uint32_t g_panic_bt_pc;
RTC_NOINIT_ATTR uint32_t g_panic_bt_cause;
RTC_NOINIT_ATTR uint32_t g_panic_bt_vaddr;
RTC_NOINIT_ATTR uint32_t g_panic_bt_depth;
RTC_NOINIT_ATTR uint32_t g_panic_bt_corr;
RTC_NOINIT_ATTR uint32_t g_panic_bt_frames[ROVER_PANIC_BT_MAX];
RTC_NOINIT_ATTR char g_panic_bt_task[16];

TaskHandle_t g_pan_task = nullptr;
uint32_t g_last_diag_ms = 0;

#ifndef ROVER_DIAG_PERIOD_MS
#define ROVER_DIAG_PERIOD_MS 100
#endif

#if defined(ROVER_MICROROS_WIFI)
WiFiUDP g_diag_udp;
bool g_diag_udp_host_ok = false;
IPAddress g_diag_udp_host;
RoverDiagNetTake g_net_take = nullptr;
RoverDiagNetGive g_net_give = nullptr;

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
  // Block until net lock (same recursive mu as micro-ROS WiFiUDP). Do NOT
  // fail-fast at 15ms — that still raced parsePacket under load.
  if (!rover_net_lock(2000)) {
    return;  // Serial still emitted by diag_emit; drop UDP only
  }
  g_diag_udp.beginPacket(g_diag_udp_host, ROVER_TELEM_PORT);
  g_diag_udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
  g_diag_udp.endPacket();
  rover_net_unlock();
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

void rover_diag_wifi_ready(const char* reset_reason) {
  char buf[160];
  snprintf(buf, sizeof(buf), "BOOT_WIFI reset=%s ckpt=%lu arg=%lu\n",
           reset_reason ? reset_reason : "?",
           static_cast<unsigned long>(g_prev_ckpt),
           static_cast<unsigned long>(g_prev_ckpt_arg));
  diag_emit(buf);
  // Prefer flash coredump (written by IDF on panic) → UDP telem. No USB Serial needed.
  rover_diag_emit_panic_coredump(reset_reason);
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
  uint32_t pan_stack = s.pan_stack_words;
  if (pan_stack == 0 && g_pan_task != nullptr) {
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


static bool reset_looks_like_panic(const char* reset_reason) {
  if (reset_reason == nullptr) {
    return false;
  }
  // Matches reset_reason_str() in microros_main.cpp
  return strcmp(reset_reason, "panic") == 0 || strcmp(reset_reason, "task_wdt") == 0 ||
         strcmp(reset_reason, "int_wdt") == 0 || strcmp(reset_reason, "wdt") == 0;
}

static void emit_panic_bt_lines(uint32_t pc, uint32_t cause, uint32_t vaddr, uint32_t depth,
                                uint32_t corr, const uint32_t* frames, const char* task) {
  char line[320];
  snprintf(line, sizeof(line),
           "PANIC_CORE task=%s pc=0x%08lx cause=%lu vaddr=0x%08lx depth=%lu corr=%lu\n",
           (task && task[0]) ? task : "?", static_cast<unsigned long>(pc),
           static_cast<unsigned long>(cause), static_cast<unsigned long>(vaddr),
           static_cast<unsigned long>(depth), static_cast<unsigned long>(corr));
  diag_emit(line);

  // One PANIC_BT line with up to 16 PCs (fits UDP easily).
  size_t off = 0;
  off += snprintf(line + off, sizeof(line) - off, "PANIC_BT");
  const uint32_t n = (depth > ROVER_PANIC_BT_MAX) ? ROVER_PANIC_BT_MAX : depth;
  for (uint32_t i = 0; i < n && off + 12 < sizeof(line); i++) {
    off += snprintf(line + off, sizeof(line) - off, " 0x%08lx",
                    static_cast<unsigned long>(frames[i]));
  }
  if (off + 2 < sizeof(line)) {
    line[off++] = '\n';
    line[off] = '\0';
  }
  diag_emit(line);
}

void rover_diag_emit_panic_coredump(const char* reset_reason) {
  const bool want = reset_looks_like_panic(reset_reason);
  // Always try flash image if present; also fall back to RTC stash after panic-ish reset.
  bool emitted = false;

#if defined(CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH) && defined(CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF)
  esp_err_t chk = esp_core_dump_image_check();
  if (chk == ESP_OK) {
    esp_core_dump_summary_t* summary =
        static_cast<esp_core_dump_summary_t*>(malloc(sizeof(esp_core_dump_summary_t)));
    if (summary != nullptr) {
      if (esp_core_dump_get_summary(summary) == ESP_OK) {
        const uint32_t depth = summary->exc_bt_info.depth;
        const uint32_t n =
            (depth > ROVER_PANIC_BT_MAX) ? ROVER_PANIC_BT_MAX : depth;
        // Mirror into RTC for a later boot if UDP somehow missed.
        g_panic_bt_magic = kPanicBtMagic;
        g_panic_bt_pc = summary->exc_pc;
        g_panic_bt_cause = summary->ex_info.exc_cause;
        g_panic_bt_vaddr = summary->ex_info.exc_vaddr;
        g_panic_bt_depth = n;
        g_panic_bt_corr = summary->exc_bt_info.corrupted ? 1u : 0u;
        memset(g_panic_bt_task, 0, sizeof(g_panic_bt_task));
        strncpy(g_panic_bt_task, summary->exc_task, sizeof(g_panic_bt_task) - 1);
        for (uint32_t i = 0; i < n; i++) {
          g_panic_bt_frames[i] = summary->exc_bt_info.bt[i];
        }
        for (uint32_t i = n; i < ROVER_PANIC_BT_MAX; i++) {
          g_panic_bt_frames[i] = 0;
        }

        char sha[APP_ELF_SHA256_SZ + 1];
        memcpy(sha, summary->app_elf_sha256, APP_ELF_SHA256_SZ);
        sha[APP_ELF_SHA256_SZ] = '\0';
        char meta[200];
        snprintf(meta, sizeof(meta), "PANIC_META sha=%s ver=%lu\n", sha,
                 static_cast<unsigned long>(summary->core_dump_version));
        diag_emit(meta);

        emit_panic_bt_lines(g_panic_bt_pc, g_panic_bt_cause, g_panic_bt_vaddr, g_panic_bt_depth,
                            g_panic_bt_corr, g_panic_bt_frames, g_panic_bt_task);
        emitted = true;
        // Erase so we do not re-broadcast every boot.
        (void)esp_core_dump_image_erase();
      }
      free(summary);
    }
  } else if (want) {
    char line[120];
    snprintf(line, sizeof(line), "PANIC_CORE dump_check=%d (no flash image)\n",
             static_cast<int>(chk));
    diag_emit(line);
  }
#else
  (void)want;
#endif

  // RTC fallback if flash path did not emit but we have a prior stash + panic reset.
  if (!emitted && want && g_panic_bt_magic == kPanicBtMagic && g_panic_bt_depth > 0 &&
      g_panic_bt_depth <= ROVER_PANIC_BT_MAX) {
    diag_emit("PANIC_META src=rtc\n");
    emit_panic_bt_lines(g_panic_bt_pc, g_panic_bt_cause, g_panic_bt_vaddr, g_panic_bt_depth,
                        g_panic_bt_corr, g_panic_bt_frames, g_panic_bt_task);
    // Clear magic so we only send once per crash.
    g_panic_bt_magic = 0;
    emitted = true;
  }

  if (!emitted && want) {
    diag_emit("PANIC_CORE no_backtrace_available\n");
  }
}

void rover_diag_set_net_guard(RoverDiagNetTake take, RoverDiagNetGive give) {
#if defined(ROVER_MICROROS_WIFI)
  g_net_take = take;
  g_net_give = give;
#else
  (void)take;
  (void)give;
#endif
}

extern "C" bool rover_net_lock(uint32_t timeout_ms) {
#if defined(ROVER_MICROROS_WIFI)
  if (g_net_take == nullptr) {
    return true;  // early boot before guard wired
  }
  return g_net_take(timeout_ms);
#else
  (void)timeout_ms;
  return true;
#endif
}

extern "C" void rover_net_unlock(void) {
#if defined(ROVER_MICROROS_WIFI)
  if (g_net_give != nullptr) {
    g_net_give();
  }
#endif
}
