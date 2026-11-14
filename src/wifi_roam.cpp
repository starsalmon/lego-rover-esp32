#include "wifi_roam.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

namespace {

const char* g_ssid = "";
const char* g_pass = "";
uint32_t g_roam_count = 0;
uint32_t g_last_check_ms = 0;
uint32_t g_last_roam_ms = 0;

enum class ScanPhase : uint8_t { kIdle, kRunning };
ScanPhase g_scan = ScanPhase::kIdle;
int g_scan_rssi_before = 0;
uint8_t g_scan_bssid_before[6] = {0};

void copy_current_bssid(uint8_t out[6]) {
  const uint8_t* b = WiFi.BSSID();
  if (b) {
    memcpy(out, b, 6);
  } else {
    memset(out, 0, 6);
  }
}

bool bssid_same(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

void log_bssid(const char* tag, const uint8_t b[6]) {
  Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", tag, b[0], b[1], b[2], b[3], b[4],
                b[5]);
}

void apply_assisted_roaming() {
  wifi_config_t cfg{};
  if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
    return;
  }
  cfg.sta.rm_enabled = 1;
  cfg.sta.btm_enabled = 1;
#if defined(ESP_IDF_VERSION_MAJOR) && \
    (ESP_IDF_VERSION_MAJOR > 5 || \
     (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 2))
  cfg.sta.ft_enabled = 1;
#endif
  if (esp_wifi_set_config(WIFI_IF_STA, &cfg) == ESP_OK) {
    Serial.println("[wifi] 802.11k/v assisted roaming enabled (rm+btm)");
  }
}

void finish_scan_and_maybe_roam(uint32_t now_ms) {
  const int n = WiFi.scanComplete();
  g_scan = ScanPhase::kIdle;
  if (n <= 0) {
    if (n >= 0) {
      WiFi.scanDelete();
    }
    return;
  }

  int best_idx = -1;
  int best_rssi = -127;
  const int current_rssi = WiFi.RSSI();

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) != g_ssid) {
      continue;
    }
    const uint8_t* cand = WiFi.BSSID(i);
    if (cand && bssid_same(cand, g_scan_bssid_before)) {
      continue;
    }
    const int rssi = WiFi.RSSI(i);
    if (rssi >= current_rssi + ROVER_WIFI_ROAM_HYSTERESIS_DB &&
        rssi > best_rssi) {
      best_rssi = rssi;
      best_idx = i;
    }
  }
  if (best_idx < 0) {
    WiFi.scanDelete();
    return;
  }
  if (g_last_roam_ms && (now_ms - g_last_roam_ms) < ROVER_WIFI_ROAM_MIN_INTERVAL_MS) {
    return;
  }

  const uint8_t* tgt = WiFi.BSSID(best_idx);
  const int32_t ch = WiFi.channel(best_idx);
  if (!tgt || ch <= 0) {
    WiFi.scanDelete();
    return;
  }
  uint8_t tgt_bssid[6];
  memcpy(tgt_bssid, tgt, 6);

  Serial.printf("[wifi] roam #%lu: rssi %d -> %d ch%ld ",
                (unsigned long)(g_roam_count + 1), g_scan_rssi_before, best_rssi, (long)ch);
  log_bssid("from", g_scan_bssid_before);
  log_bssid("to", tgt_bssid);

  g_roam_count++;
  g_last_roam_ms = now_ms;
  WiFi.scanDelete();
  WiFi.disconnect(false);
  WiFi.begin(g_ssid, g_pass, ch, tgt_bssid);
}

}  // namespace

void rover_wifi_sta_common_setup(int tx_power_wifi_enum) {
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(WIFI_PS_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (tx_power_wifi_enum != 0) {
    WiFi.setTxPower((wifi_power_t)tx_power_wifi_enum);
  }
  apply_assisted_roaming();
}

void rover_wifi_roam_init(const char* ssid, const char* pass) {
  g_ssid = ssid ? ssid : "";
  g_pass = pass ? pass : "";
  g_last_check_ms = millis();
}

void rover_wifi_roam_tick(uint32_t now_ms) {
  if (!g_ssid[0]) {
    return;
  }

  if (g_scan == ScanPhase::kRunning) {
    const int pending = WiFi.scanComplete();
    if (pending == -2) {
      g_scan = ScanPhase::kIdle;
      return;
    }
    if (pending == -1) {
      return;
    }
    finish_scan_and_maybe_roam(now_ms);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (g_last_check_ms && (now_ms - g_last_check_ms) < ROVER_WIFI_ROAM_CHECK_MS) {
    return;
  }
  g_last_check_ms = now_ms;

  const int rssi = WiFi.RSSI();
  if (rssi >= ROVER_WIFI_ROAM_RSSI_TRIGGER_DBM) {
    return;
  }

  g_scan_rssi_before = rssi;
  copy_current_bssid(g_scan_bssid_before);
  const int rc = WiFi.scanNetworks(true, false);
  if (rc == -1) {
    g_scan = ScanPhase::kRunning;
  } else if (rc >= 0) {
    finish_scan_and_maybe_roam(now_ms);
  }
}

uint32_t rover_wifi_roam_count() { return g_roam_count; }

uint8_t rover_wifi_bssid_last_octet() {
  const uint8_t* b = WiFi.BSSID();
  return b ? b[5] : 0;
}