#include "aicam_whistle.h"

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#include "aicam_audio.h"
#include "rover_pins_aicam.h"

namespace {

constexpr i2s_port_t kPort = I2S_NUM_0;  // ESP32-S3 PDM RX exists only on I2S0; the speaker uses I2S1
constexpr int kRate = 16000;
constexpr int kN = 256;                  // 16 ms block, 62.5 Hz bins
constexpr float kF0 = 1000.0f, kF1 = 3000.0f, kStep = kRate / (float)kN;
constexpr int kBins = (int)((kF1 - kF0) / kStep) + 1;  // 33

float g_coef[kBins];
volatile float g_min_ratio = 0.25f;  // share of block energy in the peak bin (pure tone ~0.4-1.0)
volatile float g_min_rms = 40.0f;    // raw 16-bit counts, after DC removal
volatile int g_min_ms = 400;
volatile uint32_t g_count = 0, g_last_ms = 0;
volatile float g_last_freq = 0, g_last_dur = 0, g_rms = 0, g_ratio = 0, g_peak = 0;
bool g_ok = false;

void mic_task(void *) {
  static int16_t buf[kN];
  float run_f = 0;
  int run_ms = 0, miss = 0;
  bool fired = false;
  uint32_t quiet_until = 0;
  for (;;) {
    size_t got = 0;
    if (i2s_read(kPort, buf, sizeof(buf), &got, portMAX_DELAY) != ESP_OK || got != sizeof(buf)) continue;
    float mean = 0;
    for (int i = 0; i < kN; i++) mean += buf[i];
    mean /= kN;
    float x[kN], ss = 0;
    for (int i = 0; i < kN; i++) { x[i] = buf[i] - mean; ss += x[i] * x[i]; }
    float best = 0; int bi = 0;
    for (int b = 0; b < kBins; b++) {
      float s1 = 0, s2 = 0, c = g_coef[b];
      for (int i = 0; i < kN; i++) { float s0 = x[i] + c * s1 - s2; s2 = s1; s1 = s0; }
      float p = s1 * s1 + s2 * s2 - c * s1 * s2;
      if (p > best) { best = p; bi = b; }
    }
    const float rms = sqrtf(ss / kN);
    const float ratio = ss > 0 ? best / (ss * kN / 2.0f) : 0;
    const float f = kF0 + bi * kStep;
    g_rms = rms; g_ratio = ratio; g_peak = f;
    const uint32_t now = millis();
    if (audio_is_playing()) quiet_until = now + 400;
    const bool tone = now >= quiet_until && rms >= g_min_rms && ratio >= g_min_ratio;
    if (tone && (run_ms == 0 || fabsf(f - run_f) <= 250.0f)) {
      run_f = run_ms ? 0.8f * run_f + 0.2f * f : f;
      run_ms += 16; miss = 0;
      if (!fired && run_ms >= g_min_ms) {
        fired = true;
        g_last_freq = run_f; g_last_dur = run_ms; g_last_ms = now; g_count = g_count + 1;
      }
    } else if (run_ms && ++miss > 3) {      // allow ~50 ms dropouts inside one whistle
      run_ms = 0; miss = 0; fired = false;
    }
  }
}

esp_err_t h_whistle(httpd_req_t *r) {
  char q[96], v[16];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK) {
    if (httpd_query_key_value(q, "ratio", v, sizeof(v)) == ESP_OK) g_min_ratio = constrain(atof(v), 0.05, 1.0);
    if (httpd_query_key_value(q, "rms", v, sizeof(v)) == ESP_OK) g_min_rms = constrain(atof(v), 1.0, 30000.0);
    if (httpd_query_key_value(q, "ms", v, sizeof(v)) == ESP_OK) g_min_ms = constrain(atoi(v), 100, 3000);
  }
  char b[320];
  snprintf(b, sizeof(b),
           "{\"ok\":%d,\"count\":%u,\"age_ms\":%u,\"freq\":%.0f,\"dur_ms\":%.0f,\"rms\":%.1f,\"ratio\":%.3f,\"peak_hz\":%.0f,"
           "\"min_ratio\":%.3f,\"min_rms\":%.1f,\"min_ms\":%d}",
           g_ok, (unsigned)g_count, g_count ? (unsigned)(millis() - g_last_ms) : 0u, g_last_freq, g_last_dur, g_rms, g_ratio, g_peak,
           g_min_ratio, g_min_rms, g_min_ms);
  httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(r, "application/json");
  return httpd_resp_sendstr(r, b);
}

}  // namespace

void whistle_begin() {
  for (int b = 0; b < kBins; b++) {
    const float k = (kF0 + b * kStep) / kStep;  // integer bin index
    g_coef[b] = 2.0f * cosf(2.0f * PI * k / kN);
  }
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  cfg.sample_rate = kRate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = kN;
  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = I2S_PIN_NO_CHANGE;
  pins.ws_io_num = AICAM_MIC_PDM_CLK;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = AICAM_MIC_PDM_DATA;
  g_ok = i2s_driver_install(kPort, &cfg, 0, nullptr) == ESP_OK && i2s_set_pin(kPort, &pins) == ESP_OK;
  Serial.printf("[whistle] PDM mic %s\n", g_ok ? "ok" : "FAIL");
  if (g_ok) xTaskCreatePinnedToCore(mic_task, "whistle", 6144, nullptr, 2, nullptr, 0);
}

void whistle_register(httpd_handle_t s) {
  const httpd_uri_t u = {"/whistle", HTTP_GET, h_whistle, nullptr};
  httpd_register_uri_handler(s, &u);
}
