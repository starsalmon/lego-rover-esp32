// Rover camera node: DFRobot ESP32-S3 AI Camera V1.1 (OV3660).
//   :80  /            status page + links
//   :80  /capture     single JPEG frame
//   :80  /status      JSON of current sensor settings
//   :80  /control?var=<name>&val=<int>   tune a camera setting (see apply_control)
//   :80  /ir?on=0|1   built-in IR LEDs
//   :81  /stream      MJPEG (multipart/x-mixed-replace)
// OTA via espota (ArduinoOTA, port 3232), hostname ROVER_CAM_HOSTNAME.
//
// SLAM defaults: small frame, high fps, FIXED exposure + gain (no auto night logic).
// Every sensor setting is pushed at boot (lesson from the HA config); flip/mirror
// also work at runtime via /control.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>

#include "aicam_audio.h"
#include "aicam_whistle.h"
#include "rover_ota.h"
#include "rover_pins_aicam.h"

#ifndef ROVER_CAM_HOSTNAME
#define ROVER_CAM_HOSTNAME "rover-cam"
#endif
#ifndef CAM_DEFAULT_FRAMESIZE
#define CAM_DEFAULT_FRAMESIZE FRAMESIZE_VGA  // 640x480 (live-tested 2026-10-30, matches Beeper)
#endif
#ifndef CAM_DEFAULT_QUALITY
#define CAM_DEFAULT_QUALITY 12  // 0-63, lower = better
#endif
#ifndef CAM_DEFAULT_AEC_VALUE
#define CAM_DEFAULT_AEC_VALUE 1200  // 0-1200 manual exposure (tuned on Rover, indoor evening)
#endif
#ifndef CAM_DEFAULT_AGC_GAIN
#define CAM_DEFAULT_AGC_GAIN 15  // 0-30 manual gain (tuned on Rover)
#endif
#ifndef CAM_DEFAULT_BRIGHTNESS
#define CAM_DEFAULT_BRIGHTNESS 0  // -2..2 digital lift (tuned on Rover)
#endif
#ifndef CAM_DEFAULT_NIGHT
#define CAM_DEFAULT_NIGHT 0  // HA-style night mode (see apply_night)
#endif
#ifndef CAM_NIGHT_MAX_EXPO
#define CAM_NIGHT_MAX_EXPO 4095  // HA "OV3660 Exposure" default (256-4095)
#endif
#ifndef CAM_NIGHT_GAIN_CEIL
#define CAM_NIGHT_GAIN_CEIL 255  // 16x (HA used 1023; with IR on, 16x is plenty)
#endif
#ifndef CAM_NIGHT_AE_LEVEL
#define CAM_NIGHT_AE_LEVEL 2  // HA "OV3660 AE Level" default (-5..5)
#endif
#ifndef CAM_DEFAULT_IR
#define CAM_DEFAULT_IR 0  // IR LEDs off at boot (Cain: 5 V rail sag, 2026-10-20)
#endif
#ifndef CAM_DEFAULT_HMIRROR
#define CAM_DEFAULT_HMIRROR 0  // mounted upside down on Rover: 180 deg = vflip+hmirror inverted vs HA
#endif
#ifndef CAM_DEFAULT_VFLIP
#define CAM_DEFAULT_VFLIP 1
#endif

namespace {

RoverOta g_ota;
httpd_handle_t g_http = nullptr;
httpd_handle_t g_stream = nullptr;
bool g_cam_ok = false;
bool g_ir_on = false;
volatile uint32_t g_frames = 0;

// Desired settings, re-pushed as a whole after any change.
struct CamSettings {
  int framesize = CAM_DEFAULT_FRAMESIZE;
  int quality = CAM_DEFAULT_QUALITY;
  int aec = 1;  // 0 = manual exposure (fixed), 1 = auto
  int aec_value = CAM_DEFAULT_AEC_VALUE;
  int aec2 = 1;  // HA: AEC2 on
  int ae_level = 0;  // 0: +2 blew highlights (live-tested 2026-10-30)
  int agc = 1;       // 0 = manual gain
  int agc_gain = CAM_DEFAULT_AGC_GAIN;
  int gainceiling = 3;  // 0=2x 1=4x 2=8x 3=16x 4=32x. 16x: 32x was grainy/banded, 8x too dark (2026-10-30).
  // ALS auto day/night: picks gainceiling from the LTR-308 lux with hysteresis.
  int als_auto = 1;
  // This room reads about 8 lux on the LTR-308 and is still bright enough for color.
  // The old 15/40 trip points locked night+IR here and washed the orange out.
  int lux_night = 3;    // go to night profile below this (held 3 s)
  int lux_day = 4;      // back to day profile above this (held 3 s)
  int day_ceiling = 3;  // 16x (live-tested 2026-10-30 at ~5 lux)
  int night_ceiling = 2;  // 8x: dark rooms (16x was the old grainy look)
  int awb = 1;
  int brightness = 0, contrast = 0, saturation = 0;  // +1/+2 gave purple cast + washout (2026-10-30)
  int hmirror = CAM_DEFAULT_HMIRROR, vflip = CAM_DEFAULT_VFLIP;
  int lenc = 1, bpc = 1, wpc = 1, dcw = 1, colorbar = 0;
  int special_effect = 0;  // 0 none, 2 grayscale
  // Night profile (HA ai-cam.yaml): AEC+AGC auto, AEC2/night (0x3A00 bit2),
  // max exposure ceiling, gain ceiling, AE level -5..5.
  int night = CAM_DEFAULT_NIGHT;
  int night_expo = CAM_NIGHT_MAX_EXPO;
  int night_gain = CAM_NIGHT_GAIN_CEIL;
  int night_ae = CAM_NIGHT_AE_LEVEL;
  int night_frames = 0;  // SLAM tune (HA default 7); insertion halves fps per step
  // Frame length after the HA night writes. HA uses 3136 lines (~6 fps at QVGA);
  // 1300 gives ~17 fps / ~106 mean in Cain's room. Must stay >= 783 (QVGA default).
  int night_vts = 1564;  // HA normal frame length; HA night used 3136 + insertion 7 (~2 fps)
  int ir_auto = 1;  // IR LEDs follow the night profile
} g_set;

bool g_is_night = false;
int g_als_ok = -1;   // -1 unknown, 0 missing, 1 present
int g_als_part = -1;
float g_lux = -1.f;
float g_ir_offset = 0.f;  // lux the IR LEDs add to the ALS (measured at each IR switch-on)

// LTR-308 shares the SCCB bus with the OV3660; use the camera driver's SCCB helpers.
extern "C" uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
extern "C" int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);

int rd(sensor_t *s, int reg) { return s->get_reg(s, reg, 0xFF); }
void wr(sensor_t *s, int reg, int v) { s->set_reg(s, reg, 0xFF, v & 0xFF); }

// Exact port of HA external_components/ov3660_night.h ov3660_night_mode().
void ov3660_night_mode(sensor_t *s) {
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  int r = rd(s, 0x3A00);
  if (r >= 0) wr(s, 0x3A00, r | 0x04);  // night mode
  r = rd(s, 0x3A05);
  if (r >= 0) wr(s, 0x3A05, r | 0x40);  // frame-period insertion enable
  wr(s, 0x3A02, 0x0F);                  // max exposure 0x0FFF (60 Hz)
  wr(s, 0x3A03, 0xFF);
  wr(s, 0x3A14, 0x0F);                  // max exposure 0x0FFF (50 Hz)
  wr(s, 0x3A15, 0xFF);
  wr(s, 0x380E, 0x0C);                  // frame length 0x0C40 = 3136 lines
  wr(s, 0x380F, 0x40);
  r = rd(s, 0x3A17);
  if (r >= 0) wr(s, 0x3A17, (r & 0xFC) | 0x03);  // night gain threshold
  wr(s, 0x3A18, 0x03);                  // gain ceiling 0x03FF
  wr(s, 0x3A19, 0xFF);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);
  s->set_ae_level(s, 2);
  r = rd(s, 0x3A21);
  if (r >= 0) wr(s, 0x3A21, (r & 0x8F) | (7 << 4));  // max frame insertion
}
void ov3660_set_exposure_value(sensor_t *s, int v) {
  v = constrain(v, 256, 4095);
  wr(s, 0x3A02, (v >> 8) & 0x0F); wr(s, 0x3A03, v & 0xFF);
  wr(s, 0x3A14, (v >> 8) & 0x0F); wr(s, 0x3A15, v & 0xFF);
}
void ov3660_set_gain_value(sensor_t *s, int v) {
  v = constrain(v, 0, 1023);
  wr(s, 0x3A18, (v >> 8) & 0x03); wr(s, 0x3A19, v & 0xFF);
}
void ov3660_set_frame_insertion(sensor_t *s, int v) {
  v = constrain(v, 0, 7);
  const int r = rd(s, 0x3A21);
  if (r >= 0) wr(s, 0x3A21, (r & 0x8F) | ((v & 0x07) << 4));
}

// Same order as the HA apply script when night_mode_active:
// night_mode(); exposure(4095); gain(1023); frame insertion(7); then AE level(2).
void apply_night(sensor_t *s) {
  ov3660_night_mode(s);
  ov3660_set_exposure_value(s, g_set.night_expo);
  ov3660_set_gain_value(s, g_set.night_gain);
  ov3660_set_frame_insertion(s, g_set.night_frames);
  s->set_ae_level(s, constrain(g_set.night_ae, -5, 5));
  // SLAM override of the HA 0x0C40 frame length (keeps exposure/gain ceilings).
  wr(s, 0x380E, (g_set.night_vts >> 8) & 0xFF);
  wr(s, 0x380F, g_set.night_vts & 0xFF);
}

void push_settings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_framesize(s, static_cast<framesize_t>(g_set.framesize));
  s->set_quality(s, g_set.quality);
  s->set_brightness(s, g_set.brightness);
  s->set_contrast(s, g_set.contrast);
  s->set_saturation(s, g_set.saturation);
  s->set_whitebal(s, g_set.awb);
  s->set_awb_gain(s, g_set.awb);
  s->set_exposure_ctrl(s, g_set.aec);
  s->set_aec2(s, g_set.aec2);
  s->set_ae_level(s, g_set.ae_level);
  if (!g_set.aec) s->set_aec_value(s, g_set.aec_value);
  s->set_gain_ctrl(s, g_set.agc);
  if (!g_set.agc) s->set_agc_gain(s, g_set.agc_gain);
  // esp32-camera's OV3660 set_gainceiling writes the enum straight into 0x3A19
  // (ceiling 0.125x -> AGC pinned at 1x). Write the real ceiling instead.
  {
    const int ceil = (32 << constrain(g_set.gainceiling, 0, 5)) - 1;  // 0x10 = 1x
    wr(s, 0x3A18, (ceil >> 8) & 0x03);
    wr(s, 0x3A19, ceil & 0xFF);
  }
  s->set_lenc(s, g_set.lenc);
  s->set_bpc(s, g_set.bpc);
  s->set_wpc(s, g_set.wpc);
  s->set_dcw(s, g_set.dcw);
  s->set_hmirror(s, g_set.hmirror);
  s->set_vflip(s, g_set.vflip);
  s->set_colorbar(s, g_set.colorbar);
  s->set_special_effect(s, g_set.special_effect);
  if (g_set.night) {
    apply_night(s);  // last: overrides AEC/AGC/ceilings
  } else if (g_set.aec && g_set.agc) {
    // HA ov3660_normal_mode(): night bit + insertion off, exposure ceiling 0x066D,
    // frame length 0x061C (1564 lines).
    int r = rd(s, 0x3A00);
    if (r >= 0) wr(s, 0x3A00, r & ~0x04);
    r = rd(s, 0x3A05);
    if (r >= 0) wr(s, 0x3A05, r & ~0x40);
    wr(s, 0x3A02, 0x06); wr(s, 0x3A03, 0x6D);
    wr(s, 0x3A14, 0x06); wr(s, 0x3A15, 0x6D);
    wr(s, 0x380E, 0x06); wr(s, 0x380F, 0x1C);
    r = rd(s, 0x3A21);
    if (r >= 0) wr(s, 0x3A21, r & 0x8F);
  }
}

bool apply_control(const char *var, int val) {
  struct { const char *name; int *field; int lo, hi; } map[] = {
      {"framesize", &g_set.framesize, 0, FRAMESIZE_QXGA},
      {"quality", &g_set.quality, 4, 63},
      {"aec", &g_set.aec, 0, 1},
      {"aec_value", &g_set.aec_value, 0, 1200},
      {"exposure", &g_set.aec_value, 0, 1200},
      {"aec2", &g_set.aec2, 0, 1},
      {"ae_level", &g_set.ae_level, -5, 5},
      {"agc", &g_set.agc, 0, 1},
      {"agc_gain", &g_set.agc_gain, 0, 64},
      {"gain", &g_set.agc_gain, 0, 64},
      {"gainceiling", &g_set.gainceiling, 0, 5},
      {"als_auto", &g_set.als_auto, 0, 1},
      {"ir_auto", &g_set.ir_auto, 0, 1},
      {"lux_night", &g_set.lux_night, 0, 10000},
      {"lux_day", &g_set.lux_day, 0, 10000},
      {"day_ceiling", &g_set.day_ceiling, 0, 5},
      {"night_ceiling", &g_set.night_ceiling, 0, 5},
      {"awb", &g_set.awb, 0, 1},
      {"brightness", &g_set.brightness, -2, 2},
      {"contrast", &g_set.contrast, -2, 2},
      {"saturation", &g_set.saturation, -2, 2},
      {"hmirror", &g_set.hmirror, 0, 1},
      {"vflip", &g_set.vflip, 0, 1},
      {"lenc", &g_set.lenc, 0, 1},
      {"bpc", &g_set.bpc, 0, 1},
      {"wpc", &g_set.wpc, 0, 1},
      {"dcw", &g_set.dcw, 0, 1},
      {"colorbar", &g_set.colorbar, 0, 1},
      {"special_effect", &g_set.special_effect, 0, 6},
      {"night", &g_set.night, 0, 1},
      {"night_expo", &g_set.night_expo, 256, 4095},
      {"night_gain", &g_set.night_gain, 0, 1023},
      {"night_ae", &g_set.night_ae, -5, 5},
      {"night_frames", &g_set.night_frames, 0, 7},
      {"night_vts", &g_set.night_vts, 783, 3136},
  };
  for (auto &m : map) {
    if (strcmp(var, m.name) == 0) {
      if (val < m.lo || val > m.hi) return false;
      *m.field = val;
      push_settings();
      return true;
    }
  }
  return false;
}

void als_init() {
  SCCB_Write(AICAM_LTR308_ADDR, 0x00, 0x02);  // MAIN_CTRL: ALS active
  SCCB_Write(AICAM_LTR308_ADDR, 0x04, 0x22);  // 18-bit, 100 ms
  SCCB_Write(AICAM_LTR308_ADDR, 0x05, 0x01);  // gain 3x
  g_als_part = SCCB_Read(AICAM_LTR308_ADDR, 0x06);
  g_als_ok = ((g_als_part & 0xF0) == 0xB0) ? 1 : 0;
  Serial.printf("LTR-308 part id 0x%02X -> %s\n", g_als_part, g_als_ok ? "ok" : "missing");
}

void set_ir(bool on);
void als_tick() {
  static uint32_t last = 0, cond_since = 0;
  const uint32_t now = millis();
  if (!g_cam_ok || now - last < 500) return;
  last = now;
  if (g_als_ok < 0) als_init();
  if (g_als_ok != 1) return;
  const uint32_t raw = SCCB_Read(AICAM_LTR308_ADDR, 0x0D) | (SCCB_Read(AICAM_LTR308_ADDR, 0x0E) << 8) |
                       ((SCCB_Read(AICAM_LTR308_ADDR, 0x0F) & 0x0F) << 16);
  g_lux = 0.6f * raw / 3.0f;  // datasheet: 0.6 * data / (gain * int), int=1 at 100 ms
  static uint32_t hold_until = 0, ir_on_at = 0;
  static float lux_before_ir = -1.f;
  // Measure what the IR LEDs add (reflection), 1.5 s after switching them on.
  if (ir_on_at && now - ir_on_at > 1500) {
    g_ir_offset = max(0.f, g_lux - lux_before_ir);
    ir_on_at = 0;
  }
  if (!g_set.als_auto || now < hold_until || ir_on_at) { cond_since = 0; return; }
  const float eff = g_ir_on ? max(0.f, g_lux - g_ir_offset) : g_lux;
  const bool want_night = g_is_night ? (eff < g_set.lux_day) : (eff < g_set.lux_night);
  if (want_night == g_is_night) { cond_since = 0; return; }
  if (!cond_since) cond_since = now;
  if (now - cond_since < 3000) return;
  cond_since = 0;
  g_is_night = want_night;
  hold_until = now + 3000;  // ignore the ALS while the IR/exposure settle
  if (g_is_night && g_set.ir_auto) {  // HA order: IR first, then night registers
    lux_before_ir = g_lux;
    set_ir(true);
    ir_on_at = now;
    delay(100);
  }
  g_set.night = g_is_night ? 1 : 0;
  g_set.gainceiling = g_is_night ? g_set.night_ceiling : g_set.day_ceiling;
  push_settings();
  if (!g_is_night && g_set.ir_auto) set_ir(false);  // HA order: normal registers, then IR off
  Serial.printf("ALS %.1f lux (eff %.1f) -> %s\n", g_lux, eff, g_is_night ? "night" : "day");
}

void set_ir(bool on) {
  g_ir_on = on;
  digitalWrite(AICAM_IR_LED_PIN, on ? HIGH : LOW);
}

bool init_camera() {
  camera_config_t c = {};
  c.pin_pwdn = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_d0 = CAM_PIN_D0;
  c.pin_d1 = CAM_PIN_D1;
  c.pin_d2 = CAM_PIN_D2;
  c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4;
  c.pin_d5 = CAM_PIN_D5;
  c.pin_d6 = CAM_PIN_D6;
  c.pin_d7 = CAM_PIN_D7;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href = CAM_PIN_HREF;
  c.pin_pclk = CAM_PIN_PCLK;
  c.xclk_freq_hz = CAM_XCLK_HZ;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;
  // Allocate for VGA so framesize can be raised at runtime without re-init.
  c.frame_size = FRAMESIZE_XGA;
  c.jpeg_quality = g_set.quality;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;  // always newest frame (low latency for SLAM)
  const esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("camera init failed 0x%x\n", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("camera PID 0x%x (OV3660=0x%x)\n", s ? s->id.PID : 0, OV3660_PID);
  push_settings();
  return true;
}

// ---- Shared latest-frame buffer (one grabber, many stream clients) ----
// One task owns the camera; every MJPEG client and /capture reads the newest
// copy, so clients don't steal frames from each other.
constexpr size_t kJpgCap = 256 * 1024;
uint8_t *g_jpg = nullptr;
size_t g_jpg_len = 0;
volatile uint32_t g_jpg_seq = 0;
SemaphoreHandle_t g_jpg_mtx = nullptr;
volatile uint8_t g_stream_clients = 0;
constexpr uint8_t kMaxStreamClients = 4;

void grab_task(void *) {
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    if (fb->len <= kJpgCap) {
      xSemaphoreTake(g_jpg_mtx, portMAX_DELAY);
      memcpy(g_jpg, fb->buf, fb->len);
      g_jpg_len = fb->len;
      g_jpg_seq++;
      xSemaphoreGive(g_jpg_mtx);
    }
    esp_camera_fb_return(fb);
  }
}

// Copy the newest frame newer than *seq into buf; returns length (0 = none yet).
size_t latest_frame(uint8_t *buf, uint32_t *seq) {
  if (g_jpg_seq == *seq) return 0;
  xSemaphoreTake(g_jpg_mtx, portMAX_DELAY);
  const size_t n = g_jpg_len;
  memcpy(buf, g_jpg, n);
  *seq = g_jpg_seq;
  xSemaphoreGive(g_jpg_mtx);
  return n;
}

// ---- HTTP handlers ----

esp_err_t h_capture(httpd_req_t *req) {
  uint8_t *buf = static_cast<uint8_t *>(ps_malloc(kJpgCap));
  uint32_t seq = 0;
  size_t n = 0;
  for (int i = 0; buf && i < 100 && !(n = latest_frame(buf, &seq)); i++) vTaskDelay(pdMS_TO_TICKS(10));
  if (!n) {
    free(buf);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  const esp_err_t r = httpd_resp_send(req, reinterpret_cast<const char *>(buf), n);
  free(buf);
  return r;
}

#define PART_BOUNDARY "roverframe"
// MJPEG on :81 — own TCP server, one FreeRTOS task per client (max kMaxStreamClients).
void stream_client_task(void *arg) {
  WiFiClient *c = static_cast<WiFiClient *>(arg);
  uint8_t *buf = static_cast<uint8_t *>(ps_malloc(kJpgCap));
  // Read and discard the request headers (until blank line or 2 s).
  const uint32_t t0 = millis();
  String line;
  while (buf && c->connected() && millis() - t0 < 2000) {
    if (!c->available()) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    line = c->readStringUntil('\n');
    if (line.length() <= 1) break;
  }
  if (buf && c->connected()) {
    c->setNoDelay(true);
    c->print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=" PART_BOUNDARY
             "\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n");
    uint32_t seq = 0;
    char hdr[96];
    while (c->connected()) {
      const size_t n = latest_frame(buf, &seq);
      if (!n) { vTaskDelay(pdMS_TO_TICKS(3)); continue; }
      const int h = snprintf(hdr, sizeof(hdr),
                             "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                             static_cast<unsigned>(n));
      if (c->write(reinterpret_cast<const uint8_t *>(hdr), h) != static_cast<size_t>(h)) break;
      if (c->write(buf, n) != n) break;
      g_frames++;
    }
  }
  c->stop();
  delete c;
  free(buf);
  g_stream_clients--;
  vTaskDelete(nullptr);
}

void stream_server_task(void *) {
  WiFiServer server(81);
  server.begin();
  for (;;) {
    WiFiClient cl = server.available();
    if (cl) {
      if (g_stream_clients >= kMaxStreamClients) {
        cl.print("HTTP/1.1 503 Busy\r\nConnection: close\r\n\r\n");
        cl.stop();
      } else {
        g_stream_clients++;
        auto *heap_cl = new WiFiClient(cl);
        if (xTaskCreatePinnedToCore(stream_client_task, "mjpeg", 6144, heap_cl, 3, nullptr, 1) != pdPASS) {
          heap_cl->stop();
          delete heap_cl;
          g_stream_clients--;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

esp_err_t h_control(httpd_req_t *req) {
  char q[96], var[32], val[16];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(q, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need var & val");
    return ESP_FAIL;
  }
  if (!apply_control(var, atoi(val))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown var or out of range");
    return ESP_FAIL;
  }
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

// /reg?addr=0x3A19&val=0xFF  (write)  or  /reg?addr=0x3A19  (read)
esp_err_t h_reg(httpd_req_t *req) {
  char q[64], a[12], v[12];
  sensor_t *s = esp_camera_sensor_get();
  if (!s || httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "addr", a, sizeof(a)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need addr");
    return ESP_FAIL;
  }
  const int addr = strtol(a, nullptr, 0);
  if (httpd_query_key_value(q, "val", v, sizeof(v)) == ESP_OK) {
    s->set_reg(s, addr, 0xFF, strtol(v, nullptr, 0));
  }
  char b[48];
  snprintf(b, sizeof(b), "{\"addr\":%d,\"val\":%d}", addr, s->get_reg(s, addr, 0xFF));
  httpd_resp_sendstr(req, b);
  return ESP_OK;
}

esp_err_t h_ir(httpd_req_t *req) {
  char q[32], v[8];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "on", v, sizeof(v)) == ESP_OK) {
    set_ir(atoi(v) != 0);
  }
  httpd_resp_sendstr(req, g_ir_on ? "{\"ir\":1}" : "{\"ir\":0}");
  return ESP_OK;
}

esp_err_t h_status(httpd_req_t *req) {
  char b[1100];
  int expo_now = -1, gain_now = -1, vts = -1;
  if (sensor_t *s = esp_camera_sensor_get()) {
    expo_now = ((s->get_reg(s, 0x3500, 0xFF) & 0x0F) << 12) | (s->get_reg(s, 0x3501, 0xFF) << 4) |
               (s->get_reg(s, 0x3502, 0xFF) >> 4);
    gain_now = ((s->get_reg(s, 0x350A, 0xFF) & 0x03) << 8) | s->get_reg(s, 0x350B, 0xFF);
    vts = (s->get_reg(s, 0x380E, 0xFF) << 8) | s->get_reg(s, 0x380F, 0xFF);
  }
  snprintf(b, sizeof(b),
           "{\"cam_ok\":%d,\"framesize\":%d,\"quality\":%d,\"aec\":%d,\"aec_value\":%d,\"aec2\":%d,"
           "\"ae_level\":%d,\"agc\":%d,\"agc_gain\":%d,\"gainceiling\":%d,\"awb\":%d,\"brightness\":%d,"
           "\"contrast\":%d,\"saturation\":%d,\"hmirror\":%d,\"vflip\":%d,\"ir\":%d,\"frames\":%lu,"
           "\"night_frames\":%d,\"stream_clients\":%d,\"reset_reason\":%d,\"rssi\":%d,\"heap\":%u,\"psram\":%u,\"uptime_s\":%lu,\"night\":%d,\"night_expo\":%d,"
           "\"night_gain\":%d,\"night_ae\":%d,\"expo_now\":%d,\"gain_now\":%d,\"vts\":%d,"
           "\"als_ok\":%d,\"als_part\":%d,\"lux\":%.1f,\"als_auto\":%d,\"profile\":\"%s\",\"lux_night\":%d,\"lux_day\":%d,\"ir_offset\":%.1f}",
           g_cam_ok, g_set.framesize, g_set.quality, g_set.aec, g_set.aec_value, g_set.aec2,
           g_set.ae_level, g_set.agc, g_set.agc_gain, g_set.gainceiling, g_set.awb, g_set.brightness,
           g_set.contrast, g_set.saturation, g_set.hmirror, g_set.vflip, g_ir_on,
           static_cast<unsigned long>(g_frames), g_set.night_frames, g_stream_clients, static_cast<int>(esp_reset_reason()), WiFi.RSSI(), ESP.getFreeHeap(), ESP.getFreePsram(),
           static_cast<unsigned long>(millis() / 1000), g_set.night, g_set.night_expo,
           g_set.night_gain, g_set.night_ae, expo_now, gain_now, vts,
           g_als_ok, g_als_part, g_lux, g_set.als_auto, g_is_night ? "night" : "day", g_set.lux_night, g_set.lux_day, g_ir_offset);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, b);
  return ESP_OK;
}

esp_err_t h_index(httpd_req_t *req) {
  httpd_resp_sendstr(req,
                     "<html><body><h3>" ROVER_CAM_HOSTNAME "</h3>"
                     "<p><a href=/capture>/capture</a> | <a href=/status>/status</a> | "
                     "stream on :81/stream | /control?var=..&val=.. | /ir?on=0|1 | "
                     "<a href=/sounds>/sounds</a> | /play?file=chime | /stop | /volume?v=0-21 | <a href=/whistle>/whistle</a></p>"
                     "<img id=s style='max-width:100%'><script>"
                     "s.src=location.protocol+'//'+location.hostname+':81/stream'</script>"
                     "</body></html>");
  return ESP_OK;
}

void start_http() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  cfg.max_uri_handlers = 16;
  cfg.stack_size = 8192;  // /upload + /sounds use LittleFS
  if (httpd_start(&g_http, &cfg) == ESP_OK) {
    const httpd_uri_t uris[] = {
        {"/", HTTP_GET, h_index, nullptr},
        {"/capture", HTTP_GET, h_capture, nullptr},
        {"/control", HTTP_GET, h_control, nullptr},
        {"/ir", HTTP_GET, h_ir, nullptr},
        {"/reg", HTTP_GET, h_reg, nullptr},
        {"/status", HTTP_GET, h_status, nullptr},
    };
    for (const auto &u : uris) httpd_register_uri_handler(g_http, &u);
    audio_register(g_http);
    whistle_register(g_http);
  }
  if (!g_jpg) {
    g_jpg = static_cast<uint8_t *>(ps_malloc(kJpgCap));
    g_jpg_mtx = xSemaphoreCreateMutex();
    if (g_cam_ok && g_jpg) xTaskCreatePinnedToCore(grab_task, "grab", 4096, nullptr, 4, nullptr, 1);
  }
  xTaskCreatePinnedToCore(stream_server_task, "mjpegsrv", 4096, nullptr, 2, nullptr, 1);
  Serial.printf("HTTP up: http://%s/  stream http://%s:81/stream\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(AICAM_IR_LED_PIN, OUTPUT);
  set_ir(CAM_DEFAULT_IR);
  pinMode(AICAM_STATUS_LED_PIN, OUTPUT);
  digitalWrite(AICAM_STATUS_LED_PIN, LOW);

  Serial.printf("rover-cam boot, reset_reason %d (1=poweron 3=sw 4=panic 5=int_wdt 6=task_wdt 7=wdt 9=brownout), PSRAM %u bytes\n",
                static_cast<int>(esp_reset_reason()), ESP.getPsramSize());
  g_cam_ok = init_camera();
  audio_begin();
  whistle_begin();

  g_ota.begin(ROVER_CAM_HOSTNAME);  // starts Wi-Fi STA (credentials from platformio_private.ini)
  WiFi.setAutoReconnect(true);
}

void loop() {
  static bool http_started = false;
  static uint32_t last_blink = 0;
  g_ota.tick();
  if (!g_ota.busy()) als_tick();
  if (!http_started && WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);
    start_http();
    http_started = true;
    audio_play_startup();
  }
  // Status LED: solid = streaming-ready, blink = no Wi-Fi, fast blink = camera failed.
  const uint32_t now = millis();
  const uint32_t period = !g_cam_ok ? 150 : (WiFi.status() == WL_CONNECTED ? 0 : 600);
  if (period == 0) {
    digitalWrite(AICAM_STATUS_LED_PIN, HIGH);
  } else if (now - last_blink >= period) {
    last_blink = now;
    digitalWrite(AICAM_STATUS_LED_PIN, !digitalRead(AICAM_STATUS_LED_PIN));
  }
  delay(g_ota.busy() ? 0 : 5);
}
