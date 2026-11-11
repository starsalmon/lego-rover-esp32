// Rover AI cam audio: ESP32-audioI2S on the onboard MAX98357.
//   GET  /play?file=chime            LittleFS file (case-insensitive, ".mp3" optional)
//   GET  /play?url=http://host/x.mp3 stream from HTTP
//   GET  /stop
//   GET  /volume?v=0..21             (no v = read)
//   POST /upload?name=x.mp3          raw body = file bytes (curl --data-binary @x.mp3)
//   GET  /sounds                     list files + state
#include "aicam_audio.h"

#include <Arduino.h>
#include <Audio.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <time.h>

#include "rover_pins_aicam.h"

namespace {

enum CmdType : uint8_t { CMD_FILE, CMD_URL, CMD_STOP, CMD_VOL };
struct Cmd {
  CmdType type;
  int vol;
  char arg[200];
};

Audio *g_audio = nullptr;
QueueHandle_t g_q = nullptr;
bool g_fs_ok = false;
volatile bool g_playing = false;
volatile int g_volume = 12;
char g_now[200] = "";

constexpr int kPlayMax = 12;
char g_play[kPlayMax][32];
volatile int g_play_n = 0;
volatile int g_play_i = 0;
uint32_t g_play_gap_until = 0;
bool g_clip_seen = false;

bool start_file(const char *path) {
  if (!g_audio || !path || !path[0]) return false;
  g_audio->stopSong();
  g_audio->connecttoFS(LittleFS, path);
  strlcpy(g_now, path, sizeof(g_now));
  g_playing = true;
  g_clip_seen = false;
  return true;
}

void playlist_advance() {
  if (g_play_i >= g_play_n) return;
  const uint32_t now = millis();
  if (g_play_gap_until == 0) {
    g_play_gap_until = now + 60;
    return;
  }
  if (now < g_play_gap_until) return;
  g_play_gap_until = 0;
  const int i = g_play_i;
  g_play_i = i + 1;
  start_file(g_play[i]);
}

bool playlist_start(const char names[][32], int n) {
  if (n <= 0) return false;
  if (n > kPlayMax) n = kPlayMax;
  g_play_n = n;
  g_play_i = 1;
  g_play_gap_until = 0;
  for (int i = 0; i < n; i++) strlcpy(g_play[i], names[i], sizeof(g_play[i]));
  return start_file(g_play[0]);
}


void audio_task(void *) {
  g_audio = new Audio(false, 3, I2S_NUM_1);  // I2S0 is the only PDM-RX port on the S3: the whistle mic has it
  g_audio->setPinout(AICAM_SPK_I2S_BCLK, AICAM_SPK_I2S_LRC, AICAM_SPK_I2S_DOUT);
  g_audio->setVolume(g_volume);
  Cmd c;
  for (;;) {
    while (xQueueReceive(g_q, &c, 0) == pdTRUE) {
      switch (c.type) {
        case CMD_FILE: g_audio->stopSong(); g_audio->connecttoFS(LittleFS, c.arg); strlcpy(g_now, c.arg, sizeof(g_now)); break;
        case CMD_URL:  g_audio->stopSong(); g_audio->connecttohost(c.arg); strlcpy(g_now, c.arg, sizeof(g_now)); break;
        case CMD_STOP: g_audio->stopSong(); g_now[0] = 0; break;
        case CMD_VOL:  g_audio->setVolume(c.vol); break;
      }
    }
    g_audio->loop();
    const bool running = g_audio->isRunning();
    if (running) {
      g_clip_seen = true;
      g_playing = true;
    } else if (g_clip_seen) {
      // Clip actually finished (not just "not started yet").
      g_clip_seen = false;
      g_playing = false;
      g_now[0] = 0;
      playlist_advance();
    } else {
      g_playing = false;
    }
    vTaskDelay(running || g_play_i < g_play_n ? 1 : 10);
  }
}

bool send(CmdType t, const char *arg = "", int vol = 0) {
  if (!g_q) return false;
  Cmd c{t, vol, {0}};
  strlcpy(c.arg, arg, sizeof(c.arg));
  return xQueueSend(g_q, &c, pdMS_TO_TICKS(100)) == pdTRUE;
}

// Case-insensitive lookup in the LittleFS root; ".mp3" optional. out = "/Name.mp3".
bool find_file(const char *want, char *out, size_t n) {
  String w(want);
  w.trim();
  if (w.startsWith("/")) w.remove(0, 1);
  w.toLowerCase();
  String w_mp3 = w.endsWith(".mp3") ? w : w + ".mp3";
  File root = LittleFS.open("/");
  if (!root) return false;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    String name = f.name();
    if (name.startsWith("/")) name.remove(0, 1);
    String l = name;
    l.toLowerCase();
    if (l == w || l == w_mp3) {
      snprintf(out, n, "/%s", name.c_str());
      return true;
    }
  }
  return false;
}

void cors(httpd_req_t *r) { httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*"); }

esp_err_t err(httpd_req_t *r, const char *msg) {
  httpd_resp_set_status(r, "400 Bad Request");
  cors(r);
  return httpd_resp_sendstr(r, msg);
}

// Minimal %XX / '+' decoding for the url= parameter.
void url_decode(char *s) {
  char *o = s;
  for (; *s; ++s) {
    if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char h[3] = {s[1], s[2], 0};
      *o++ = (char)strtol(h, nullptr, 16);
      s += 2;
    } else {
      *o++ = (*s == '+') ? ' ' : *s;
    }
  }
  *o = 0;
}

esp_err_t h_play(httpd_req_t *r) {
  char q[300], v[220];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) != ESP_OK) return err(r, "need ?file=name or ?url=http://...");
  if (httpd_query_key_value(q, "url", v, sizeof(v)) == ESP_OK) {
    url_decode(v);
    if (strncasecmp(v, "http://", 7) && strncasecmp(v, "https://", 8)) return err(r, "url must start with http:// or https://");
    send(CMD_URL, v);
    cors(r);
    return httpd_resp_sendstr(r, "ok");
  }
  if (httpd_query_key_value(q, "file", v, sizeof(v)) == ESP_OK) {
    url_decode(v);
    char path[200];
    if (!g_fs_ok) return err(r, "LittleFS not mounted");
    if (!find_file(v, path, sizeof(path))) return err(r, "file not found (see /sounds)");
    send(CMD_FILE, path);
    cors(r);
    return httpd_resp_sendstr(r, path);
  }
  return err(r, "need ?file=name or ?url=http://...");
}

esp_err_t h_stop(httpd_req_t *r) {
  send(CMD_STOP);
  cors(r);
  return httpd_resp_sendstr(r, "ok");
}

esp_err_t h_volume(httpd_req_t *r) {
  char q[32], v[8];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "v", v, sizeof(v)) == ESP_OK) {
    g_volume = constrain(atoi(v), 0, 21);
    send(CMD_VOL, "", g_volume);
  }
  char b[32];
  snprintf(b, sizeof(b), "{\"volume\":%d}", g_volume);
  cors(r);
  httpd_resp_set_type(r, "application/json");
  return httpd_resp_sendstr(r, b);
}

esp_err_t h_upload(httpd_req_t *r) {
  char q[96], name[64];
  if (!g_fs_ok) return err(r, "LittleFS not mounted");
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "name", name, sizeof(name)) != ESP_OK || !name[0] || strchr(name, '/'))
    return err(r, "need ?name=file.mp3");
  String n(name);
  if (!n.endsWith(".mp3") && !n.endsWith(".MP3")) n += ".mp3";
  if (r->content_len == 0 || r->content_len > LittleFS.totalBytes() - LittleFS.usedBytes())
    return err(r, "empty body or not enough space");
  File f = LittleFS.open("/" + n, "w");
  if (!f) return err(r, "open failed");
  static char buf[2048];
  size_t left = r->content_len;
  while (left) {
    int got = httpd_req_recv(r, buf, min(left, sizeof(buf)));
    if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (got <= 0) { f.close(); LittleFS.remove("/" + n); return err(r, "receive failed"); }
    f.write(reinterpret_cast<uint8_t *>(buf), got);
    left -= got;
  }
  f.close();
  cors(r);
  return httpd_resp_sendstr(r, ("saved /" + n).c_str());
}

esp_err_t h_sounds(httpd_req_t *r) {
  String j = "{\"fs_ok\":" + String(g_fs_ok ? 1 : 0) + ",\"playing\":" + String(g_playing ? 1 : 0) +
             ",\"now\":\"" + String(g_now) + "\",\"volume\":" + String(g_volume) + ",\"files\":[";
  if (g_fs_ok) {
    File root = LittleFS.open("/");
    bool first = true;
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      String nm = f.name();
      j += String(first ? "" : ",") + "{\"name\":\"" + nm + "\",\"bytes\":" + String(f.size()) + "}";
      first = false;
    }
    j += "],\"free\":" + String(LittleFS.totalBytes() - LittleFS.usedBytes());
  } else {
    j += "]";
  }
  j += "}";
  cors(r);
  httpd_resp_set_type(r, "application/json");
  return httpd_resp_sendstr(r, j.c_str());
}

}  // namespace

void audio_begin() {
  g_fs_ok = LittleFS.begin(true);  // format on first boot if the partition is blank
  Serial.printf("[audio] LittleFS %s total %u used %u\n", g_fs_ok ? "ok" : "FAIL",
                g_fs_ok ? LittleFS.totalBytes() : 0, g_fs_ok ? LittleFS.usedBytes() : 0);
  g_q = xQueueCreate(6, sizeof(Cmd));
  // Core 0 (camera grab + MJPEG tasks are pinned to core 1), below Wi-Fi priority.
  xTaskCreatePinnedToCore(audio_task, "audio", 8192, nullptr, 3, nullptr, 0);
}

// Melbourne time for the greeting. POSIX TZ: AEST UTC+10, AEDT UTC+11,
// DST starts first Sunday of October, ends first Sunday of April at 03:00.
static const char *kTzMelbourne = "AEST-10AEDT,M10.1.0,M4.1.0/3";

bool wait_for_local_time(struct tm *out, uint32_t budget_ms) {
  setenv("TZ", kTzMelbourne, 1);
  tzset();
  configTzTime(kTzMelbourne, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
  const uint32_t start = millis();
  while (millis() - start < budget_ms) {
    if (getLocalTime(out, 200) && out->tm_year >= (2024 - 1900)) {
      return true;
    }
  }
  return false;
}

int fetch_battery_percent(uint32_t budget_ms) {
  const char *urls[] = {
      "http://rover-esp.lan/battery",
      "http://rover-esp.local/battery",
      "http://192.168.3.71/battery",
  };
  const uint32_t start = millis();
  int url_i = 0;
  while (millis() - start < budget_ms) {
    HTTPClient http;
    http.setConnectTimeout(800);
    http.setTimeout(800);
    if (!http.begin(urls[url_i % 3])) {
      url_i++;
      delay(150);
      continue;
    }
    const int code = http.GET();
    String body = (code > 0) ? http.getString() : String();
    http.end();
    if (code == 200) {
      int p = -1;
      int idx = body.indexOf("\"percent\"");
      if (idx >= 0) {
        idx = body.indexOf(':', idx);
        if (idx >= 0) p = body.substring(idx + 1).toInt();
      } else {
        p = body.toInt();
      }
      if (p >= 0 && p <= 100) return p;
    }
    url_i++;
    delay(200);
  }
  return -1;
}

void append_number_clips(char names[][32], int &n, int pct) {
  auto add = [&](const char *stem) {
    char path[64];
    if (n >= kPlayMax) return;
    if (find_file(stem, path, sizeof(path))) {
      strlcpy(names[n++], path, 32);
    } else {
      Serial.printf("[audio] missing clip %s\n", stem);
    }
  };
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  if (pct <= 19 || pct == 100) {
    char stem[8];
    snprintf(stem, sizeof(stem), "n%d", pct);
    add(stem);
    return;
  }
  const int tens = (pct / 10) * 10;
  const int units = pct % 10;
  char stem[8];
  snprintf(stem, sizeof(stem), "n%d", tens);
  add(stem);
  if (units) {
    snprintf(stem, sizeof(stem), "n%d", units);
    add(stem);
  }
}

void greeting_task(void *) {
  struct tm tinfo = {};
  const bool got_time = wait_for_local_time(&tinfo, 4000);
  const int pct = fetch_battery_percent(5000);

  const char *greet = "hello";
  if (got_time) {
    const int hm = tinfo.tm_hour * 60 + tinfo.tm_min;
    if (hm >= 5 * 60 && hm < 12 * 60) greet = "morning";
    else if (hm >= 12 * 60 && hm < 18 * 60) greet = "afternoon";
    else greet = "evening";
    Serial.printf("[audio] local %02d:%02d greeting=%s battery=%d\n", tinfo.tm_hour,
                  tinfo.tm_min, greet, pct);
  } else {
    Serial.printf("[audio] no NTP, fallback hello, battery=%d\n", pct);
  }

  char names[kPlayMax][32];
  int n = 0;
  auto add = [&](const char *stem) {
    char path[64];
    if (n >= kPlayMax) return;
    if (find_file(stem, path, sizeof(path))) {
      strlcpy(names[n++], path, 32);
    } else {
      Serial.printf("[audio] missing clip %s\n", stem);
    }
  };
  add("hi");
  add(greet);
  add("im_rover");
  if (pct >= 0) {
    add("my_battery_is");
    append_number_clips(names, n, pct);
    add("percent");
  }
  if (n == 0) {
    Serial.println("[audio] no greeting clips on LittleFS");
  } else {
    playlist_start(names, n);
  }
  vTaskDelete(nullptr);
}

// Spoken boot greeting (after Wi-Fi is up). Own task so NTP / battery wait
// does not stall the camera HTTP loop.
void audio_play_startup() {
  static bool started = false;
  if (started || !g_fs_ok) return;
  started = true;
  xTaskCreatePinnedToCore(greeting_task, "greet", 8192, nullptr, 2, nullptr, 0);
}

bool audio_is_playing() { return g_playing; }

void audio_register(httpd_handle_t s) {
  const httpd_uri_t uris[] = {
      {"/play", HTTP_GET, h_play, nullptr},
      {"/stop", HTTP_GET, h_stop, nullptr},
      {"/volume", HTTP_GET, h_volume, nullptr},
      {"/upload", HTTP_POST, h_upload, nullptr},
      {"/sounds", HTTP_GET, h_sounds, nullptr},
  };
  for (const auto &u : uris) httpd_register_uri_handler(s, &u);
}
