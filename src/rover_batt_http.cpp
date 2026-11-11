#include "rover_batt_http.h"

#include <WebServer.h>
#include <math.h>

namespace {

// Same 2S pack scale as fleet-ha (v_empty=6.2, v_full=8.3).
constexpr float kEmptyV = 6.2f;
constexpr float kFullV = 8.3f;

WebServer *s_srv = nullptr;
float s_volts = 0.0f;

int percent_from_v(float v) {
  if (v <= 0.5f || kFullV <= kEmptyV) {
    return -1;
  }
  int p = static_cast<int>(lroundf((v - kEmptyV) / (kFullV - kEmptyV) * 100.0f));
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

void handle_battery() {
  const int p = percent_from_v(s_volts);
  char buf[96];
  if (p < 0) {
    s_srv->send(503, "application/json", "{\"ok\":false}");
    return;
  }
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"percent\":%d,\"voltage\":%.2f}", p, s_volts);
  s_srv->send(200, "application/json", buf);
}

}  // namespace

void rover_batt_http_begin() {
  if (s_srv != nullptr) {
    return;
  }
  s_srv = new WebServer(80);
  s_srv->on("/battery", HTTP_GET, handle_battery);
  s_srv->begin();
  Serial.println("battery HTTP /battery on :80");
}

void rover_batt_http_tick(float voltage) {
  s_volts = voltage;
  if (s_srv != nullptr) {
    s_srv->handleClient();
  }
}
