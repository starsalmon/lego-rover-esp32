#include "rover_ota.h"

#ifndef ROVER_WIFI_PASS
#define ROVER_WIFI_PASS ""
#endif

#if defined(ENABLE_OTA) && defined(ROVER_WIFI_SSID)

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>

namespace {

constexpr uint32_t WIFI_CONNECT_MS = 30000;

}  // namespace

void RoverOta::_start_ota() {
  if (_ota_started) return;
  _ota_started = true;

  ArduinoOTA.setHostname(_hostname);
  // Optional upload password — set ROVER_OTA_PASS in platformio_private.ini
#if defined(ROVER_OTA_PASS)
  ArduinoOTA.setPassword(ROVER_OTA_PASS);
#endif
  ArduinoOTA.onStart([this]() {
    _ota_busy = true;
    WiFi.setSleep(WIFI_PS_NONE);
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([this]() {
    _ota_busy = false;
    Serial.println("OTA end — rebooting");
  });
  ArduinoOTA.onError([this](ota_error_t err) {
    _ota_busy = false;
    Serial.printf("OTA err %u\n", err);
  });
  ArduinoOTA.begin();

  if (MDNS.begin(_hostname)) {
    MDNS.addService("arduino", "tcp", 3232);
  }

  WiFi.localIP().toString().toCharArray(_ip, sizeof(_ip));
  _state = OtaState::kReady;
  Serial.printf("OTA ready — http://%s.local  IP %s\n", _hostname, _ip);
}

void RoverOta::begin(const char *hostname) {
  if (hostname && hostname[0]) {
    _hostname = hostname;
  }
  _state = OtaState::kConnecting;
  _connect_t0 = millis();
  _ip[0] = '\0';

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(_hostname);
  WiFi.begin(ROVER_WIFI_SSID, ROVER_WIFI_PASS);
  Serial.printf("OTA: WiFi connecting to \"%s\"...\n", ROVER_WIFI_SSID);
}

void RoverOta::beginConnected(const char *hostname) {
  if (hostname && hostname[0]) {
    _hostname = hostname;
  }
  _state = OtaState::kConnecting;
  _connect_t0 = millis();
  _ip[0] = '\0';
}

void RoverOta::tick() {
  if (_state == OtaState::kDisabled || _state == OtaState::kFailed) {
    return;
  }

  if (_state == OtaState::kConnecting) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(WIFI_PS_NONE);
      _start_ota();
      return;
    }
    if (millis() - _connect_t0 > WIFI_CONNECT_MS) {
      _state = OtaState::kFailed;
      Serial.println("OTA: WiFi connect failed — USB flash only");
    }
    return;
  }

  if (_state == OtaState::kReady) {
    ArduinoOTA.handle();
  }
}

#else

void RoverOta::begin(const char *hostname) {
  if (hostname && hostname[0]) {
    _hostname = hostname;
  }
#if defined(ENABLE_OTA) && !defined(ROVER_WIFI_SSID)
  Serial.println("OTA: set ROVER_WIFI_SSID in platformio_private.ini");
#endif
}

void RoverOta::beginConnected(const char *hostname) { begin(hostname); }

void RoverOta::tick() {}

void RoverOta::_start_ota() {}

#endif
