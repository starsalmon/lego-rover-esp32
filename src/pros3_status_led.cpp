#include "pros3_status_led.h"

#include "rover_boot_beep.h"

#ifndef PIN_PROS3_RGB_LED
#define PIN_PROS3_RGB_LED 18
#endif
#ifndef PROS3_RGB_LED_BRIGHTNESS
#define PROS3_RGB_LED_BRIGHTNESS 20
#endif

#include <Adafruit_NeoPixel.h>

namespace {

Adafruit_NeoPixel* g_px = nullptr;

}  // namespace

void Pros3StatusLed::begin(int pin) {
  _pin = pin;
  if (_pin < 0) {
    return;
  }
  static Adafruit_NeoPixel pixel(1, _pin, NEO_GRB + NEO_KHZ800);
  g_px = &pixel;
  g_px->begin();
  g_px->setBrightness(PROS3_RGB_LED_BRIGHTNESS);
  g_px->clear();
  g_px->show();
  _show(36, 36, 36);
  const uint32_t now = millis();
  _boot_until = now + 350;
  _network_grace_until = now + ROVER_BOOT_BEEP_NETWORK_GRACE_MS;
  _ready = true;
}

Pros3StatusLed::Phase Pros3StatusLed::_phase(uint32_t now_ms) const {
  if (now_ms < _boot_until) {
    return Phase::kBoot;
  }
  const bool net_grace = now_ms < _network_grace_until;
  if (!_in.wifi_connected || !_in.uros_transport) {
    return Phase::kWifiConnecting;
  }
  if (_in.agent_connected) {
    return Phase::kAgentOk;
  }
  if (_in.agent_reconnecting && !net_grace) {
    return Phase::kAgentLost;
  }
  if (net_grace) {
    return Phase::kWifiConnecting;
  }
  return Phase::kWifiNoAgent;
}

void Pros3StatusLed::_show(uint8_t r, uint8_t g, uint8_t b) {
  if (!g_px) {
    return;
  }
  g_px->setPixelColor(0, r, g, b);
  g_px->show();
}

void Pros3StatusLed::_render(Phase phase, uint32_t now_ms) {
  bool on = true;
  if (phase == Phase::kWifiConnecting || phase == Phase::kAgentLost) {
    on = ((now_ms / 400) % 2) == 0;
  } else if (phase == Phase::kWifiNoAgent) {
    on = ((now_ms / 1200) % 2) == 0;
  }

  uint8_t r = 0, g = 0, b = 0;
  switch (phase) {
    case Phase::kBoot:
      r = g = b = 36;
      break;
    case Phase::kWifiConnecting:
      if (on) {
        r = 160;
        g = 70;
      }
      break;
    case Phase::kWifiNoAgent:
      if (on) {
        b = 90;
      }
      break;
    case Phase::kAgentOk:
      g = 32;
      break;
    case Phase::kAgentLost:
      if (on) {
        r = 120;
      }
      break;
  }
  _show(r, g, b);
}

void Pros3StatusLed::tick(uint32_t now_ms) {
  if (!_ready || _pin < 0) {
    return;
  }

  const Phase phase = _phase(now_ms);
  const bool fast = (phase == Phase::kWifiConnecting || phase == Phase::kAgentLost);
  const bool slow = (phase == Phase::kWifiNoAgent);
  const bool on = (!fast && !slow) ? true : (((now_ms / (fast ? 400 : 1200)) % 2) == 0);

  if (phase == _last_phase && on == _last_on && (now_ms - _last_show_ms) < 40) {
    return;
  }

  _last_phase = phase;
  _last_on = on;
  _last_show_ms = now_ms;
  _render(phase, now_ms);
}