#include "status_led.h"

#include <math.h>

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 10
#endif

#if STATUS_LED_PIN >= 0

#include <Adafruit_NeoPixel.h>

#ifndef STATUS_LED_BRIGHTNESS_PCT
#define STATUS_LED_BRIGHTNESS_PCT 20
#endif

// Adafruit setBrightness() is 0–255; we expose percent in platformio.ini.
#ifndef STATUS_LED_BRIGHTNESS
#define STATUS_LED_BRIGHTNESS ((255 * STATUS_LED_BRIGHTNESS_PCT) / 100)
#endif

// Waveshare ESP32-C3 Zero onboard WS2812 uses RGB channel order.
static Adafruit_NeoPixel s_px(1, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);

void StatusLed::begin(int pin) {
  _pin = pin;
  _state = LedState::kBoot;
  _flashing = false;
  if (_pin < 0) return;
  s_px.begin();
  s_px.setBrightness(STATUS_LED_BRIGHTNESS);
  s_px.clear();
  s_px.show();
  // Brief white flash so boot is visible even before the first tick().
  s_px.setPixelColor(0, 40, 40, 40);
  s_px.show();
}

void StatusLed::set_state(LedState state) {
  _state = state;
}

void StatusLed::trigger(LedEvent event) {
  _flash = event;
  _flashing = true;
  _flash_until = millis() + 600;
}

void StatusLed::tick(uint32_t now_ms) {
  if (_flashing && now_ms >= _flash_until) {
    _flashing = false;
  }
  _render(now_ms);
}

void StatusLed::_write(int pin, uint8_t r, uint8_t g, uint8_t b) {
  (void)pin;
  s_px.setPixelColor(0, r, g, b);
  s_px.show();
}

uint8_t StatusLed::_scale(uint8_t v, float gain) {
  const int x = (int)((float)v * gain);
  return (uint8_t)constrain(x, 0, 255);
}

void StatusLed::_render(uint32_t now_ms) {
  if (_pin < 0) return;

  if (_flashing) {
    const bool on = ((now_ms / 80) % 2) == 0;
    if (_flash == LedEvent::kBump) {
      _write(_pin, on ? 220 : 0, 0, 0);
    } else {
      _write(_pin, on ? 200 : 0, on ? 160 : 0, 0);
    }
    return;
  }

  const float t = now_ms / 1000.0f;
  const float pulse_slow = 0.5f + 0.5f * sinf(t * 2.0f * (float)M_PI * 0.8f);
  const float pulse_med = 0.5f + 0.5f * sinf(t * 2.0f * (float)M_PI * 2.0f);
  const float pulse_fast = 0.5f + 0.5f * sinf(t * 2.0f * (float)M_PI * 4.0f);

  switch (_state) {
    case LedState::kBoot:
      _write(_pin, _scale(220, pulse_slow), _scale(120, pulse_slow), 0);
      break;
    case LedState::kMpuError:
      _write(_pin, 80, 0, 0);
      break;
    case LedState::kWaitAgent:
      _write(_pin, 0, 0, _scale(200, pulse_slow));
      break;
    case LedState::kConnecting:
      _write(_pin, 0, _scale(200, pulse_fast), _scale(220, pulse_fast));
      break;
    case LedState::kReady:
      _write(_pin, 0, _scale(90, 0.6f + 0.4f * pulse_slow), 0);
      break;
    case LedState::kDriving:
      _write(_pin, 0, _scale(220, 0.55f + 0.45f * pulse_med), 0);
      break;
    case LedState::kLinkLost:
      _write(_pin, _scale(200, pulse_fast), 0, _scale(200, pulse_fast));
      break;
  }
}

#else  // STATUS_LED_PIN < 0 — T-Display-S3 uses onboard TFT for status instead.

void StatusLed::begin(int pin) {
  _pin = pin;
  _state = LedState::kBoot;
  _flashing = false;
}

void StatusLed::set_state(LedState state) {
  _state = state;
}

void StatusLed::trigger(LedEvent /*event*/) {}

void StatusLed::tick(uint32_t /*now_ms*/) {}

void StatusLed::_write(int /*pin*/, uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}

uint8_t StatusLed::_scale(uint8_t v, float gain) {
  const int x = (int)((float)v * gain);
  return (uint8_t)constrain(x, 0, 255);
}

void StatusLed::_render(uint32_t /*now_ms*/) {}

#endif
