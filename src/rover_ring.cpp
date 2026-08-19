#include "rover_ring.h"

#include <Adafruit_NeoPixel.h>
#include <math.h>

namespace {

Adafruit_NeoPixel* g_px = nullptr;

}  // namespace

bool RoverRing::begin(int pin, uint8_t count, uint8_t brightness) {
  _pin = pin;
  _count = count > 0 ? count : ROVER_RING_COUNT;
  _ok = false;
  if (_pin < 0) {
    return false;
  }
  if (g_px != nullptr) {
    delete g_px;
    g_px = nullptr;
  }
  g_px = new Adafruit_NeoPixel(_count, _pin, NEO_GRB + NEO_KHZ800);
  if (g_px == nullptr) {
    return false;
  }
  g_px->begin();
  g_px->setBrightness(brightness);
  g_px->clear();
  g_px->show();
  for (uint8_t i = 0; i < _count; i++) {
    _sonar_bins[i] = 255;
  }
  _ok = true;
  _mode = kModeStandby;
  return true;
}

void RoverRing::set_pixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  if (!g_px) return;
  g_px->setPixelColor(i % _count, r, g, b);
}

void RoverRing::hsv_color(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) const {
  h = fmodf(h, 1.0f);
  if (h < 0.0f) {
    h += 1.0f;
  }
  const int i = static_cast<int>(floorf(h * 6.0f));
  const float f = h * 6.0f - static_cast<float>(i);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - f * s);
  const float t = v * (1.0f - (1.0f - f) * s);
  float rf = 0.0f;
  float gf = 0.0f;
  float bf = 0.0f;
  switch (i % 6) {
    case 0:
      rf = v;
      gf = t;
      bf = p;
      break;
    case 1:
      rf = q;
      gf = v;
      bf = p;
      break;
    case 2:
      rf = p;
      gf = v;
      bf = t;
      break;
    case 3:
      rf = p;
      gf = q;
      bf = v;
      break;
    case 4:
      rf = t;
      gf = p;
      bf = v;
      break;
    default:
      rf = v;
      gf = p;
      bf = q;
      break;
  }
  r = static_cast<uint8_t>(rf * 255.0f);
  g = static_cast<uint8_t>(gf * 255.0f);
  b = static_cast<uint8_t>(bf * 255.0f);
}

void RoverRing::set_mode(uint8_t mode_id) {
  _mode = mode_id;
  if (mode_id == kModeSonar) {
    for (uint8_t i = 0; i < _count; i++) {
      _sonar_bins[i] = 255;
    }
  }
}

void RoverRing::flash(uint8_t flash_id, uint32_t duration_ms) {
  _flash = flash_id;
  _flash_until_ms = millis() + duration_ms;
}

void RoverRing::sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led) {
  if (!dist_cm || count == 0) {
    return;
  }
  const uint8_t n = count < _count ? count : _count;
  for (uint8_t i = 0; i < n; i++) {
    _sonar_bins[i] = dist_cm[i];
  }
  _sonar_sweep = sweep_led % _count;
  _mode = kModeSonar;
}

void RoverRing::render(uint32_t now_ms) {
  if (!g_px) return;

  const float t = now_ms / 1000.0f;
  if (_flash != 0 && now_ms < _flash_until_ms) {
    if (_flash == kFlashBump) {
      const bool on = (now_ms / 70) % 2 == 0;
      for (uint8_t i = 0; i < _count; i++) {
        set_pixel(i, on ? 220 : 0, 0, on ? 40 : 0);
      }
      g_px->show();
      return;
    }
    if (_flash == kFlashStall) {
      for (uint8_t i = 0; i < _count; i++) {
        const float wave = (sinf(t * 10.0f) + 1.0f) * 0.5f;
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        hsv_color(0.12f, 1.0f, 0.15f + 0.85f * wave, r, g, b);
        set_pixel(i, r, g, b);
      }
      g_px->show();
      return;
    }
    if (_flash == kFlashStart) {
      for (uint8_t i = 0; i < _count; i++) {
        const float h = fmodf(static_cast<float>(i) / _count + _phase * 0.4f, 1.0f);
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        hsv_color(h, 1.0f, 0.9f, r, g, b);
        set_pixel(i, r, g, b);
      }
      g_px->show();
      return;
    }
    if (_flash == kFlashStop) {
      for (uint8_t i = 0; i < _count; i++) {
        const float dist = fabsf(static_cast<float>(i) - _count / 2.0f) / (_count / 2.0f);
        const float fade = fmaxf(0.0f, 1.0f - dist);
        set_pixel(i, static_cast<uint8_t>(200.0f * fade), 0, static_cast<uint8_t>(80.0f * fade));
      }
      g_px->show();
      return;
    }
  } else if (_flash != 0 && now_ms >= _flash_until_ms) {
    _flash = 0;
  }

  switch (_mode) {
    case kModeStandby:
      for (uint8_t i = 0; i < _count; i++) {
        const float h = fmodf(static_cast<float>(i) / _count + _phase * 0.08f, 1.0f);
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        hsv_color(h, 0.85f, 0.35f + 0.25f * sinf(_phase + i * 0.5f), r, g, b);
        set_pixel(i, r, g, b);
      }
      break;
    case kModeReady:
      for (uint8_t i = 0; i < _count; i++) {
        const float dist = fmodf(static_cast<float>(i) - _phase * 2.5f + _count, _count);
        const float v = fmaxf(0.0f, 1.0f - dist / (_count * 0.45f));
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        hsv_color(0.35f, 0.9f, v, r, g, b);
        set_pixel(i, r, g, b);
      }
      break;
    case kModeAuto:
      for (uint8_t i = 0; i < _count; i++) {
        const float h1 = fmodf(static_cast<float>(i) / _count + _phase * 0.15f, 1.0f);
        const float h2 = fmodf(static_cast<float>(i) / _count - _phase * 0.11f + 0.5f, 1.0f);
        uint8_t r1 = 0;
        uint8_t g1 = 0;
        uint8_t b1 = 0;
        uint8_t r2 = 0;
        uint8_t g2 = 0;
        uint8_t b2 = 0;
        hsv_color(h1, 1.0f, 0.55f, r1, g1, b1);
        hsv_color(h2, 1.0f, 0.45f, r2, g2, b2);
        set_pixel(i, (r1 + r2) / 2, (g1 + g2) / 2, (b1 + b2) / 2);
      }
      break;
    case kModeEscape:
      for (uint8_t i = 0; i < _count; i++) {
        const float h = fmodf(0.02f + i * 0.02f + _phase * 0.25f, 1.0f);
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        hsv_color(h, 1.0f, 0.8f, r, g, b);
        set_pixel(i, r, g, b);
      }
      break;
    case kModeSonar:
      for (uint8_t i = 0; i < _count; i++) {
        if (i == _sonar_sweep) {
          set_pixel(i, 40, 200, 255);
          continue;
        }
        const uint8_t d = _sonar_bins[i];
        if (d >= 255) {
          set_pixel(i, 0, 10, 24);
          continue;
        }
        const float tt = constrain(d / 120.0f, 0.0f, 1.0f);
        const uint8_t r = static_cast<uint8_t>(255.0f * (1.0f - tt * 0.85f));
        const uint8_t g = static_cast<uint8_t>(40.0f + 180.0f * tt);
        const uint8_t b = static_cast<uint8_t>(60.0f + 195.0f * tt);
        set_pixel(i, r, g, b);
      }
      break;
    default:
      for (uint8_t i = 0; i < _count; i++) {
        set_pixel(i, 0, 0, 0);
      }
      break;
  }
  g_px->show();
}

void RoverRing::tick(uint32_t now_ms) {
  if (!_ok) return;
  static uint32_t last_ms = 0;
  if (now_ms - last_ms < 40) {
    return;
  }
  last_ms = now_ms;
  _phase += 0.06f;
  render(now_ms);
}
