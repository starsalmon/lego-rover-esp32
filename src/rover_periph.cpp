#include "rover_periph.h"

#include <math.h>

#ifndef SONAR_PAN_CENTER_DEG
#define SONAR_PAN_CENTER_DEG 90
#endif

bool RoverPeriph::begin(int speaker_pin, int ring_pin) {
  const bool spk = _speaker.begin(speaker_pin);
  const bool ring = _ring.begin(ring_pin);
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    _sonar_bins[i] = 255;
  }
  _ring.set_mode(kRingStandby);
  if (spk || ring) {
    Serial.printf("RoverPeriph speaker GPIO %d=%s ring GPIO %d=%s\n", speaker_pin,
                  spk ? "OK" : "--", ring_pin, ring ? "OK" : "--");
  }
  return spk || ring;
}

void RoverPeriph::tick(uint32_t now_ms) {
  _speaker.tick(now_ms);
  _ring.tick(now_ms);
}

void RoverPeriph::play(uint8_t melody_id) { _speaker.play(melody_id); }

void RoverPeriph::ring_mode(uint8_t mode_id) { _ring.set_mode(mode_id); }

void RoverPeriph::notify_bump() {
  play(kMelodyBump);
  _ring.flash(RoverRing::kFlashBump, 800);
}

void RoverPeriph::notify_stall() {
  play(kMelodyStall);
  _ring.flash(RoverRing::kFlashStall, 900);
}

void RoverPeriph::notify_escape() { ring_mode(kRingEscape); }

void RoverPeriph::notify_ready() {
  play(kMelodyReady);
  ring_mode(kRingReady);
}

void RoverPeriph::notify_session(bool active) {
  _session_active = active;
  play(active ? kMelodyAutoStart : kMelodyAutoStop);
  ring_mode(active ? kRingAuto : kRingStandby);
  if (active) {
    _ring.flash(RoverRing::kFlashStart, 700);
  } else {
    _ring.flash(RoverRing::kFlashStop, 700);
  }
}

void RoverPeriph::sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led) {
  _ring.sonar_frame(dist_cm, count, sweep_led);
}

uint8_t RoverPeriph::sonar_m_to_cm(float m) {
  if (m <= 0.0f) {
    return 255;
  }
  int cm = static_cast<int>(m * 100.0f);
  if (cm < 0) {
    cm = 0;
  }
  if (cm > 254) {
    cm = 254;
  }
  return static_cast<uint8_t>(cm);
}

int RoverPeriph::sonar_pan_to_led(float pan_deg) {
  const float bearing = fmodf(static_cast<float>(SONAR_PAN_CENTER_DEG) - pan_deg + 360.0f, 360.0f);
  const float step = 360.0f / static_cast<float>(ROVER_RING_COUNT);
  const float delta = fmodf(bearing - ROVER_RING_ZERO_BEARING + 360.0f, 360.0f);
  int led = static_cast<int>(lroundf(delta / step)) % ROVER_RING_COUNT;
  if (led < 0) {
    led += ROVER_RING_COUNT;
  }
  return led;
}

void RoverPeriph::tick_sonar_map(uint32_t now_ms, float pan_deg, float range_m, bool force_scan) {
  static uint32_t last_ms = 0;
  if (now_ms - last_ms < 50) {
    return;
  }
  last_ms = now_ms;

  const int sweep = sonar_pan_to_led(pan_deg);
  if (range_m > 0.0f) {
    _sonar_bins[sweep] = sonar_m_to_cm(range_m);
    _sonar_hold_until_ms = now_ms + 4000;
  }

  const bool off_center = fabsf(pan_deg - static_cast<float>(SONAR_PAN_CENTER_DEG)) > 10.0f;
  if (off_center || force_scan) {
    _sonar_scanning = true;
    _sonar_hold_until_ms = now_ms + 4000;
  } else if (_sonar_scanning && now_ms < _sonar_hold_until_ms) {
    // Keep bins while pan returns to centre.
  } else {
    _sonar_scanning = false;
  }

  if (!_sonar_scanning) {
    for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
      if (_sonar_bins[i] < 255) {
        _sonar_bins[i]++;
      }
    }
  }

  sonar_frame(_sonar_bins, ROVER_RING_COUNT, static_cast<uint8_t>(sweep));
}
