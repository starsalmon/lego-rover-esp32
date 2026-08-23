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
    _ir_bins[i] = 255;
    _tof_bins[i] = 255;
  }
  _ring.set_mode(kRingStandby);
  if (spk || ring) {
    Serial.printf("RoverPeriph speaker GPIO %d=%s ring GPIO %d=%s\n", speaker_pin,
                  spk ? "OK" : "--", ring_pin, ring ? "OK" : "--");
  }

#if defined(ARDUINO_ARCH_ESP32)
  // Speaker timing must not depend on the main loop cadence.
  // If the loop stalls (WiFi, I2C, display), tones get “dragged out”.
  // Run a tiny task to tick the speaker at a steady rate.
  if (spk && _speaker_task == nullptr) {
    xTaskCreatePinnedToCore(&RoverPeriph::speaker_task, "rover_spk", 2048, this, 2, &_speaker_task,
                            0);
  }
#endif

  return spk || ring;
}

void RoverPeriph::tick(uint32_t now_ms) {
  // If a dedicated speaker task exists, it owns speaker timing.
#if !defined(ARDUINO_ARCH_ESP32)
  _speaker.tick(now_ms);
#else
  if (_speaker_task == nullptr) {
    _speaker.tick(now_ms);
  }
#endif
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

int RoverPeriph::bearing_to_led(float bearing_deg) {
  const float step = 360.0f / static_cast<float>(ROVER_RING_COUNT);
  const float delta = fmodf(bearing_deg - ROVER_RING_ZERO_BEARING + 360.0f, 360.0f);
  int led = static_cast<int>(lroundf(delta / step)) % ROVER_RING_COUNT;
  if (led < 0) {
    led += ROVER_RING_COUNT;
  }
  return led;
}

int RoverPeriph::sonar_pan_to_led(float pan_deg) {
  const float bearing = fmodf(static_cast<float>(SONAR_PAN_CENTER_DEG) - pan_deg + 360.0f, 360.0f);
  return bearing_to_led(bearing);
}

int RoverPeriph::rear_pan_to_led(float pan_deg) {
  // Rear aux scan uses the same 0..180° convention as sonar:
  // 0° = left, 90° = center, 180° = right — but centered on the *rear* (180° bearing).
  const float bearing =
      fmodf(static_cast<float>(SONAR_PAN_CENTER_DEG) - pan_deg + 180.0f + 360.0f, 360.0f);
  return bearing_to_led(bearing);
}

void RoverPeriph::tick_ir_map(uint32_t now_ms, bool front_l, bool front_r) {
  // Fixed front bumper IR hits: paint them as "very close" in the forward-left/right bins.
  // These are binary (hit/no-hit) — treat as "closest range" (~5 cm).
  if (front_l || front_r) {
    _ir_hold_until_ms = now_ms + 2500;
  }
  // Bearing convention: 0=forward, 90=left, 180=rear, 270=right.
  const int led_fl = bearing_to_led(45.0f);   // forward-left
  const int led_fr = bearing_to_led(315.0f);  // forward-right
  if (front_l) {
    _ir_bins[led_fl] = 5;
  }
  if (front_r) {
    _ir_bins[led_fr] = 5;
  }
}

void RoverPeriph::note_side_tof(uint32_t now_ms, float left_m, float right_m) {
  // Side ToF overlay: ONLY the most-left and most-right LEDs.
  // Note: ring left/right were observed swapped vs the display, so we intentionally
  // swap the LED targets here (without touching the ToF topic names).
  const int led_leftmost = bearing_to_led(90.0f);
  const int led_rightmost = bearing_to_led(270.0f);
  const int led_l = led_rightmost;  // swap
  const int led_r = led_leftmost;   // swap
  if (left_m > 0.0f) {
    _tof_bins[led_l] = sonar_m_to_cm(left_m);
    _tof_hold_until_ms = now_ms + 1200;
  }
  if (right_m > 0.0f) {
    _tof_bins[led_r] = sonar_m_to_cm(right_m);
    _tof_hold_until_ms = now_ms + 1200;
  }
}

void RoverPeriph::note_ir_rear_sample(uint32_t now_ms, float pan_deg, bool hit) {
  if (!hit) {
    return;
  }
  const int led = rear_pan_to_led(pan_deg);
  if (led < 0 || led >= ROVER_RING_COUNT) {
    return;
  }
  // Rear AUX IR is binary: treat as "closest range" (~5 cm).
  _ir_bins[led] = 5;
  _ir_hold_until_ms = now_ms + 3500;
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
  const bool was_scanning = _sonar_scanning;
  if (off_center || force_scan) {
    _sonar_scanning = true;
    _sonar_hold_until_ms = now_ms + 6500;
  } else if (_sonar_scanning && now_ms < _sonar_hold_until_ms) {
    // Keep bins while pan returns to centre.
  } else {
    _sonar_scanning = false;
  }
  if (was_scanning && !_sonar_scanning) {
    // Persist the final scan map for a bit after a sweep ends.
    _sonar_post_scan_until_ms = now_ms + 6000;
  }

  // Sonar "ping" when sweeping and something is close.
  // Rate-limit so it feels like a sensor ping, not a toy ringtone.
  if (_sonar_scanning && range_m > 0.0f) {
    const bool close = range_m < 0.45f;
    if (close && now_ms >= _sonar_ping_until_ms && sweep != _sonar_last_ping_led) {
      _sonar_last_ping_led = sweep;
      _sonar_ping_until_ms = now_ms + 160;
      play(kMelodySonarPing);
    }
  }

  if (!_sonar_scanning) {
    // Hold the map briefly after a scan ends, then decay slowly.
    if (now_ms >= _sonar_post_scan_until_ms) {
      for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
        if (_sonar_bins[i] < 255) {
          _sonar_bins[i]++;
        }
      }
    }
    _sonar_last_ping_led = -1;
  }

  // Decay IR bins slowly when not in a recent IR-hold window.
  if (now_ms > _ir_hold_until_ms) {
    for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
      _ir_bins[i] = 255;
    }
  }

  // Decay ToF bins when stale.
  if (now_ms > _tof_hold_until_ms) {
    for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
      _tof_bins[i] = 255;
    }
  }

  // Build frame as: base sonar scan, then overlays on specific LEDs.
  uint8_t frame[ROVER_RING_COUNT];
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    frame[i] = _sonar_bins[i];
  }
  // Overlay ToF ONLY on leftmost/rightmost LEDs.
  const int led_leftmost = bearing_to_led(90.0f);
  const int led_rightmost = bearing_to_led(270.0f);
  if (led_leftmost >= 0 && led_leftmost < ROVER_RING_COUNT && _tof_bins[led_leftmost] < 255) {
    frame[led_leftmost] = _tof_bins[led_leftmost];
  }
  if (led_rightmost >= 0 && led_rightmost < ROVER_RING_COUNT && _tof_bins[led_rightmost] < 255) {
    frame[led_rightmost] = _tof_bins[led_rightmost];
  }
  // Overlay IR (binary) wherever it hit (front bumper bins + rear AUX sweep bins).
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    if (_ir_bins[i] < 255) {
      frame[i] = _ir_bins[i];
    }
  }

  sonar_frame(frame, ROVER_RING_COUNT, static_cast<uint8_t>(sweep));
}

#if defined(ARDUINO_ARCH_ESP32)
void RoverPeriph::speaker_task(void* arg) {
  auto* self = static_cast<RoverPeriph*>(arg);
  const TickType_t kDelay = pdMS_TO_TICKS(2);  // ~500 Hz tick => tight note timing
  for (;;) {
    const uint32_t now_ms = millis();
    self->_speaker.tick(now_ms);
    vTaskDelay(kDelay);
  }
}
#endif
