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
    _l8_bins[i] = 255;
  }
  uint8_t blank[ROVER_RING_COUNT];
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    blank[i] = 255;
  }
  // Dim "no range" until L8 / side ToF arrive. Not the standby hue show.
  _ring.sonar_frame(blank, ROVER_RING_COUNT, ROVER_RING_COUNT);
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
void RoverPeriph::play_code(uint32_t code) { _speaker.play_code(code); }

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
  // Do not replace the distance map with the auto/standby hue show.
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
  // LED follows the horn. SONAR_PAN_INVERT swaps 0↔180 on the servo, so
  // paint the inverted angle or the blob sits opposite the head.
  float p = pan_deg;
#if SONAR_PAN_INVERT
  p = 2.0f * static_cast<float>(SONAR_PAN_CENTER_DEG) - pan_deg;
#endif
  return rear_pan_to_led(p);
}

int RoverPeriph::rear_pan_to_led(float pan_deg) {
  // Rear aux scan uses the same 0..180° convention as sonar:
  // 0° = left, 90° = center, 180° = right — but centered on the *rear* (180° bearing).
  const float bearing =
      fmodf(static_cast<float>(SONAR_PAN_CENTER_DEG) - pan_deg + 180.0f + 360.0f, 360.0f);
  return bearing_to_led(bearing);
}

void RoverPeriph::note_l8_cols(uint32_t now_ms, const float* cols, uint8_t n) {
  if (!cols || n < 8) {
    return;
  }
  // Front half of the 8-LED ring, left → right: +45°, 0°, 315°, 270°.
  // L8 FOV is ~45°; we stretch it across the front 180° so empty vs wall is obvious.
  // Was 4 LEDs incl. 270°, which is the left side-ToF LED (side paint swaps L/R),
  // so the L8 overwrote it every frame. Now 3 front LEDs: cols 0-2 / 3-4 / 5-7.
  const int leds[3] = {
      bearing_to_led(45.0f),
      bearing_to_led(0.0f),
      bearing_to_led(315.0f),
  };
  static const uint8_t col_lo[3] = {0, 3, 5};
  static const uint8_t col_hi[3] = {2, 4, 7};
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    _l8_bins[i] = 255;
  }
  bool any = false;
  for (uint8_t b = 0; b < 3; b++) {
    float m = -1.0f;
    for (uint8_t k = col_lo[b]; k <= col_hi[b]; k++) {
      const float v = cols[k];
      if (v > 0.0f) {
        m = (m < 0.0f) ? v : fminf(m, v);
      }
    }
    const int led = leds[b];
    if (led < 0 || led >= ROVER_RING_COUNT) {
      continue;
    }
    if (m > 0.0f) {
      _l8_bins[led] = sonar_m_to_cm(m);
      any = true;
    }
  }
  if (any) {
    _l8_hold_until_ms = now_ms + 2000;
  } else {
    // No targets: still a reading. Front LEDs go dark so "empty L8" is visible.
    _l8_hold_until_ms = now_ms + 2000;
  }
}

// Rear sonar on the pan servo. Assumption: servo 90° (logical centre) points
// straight back (ring 180°); higher servo angles swing the sonar toward the
// robot's left, so ring bearing = 270 - servo_deg (0=fwd, 90=left, 180=rear).
// Readings go to the nearest rear LED (135/180/225; cut-offs at 157.5/202.5);
// bearings outside 112.5-247.5 aren't painted (they'd be the side LEDs).
// Servo inactive/unknown → 180° LED only.
// Each painted LED holds 0.5 s, then fades out over 1 s.
void RoverPeriph::note_rear_sonar(uint32_t now_ms, float m, float servo_deg) {
  if (m <= 0.02f) return;  // no reading: let the LED fade out
  int idx = 1;
  if (servo_deg >= 0.0f) {
    const float bearing = 270.0f - servo_deg;
    if (bearing < 112.5f || bearing > 247.5f) return;
    idx = bearing < 157.5f ? 0 : (bearing <= 202.5f ? 1 : 2);
  }
  _rear_bin[idx] = sonar_m_to_cm(m);
  _rear_ms[idx] = now_ms ? now_ms : 1;
}

void RoverPeriph::paint_rear(uint32_t now_ms, uint8_t* frame) {
  const float bearings[3] = {135.0f, 180.0f, 225.0f};
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) _ring.set_sonar_gain(i, 0);
  for (uint8_t b = 0; b < 3; b++) {
    const int led = bearing_to_led(bearings[b]);
    if (led < 0 || led >= ROVER_RING_COUNT) continue;
    const uint32_t age = _rear_ms[b] ? now_ms - _rear_ms[b] : 0xFFFFFFFFu;
    if (age >= 1500 || _rear_bin[b] >= 255) {
      frame[led] = 255;
      continue;
    }
    frame[led] = _rear_bin[b];
    if (age > 500) {
      const uint32_t g = 255u - ((age - 500u) * 254u) / 1000u;
      _ring.set_sonar_gain(static_cast<uint8_t>(led), static_cast<uint8_t>(g));
    }
  }
}

void RoverPeriph::tick_distance_ring(uint32_t now_ms) {
  static uint32_t last_ms = 0;
  if (now_ms - last_ms < 50) {
    return;
  }
  last_ms = now_ms;

  if (now_ms > _tof_hold_until_ms) {
    for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
      _tof_bins[i] = 255;
    }
  }

  uint8_t frame[ROVER_RING_COUNT];
  for (uint8_t i = 0; i < ROVER_RING_COUNT; i++) {
    frame[i] = 255;
  }
  const int led_leftmost = bearing_to_led(90.0f);
  const int led_rightmost = bearing_to_led(270.0f);
  if (led_leftmost >= 0 && led_leftmost < ROVER_RING_COUNT && _tof_bins[led_leftmost] < 255) {
    frame[led_leftmost] = _tof_bins[led_leftmost];
  }
  if (led_rightmost >= 0 && led_rightmost < ROVER_RING_COUNT && _tof_bins[led_rightmost] < 255) {
    frame[led_rightmost] = _tof_bins[led_rightmost];
  }
  // L8 publishes about every 1.5 s. Hold keeps the nose colors up across that gap.
  if (now_ms <= _l8_hold_until_ms) {
    // Front LEDs only; 270° is the left side-ToF LED (side paint swaps L/R).
    const int l8leds[3] = {
        bearing_to_led(45.0f),
        bearing_to_led(0.0f),
        bearing_to_led(315.0f),
    };
    for (uint8_t b = 0; b < 3; b++) {
      const int led = l8leds[b];
      if (led >= 0 && led < ROVER_RING_COUNT) {
        frame[led] = _l8_bins[led];
      }
    }
  }
  paint_rear(now_ms, frame);
  sonar_frame(frame, ROVER_RING_COUNT, ROVER_RING_COUNT);
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
  // L8 nose: front half of the ring (left→right 45°, 0°, 315°, 270°).
  // 255 = no target (open space or failed zone) — leave that LED dark.
  if (now_ms <= _l8_hold_until_ms) {
    // Front LEDs only; 270° is the left side-ToF LED (side paint swaps L/R).
    const int l8leds[3] = {
        bearing_to_led(45.0f),
        bearing_to_led(0.0f),
        bearing_to_led(315.0f),
    };
    for (uint8_t b = 0; b < 3; b++) {
      const int led = l8leds[b];
      if (led >= 0 && led < ROVER_RING_COUNT) {
        frame[led] = _l8_bins[led];
      }
    }
  }

  paint_rear(now_ms, frame);
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
