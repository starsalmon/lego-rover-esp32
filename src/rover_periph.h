#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "rover_pins_s3.h"
#include "rover_ring.h"
#include "rover_speaker.h"

/** On-board speaker + WS2812 ring (GPIO 17/18). Same cues as legacy Pi stack. */
class RoverPeriph {
 public:
  bool begin(int speaker_pin = PIN_ROVER_SPEAKER, int ring_pin = PIN_ROVER_RING);
  void tick(uint32_t now_ms);

  void play(uint8_t melody_id);
  void ring_mode(uint8_t mode_id);
  void notify_bump();
  void notify_stall();
  void notify_escape();
  void notify_session(bool active);
  void notify_ready();
  void sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led);

  void tick_sonar_map(uint32_t now_ms, float pan_deg, float range_m, bool scanning);
  void tick_ir_map(uint32_t now_ms, bool front_l, bool front_r);
  void note_ir_rear_sample(uint32_t now_ms, float pan_deg, bool hit);
  void note_side_tof(uint32_t now_ms, float left_m, float right_m);

  static constexpr uint8_t kMelodyReady = RoverSpeaker::kMelodyReady;
  static constexpr uint8_t kMelodyAutoStart = RoverSpeaker::kMelodyAutoStart;
  static constexpr uint8_t kMelodyAutoStop = RoverSpeaker::kMelodyAutoStop;
  static constexpr uint8_t kMelodyBump = RoverSpeaker::kMelodyBump;
  static constexpr uint8_t kMelodyStall = RoverSpeaker::kMelodyStall;
  static constexpr uint8_t kMelodyButton = RoverSpeaker::kMelodyButton;
  static constexpr uint8_t kMelodyMenu = RoverSpeaker::kMelodyMenu;
  static constexpr uint8_t kMelodyMenuDone = RoverSpeaker::kMelodyMenuDone;
  static constexpr uint8_t kMelodySonarPing = RoverSpeaker::kMelodySonarPing;
  static constexpr uint8_t kRingStandby = RoverRing::kModeStandby;
  static constexpr uint8_t kRingReady = RoverRing::kModeReady;
  static constexpr uint8_t kRingAuto = RoverRing::kModeAuto;
  static constexpr uint8_t kRingEscape = RoverRing::kModeEscape;

 private:
  static uint8_t sonar_m_to_cm(float m);
  static int sonar_pan_to_led(float pan_deg);
  static int bearing_to_led(float bearing_deg);
  static int rear_pan_to_led(float pan_deg);

  RoverSpeaker _speaker;
  RoverRing _ring;
  uint8_t _sonar_bins[ROVER_RING_COUNT];
  uint8_t _ir_bins[ROVER_RING_COUNT];
  uint8_t _tof_bins[ROVER_RING_COUNT];
  uint32_t _sonar_hold_until_ms = 0;
  bool _sonar_scanning = false;
  bool _session_active = false;
  uint32_t _sonar_ping_until_ms = 0;
  int _sonar_last_ping_led = -1;
  uint32_t _ir_hold_until_ms = 0;
  uint32_t _tof_hold_until_ms = 0;
};
