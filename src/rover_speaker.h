#pragma once

#include <Arduino.h>
#include <stdint.h>

/** Passive piezo on GPIO — non-blocking melody player (LEDC PWM). */
class RoverSpeaker {
 public:
  bool begin(int pin, int8_t ledc_channel = 5);
  void tick(uint32_t now_ms);

  void play(uint8_t melody_id);
  void play_named(const char* name);

  static constexpr uint8_t kMelodyReady = 1;
  static constexpr uint8_t kMelodyAutoStart = 2;
  static constexpr uint8_t kMelodyAutoStop = 3;
  static constexpr uint8_t kMelodyBump = 4;
  static constexpr uint8_t kMelodyStall = 5;
  static constexpr uint8_t kMelodyFrontIr = 6;
  static constexpr uint8_t kMelodyButton = 7;
  static constexpr uint8_t kMelodyMenu = 8;
  static constexpr uint8_t kMelodyMenuDone = 9;
  static constexpr uint8_t kMelodySonarPing = 10;

 private:
  struct Note {
    uint16_t hz;
    uint16_t ms;
  };

  void start_melody(uint8_t melody_id);
  void silence_pwm();
  void stop_tone();

  int _pin = -1;
  int8_t _ch = 5;
  bool _ok = false;
  const Note* _melody = nullptr;
  uint8_t _melody_len = 0;
  uint8_t _note_idx = 0;
  uint32_t _note_until_ms = 0;
  bool _in_gap = false;
};
