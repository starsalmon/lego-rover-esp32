#include "rover_speaker.h"

#include <string.h>

namespace {

constexpr uint8_t kDutyBits = 8;
constexpr uint8_t kDutyOn = (1u << kDutyBits) / 3;
constexpr uint16_t kNoteGapMs = 5;
constexpr float kVol = 0.48f;

// Notification cues — “Nokia-ish” bleeps: short, stepwise, slightly moody.
// Keep these as UI pips, not toy chirps and not single microwave beeps.

struct Note {
  uint16_t hz;
  uint16_t ms;
};

struct MelodyDef {
  Note notes[24];
  uint8_t count;
};

#define NOTE(hz, ms) \
  { static_cast<uint16_t>(hz), static_cast<uint16_t>(ms) }

// Equal temperament (A4 = 440 Hz).
enum : uint16_t {
  E3 = 165,
  F3 = 175,
  G3 = 196,
  A3 = 220,
  B3 = 247,
  C4 = 262,
  D4 = 294,
  E4 = 330,
  F4 = 349,
  Fs4 = 370,
  G4 = 392,
  A4 = 440,
  B4 = 494,
  C5 = 523,
  D5 = 587,
  E5 = 659,
  Fs5 = 740,
  G5 = 784,
  A5 = 880,
};

// Kernkraft 400 hook from Cain's MIDI (140 BPM, 4/4, PPQ 384).
// Every melody note is an 8th (192 ticks ≈ 214 ms), not a 16th.
// Sounding 209 ms + kNoteGapMs 5 ≈ 214 ms so pitch changes still speak.
// The twist is G–F#–D–E (not G–F#–E–D). hz 0 = rest.
constexpr uint16_t kE8 = 209;   // eighth @ 140 BPM
constexpr uint16_t kQ = 424;    // quarter
constexpr uint16_t kH = 854;    // half (the hole after the pickup)

static const MelodyDef kMelodies[] = {
    {},  // 0 unused
    // ready — pickup, 2-beat hole, then the hook (startup connect cue)
    {{NOTE(B4, kE8), NOTE(D5, kE8), NOTE(E5, kE8), NOTE(Fs5, kE8), NOTE(0, kH), NOTE(B4, kE8),
      NOTE(D5, kE8), NOTE(E5, kE8), NOTE(Fs5, kE8), NOTE(G5, kE8), NOTE(Fs5, kE8), NOTE(D5, kE8),
      NOTE(E5, kE8), NOTE(D5, kE8), NOTE(Fs5, kE8), NOTE(B4, kQ)},
     16},
    // auto start — just the hook
    {{NOTE(B4, kE8), NOTE(D5, kE8), NOTE(E5, kE8), NOTE(Fs5, kE8), NOTE(G5, kE8), NOTE(Fs5, kE8),
      NOTE(D5, kE8), NOTE(E5, kQ)},
     8},
    // auto stop — reverse the hook
    {{NOTE(E5, kE8), NOTE(D5, kE8), NOTE(Fs5, kE8), NOTE(G5, kE8), NOTE(Fs5, kE8), NOTE(E5, kE8),
      NOTE(D5, kE8), NOTE(B4, kQ)},
     8},
    // bump — restore original “thunk thunk”
    {{NOTE(A3, 55), NOTE(A3, 55), NOTE(E3, 62)}, 3},
    // stall — restore original “stuck”
    {{NOTE(G3, 62), NOTE(F4, 44), NOTE(D4, 62)}, 3},
    // front_ir — crisp warning (high staccato)
    {{NOTE(Fs5, 36), NOTE(Fs5, 36), NOTE(D5, 60)}, 3},
    // button — go pip (mini riff)
    {{NOTE(B4, 52), NOTE(D5, 52), NOTE(E5, 72)}, 3},
    // menu — short up/down (techy)
    {{NOTE(E5, 44), NOTE(Fs5, 44), NOTE(E5, 60)}, 3},
    // menu_done — confirm down
    {{NOTE(Fs5, 44), NOTE(E5, 44), NOTE(D5, 72)}, 3},
    // sonar_ping — quick “ping” (brighter)
    {{NOTE(Fs5, 16), NOTE(D5, 22)}, 2},
};

#undef NOTE

}  // namespace

bool RoverSpeaker::begin(int pin, int8_t ledc_channel) {
  _pin = pin;
  _ch = ledc_channel;
  _ok = false;
  if (_pin < 0) {
    return false;
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(static_cast<uint8_t>(_pin));
  if (!ledcAttachChannel(static_cast<uint8_t>(_pin), 1000, kDutyBits, _ch)) {
    return false;
  }
  ledcWrite(static_cast<uint8_t>(_pin), 0);
#else
  ledcDetachPin(static_cast<uint8_t>(_pin));
  ledcSetup(static_cast<uint8_t>(_ch), 1000, kDutyBits);
  ledcAttachPin(static_cast<uint8_t>(_pin), static_cast<uint8_t>(_ch));
  ledcWrite(static_cast<uint8_t>(_ch), 0);
#endif

  _ok = true;
  return true;
}

void RoverSpeaker::silence_pwm() {
  if (!_ok) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(static_cast<uint8_t>(_pin), 0);
  ledcWrite(static_cast<uint8_t>(_pin), 0);
#else
  ledcWriteTone(static_cast<uint8_t>(_ch), 0);
  ledcWrite(static_cast<uint8_t>(_ch), 0);
#endif
}

void RoverSpeaker::stop_tone() {
  silence_pwm();
  _melody = nullptr;
  _melody_len = 0;
  _note_idx = 0;
  _in_gap = false;
}

void RoverSpeaker::start_melody(uint8_t melody_id) {
  if (!_ok || melody_id == 0 || melody_id >= sizeof(kMelodies) / sizeof(kMelodies[0])) {
    return;
  }
  const MelodyDef& def = kMelodies[melody_id];
  if (def.count == 0) {
    return;
  }
  _melody = reinterpret_cast<const RoverSpeaker::Note*>(def.notes);
  _melody_len = def.count;
  _note_idx = 0;
  _in_gap = false;
  _note_until_ms = 0;
}

void RoverSpeaker::play(uint8_t melody_id) { start_melody(melody_id); }

void RoverSpeaker::play_named(const char* name) {
  if (name == nullptr) return;
  if (strcmp(name, "ready") == 0) {
    play(kMelodyReady);
  } else if (strcmp(name, "bump") == 0) {
    play(kMelodyBump);
  } else if (strcmp(name, "stall") == 0) {
    play(kMelodyStall);
  }
}

void RoverSpeaker::tick(uint32_t now_ms) {
  if (!_ok || _melody == nullptr) {
    return;
  }

  if (_note_until_ms != 0 && now_ms < _note_until_ms) {
    return;
  }

  if (_in_gap) {
    _in_gap = false;
    silence_pwm();
    _note_idx++;
    if (_note_idx >= _melody_len) {
      _melody = nullptr;
      return;
    }
    _note_until_ms = now_ms + kNoteGapMs;
    if (now_ms < _note_until_ms) {
      return;
    }
  }

  if (_note_idx >= _melody_len) {
    stop_tone();
    return;
  }

  const Note& n = _melody[_note_idx];
  if (n.hz == 0) {
    silence_pwm();
  } else {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(static_cast<uint8_t>(_pin), n.hz);
    ledcWrite(static_cast<uint8_t>(_pin), static_cast<uint32_t>(kDutyOn * kVol));
#else
    ledcWriteTone(static_cast<uint8_t>(_ch), n.hz);
    ledcWrite(static_cast<uint8_t>(_ch), static_cast<uint32_t>(kDutyOn * kVol));
#endif
  }
  _note_until_ms = now_ms + n.ms;
  _in_gap = true;
}
