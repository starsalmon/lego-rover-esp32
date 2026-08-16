#pragma once

#include <Arduino.h>

/** Simple UART link to Pi Zero (speaker + WS2812 ring only). See PI_PERIPHERAL.md */
class RoverPiPeriph {
 public:
  void begin(HardwareSerial& serial, uint32_t baud);
  void tick(uint32_t now_ms);

  void play(uint8_t melody_id);
  void ring_mode(uint8_t mode_id);
  void notify_bump();
  void notify_stall();
  void notify_escape();
  void notify_session(bool active);
  /** Live sonar ring: 8 distance bins (cm, 255=none) + sweep LED index. */
  void sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led);

  // Melody IDs — keep in sync with pi_peripheral_daemon.py
  static constexpr uint8_t kMelodyReady = 1;
  static constexpr uint8_t kMelodyAutoStart = 2;
  static constexpr uint8_t kMelodyAutoStop = 3;
  static constexpr uint8_t kMelodyBump = 4;
  static constexpr uint8_t kMelodyStall = 5;
  static constexpr uint8_t kMelodyFrontIr = 6;
  static constexpr uint8_t kMelodyButton = 7;
  static constexpr uint8_t kMelodyMenu = 8;

  // Ring mode IDs — keep in sync with rover_ring.py modes
  static constexpr uint8_t kRingStandby = 0;
  static constexpr uint8_t kRingReady = 1;
  static constexpr uint8_t kRingAuto = 2;
  static constexpr uint8_t kRingEscape = 3;
  static constexpr uint8_t kRingBump = 4;
  static constexpr uint8_t kRingStall = 5;

 private:
  HardwareSerial* _ser = nullptr;
  void send(uint8_t cmd, uint8_t arg = 0);
};
