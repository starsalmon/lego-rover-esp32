#pragma once

#include <Arduino.h>

// Onboard WS2812 on ESP32-C3 Zero — GPIO10 (local status, not on ROS graph).

enum class LedState {
  kBoot,        // amber pulse — gyro cal / starting
  kMpuError,    // red solid — MPU missing
  kWaitAgent,   // blue slow pulse — no Pi agent
  kConnecting,  // cyan fast pulse — micro-ROS setup
  kReady,       // green dim breathe — linked, idle
  kDriving,     // green bright pulse — cmd_vel active
  kLinkLost,    // magenta flash — session dropped
};

enum class LedEvent {
  kBump,
  kStall,
};

class StatusLed {
 public:
  void begin(int pin);
  void set_state(LedState state);
  void trigger(LedEvent event);
  void tick(uint32_t now_ms);

 private:
  int _pin = -1;
  LedState _state = LedState::kBoot;
  LedEvent _flash = LedEvent::kBump;
  bool _flashing = false;
  uint32_t _flash_until = 0;

  static void _write(int pin, uint8_t r, uint8_t g, uint8_t b);
  static uint8_t _scale(uint8_t v, float gain);
  void _render(uint32_t now_ms);
};
