#pragma once

#include <Arduino.h>

// Onboard WS2812 on Unexpected Maker Pro S3[D]: data GPIO 18, rail via LDO2 (GPIO 17).

struct Pros3StatusLedInput {
  bool wifi_connected = false;
  bool uros_transport = false;  // Wi-Fi up and XRCE transport configured
  bool agent_connected = false;
  bool agent_reconnecting = false;  // session create in progress or tearing down
};

class Pros3StatusLed {
 public:
  void begin(int pin);
  void set_input(const Pros3StatusLedInput& in) { _in = in; }
  void tick(uint32_t now_ms);

 private:
  enum class Phase : uint8_t {
    kBoot,
    kWifiConnecting,
    kWifiNoAgent,
    kAgentOk,
    kAgentLost,
  };

  int _pin = -1;
  Pros3StatusLedInput _in;
  uint32_t _boot_until = 0;
  uint32_t _network_grace_until = 0;
  bool _ready = false;
  Phase _last_phase = Phase::kBoot;
  bool _last_on = false;
  uint32_t _last_show_ms = 0;

  Phase _phase(uint32_t now_ms) const;
  void _render(Phase phase, uint32_t now_ms);
  void _show(uint8_t r, uint8_t g, uint8_t b);
};