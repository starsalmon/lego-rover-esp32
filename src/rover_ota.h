#pragma once

#include <Arduino.h>

enum class OtaState : uint8_t {
  kDisabled = 0,
  kConnecting,
  kReady,
  kFailed,
};

class RoverOta {
 public:
  void begin(const char *hostname);
  /** WiFi already connected elsewhere (e.g. micro-ROS WiFi transport) — skip WiFi.begin(). */
  void beginConnected(const char *hostname);
  void tick();
  bool ready() const { return _state == OtaState::kReady; }
  bool busy() const { return _ota_busy; }
  OtaState state() const { return _state; }
  const char *ip() const { return _ip; }
  const char *hostname() const { return _hostname; }

 private:
  OtaState _state = OtaState::kDisabled;
  bool _ota_started = false;
  bool _ota_busy = false;
  uint32_t _connect_t0 = 0;
  char _ip[16] = {};
  const char *_hostname = "rover-esp";

  void _start_ota();
};
