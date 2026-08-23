#pragma once

#include <stdint.h>

// Shared with Pi (rover_esp_button.py) — keep in sync.
constexpr uint32_t ROVER_GO_LONG_MS = 800;
constexpr uint32_t ROVER_GO_DEBOUNCE_MS = 80;
constexpr uint32_t ROVER_GO_MULTI_GAP_MS = 1200;     // max pause between taps in one gesture
constexpr uint32_t ROVER_GO_MULTI_SETTLE_MS = 1200;  // idle after last tap before single-tap fires
constexpr uint32_t ROVER_GO_MULTI_TAPS = 2;          // taps to open power menu (was 3)
constexpr uint32_t ROVER_ESTOP_LONG_MS = 3000;

enum RoverButtonEvent : uint8_t {
  kBtnNone = 0,
  kBtnGoShort = 1,        // start/stop toggle, or cycle while menu open
  kBtnGoLong = 2,         // open/close mode menu, or confirm power menu
  kBtnEstopLong = 3,      // long E-stop — stop session
  kBtnGoTriple = 4,       // multi-tap Go — open power menu (2 taps)
  kBtnPowerShutdown = 5,  // confirm shutdown in power menu
  kBtnPowerReboot = 6,    // confirm reboot in power menu
  kBtnMenuCycle = 7,      // cycled an on-screen menu option
};

enum RoverDriveMode : uint8_t {
  kDriveExplore = 0,
  kDriveWall = 1,
  kDriveServoCal = 2,  // bench-only: pan/sonar timing calibration (does not start motion)
  kDriveModeCount = 3,
};

enum RoverPowerAction : uint8_t {
  kPowerShutdown = 0,
  kPowerReboot = 1,
  kPowerCancel = 2,
  kPowerActionCount = 3,
};

inline const char *drive_mode_label(RoverDriveMode mode) {
  switch (mode) {
    case kDriveWall:
      return "Wall";
    case kDriveServoCal:
      return "ServoCal";
    default:
      return "Explore";
  }
}

inline const char *power_action_label(RoverPowerAction action) {
  switch (action) {
    case kPowerReboot:
      return "Reboot";
    case kPowerCancel:
      return "Cancel";
    default:
      return "Shutdown";
  }
}
