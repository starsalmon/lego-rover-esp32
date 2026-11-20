#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ESP-NOW PS4 bridge <-> fleet robots (Jazz ProS3, Roby C3, Rover T-Display, …).

#ifndef JAZZ_PS4_BRIDGE_MAC
#define JAZZ_PS4_BRIDGE_MAC 0xe0, 0x8c, 0xfe, 0xf9, 0xc2, 0x34
#endif

#ifndef FLEET_PS4_BRIDGE_MAC
#define FLEET_PS4_BRIDGE_MAC JAZZ_PS4_BRIDGE_MAC
#endif

#define FLEET_PS4_TX_MAGIC 0x5034u  // 'P4'
#define FLEET_PS4_ACK_MAGIC 0x4A41u  // 'JA'
#define FLEET_PS4_VERSION 1u

#define FLEET_PS4_FLAG_CTRL_OK 0x01u
#define FLEET_PS4_FLAG_HELLO 0x02u

#define FLEET_PS4_BTN_OPTIONS 0x0001u
#define FLEET_PS4_BTN_TRIANGLE 0x0002u
#define FLEET_PS4_BTN_CROSS 0x0004u
#define FLEET_PS4_BTN_SQUARE 0x0008u
#define FLEET_PS4_BTN_CIRCLE 0x0010u
#define FLEET_PS4_BTN_L1 0x0020u
#define FLEET_PS4_BTN_R1 0x0040u

#define FLEET_PS4_ACK_FB_SPEED_LOW 0x01u
#define FLEET_PS4_ACK_FB_STEER_LOW 0x02u
#define FLEET_PS4_ACK_FB_RUMBLE 0x04u

#ifndef FLEET_PS4_DEADZONE
#define FLEET_PS4_DEADZONE 10
#endif

#ifndef FLEET_PS4_FAILSAFE_MS
#define FLEET_PS4_FAILSAFE_MS 300u
#endif

#pragma pack(push, 1)
struct FleetPs4TxPkt {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint32_t seq;
  int8_t lx;
  int8_t ly;
  int8_t rx;
  int8_t ry;
  uint8_t l2;
  uint8_t r2;
  uint16_t buttons;
  uint8_t checksum;
};

struct FleetPs4AckPkt {
  uint16_t magic;
  uint8_t version;
  uint8_t channel;
  uint32_t seq;
  uint8_t mac[6];
  uint8_t feedback;
  uint8_t checksum;
};
#pragma pack(pop)

static inline uint8_t fleetPs4XorChecksum(const uint8_t* data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) {
    x ^= data[i];
  }
  return x;
}

static inline void fleetPs4TxSeal(FleetPs4TxPkt& p) {
  p.magic = FLEET_PS4_TX_MAGIC;
  p.version = FLEET_PS4_VERSION;
  p.checksum = 0;
  p.checksum = fleetPs4XorChecksum(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}

static inline bool fleetPs4TxValid(const FleetPs4TxPkt& p) {
  if (p.magic != FLEET_PS4_TX_MAGIC || p.version != FLEET_PS4_VERSION) {
    return false;
  }
  FleetPs4TxPkt tmp = p;
  tmp.checksum = 0;
  return fleetPs4XorChecksum(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp) - 1) == p.checksum;
}

static inline void fleetPs4AckSeal(FleetPs4AckPkt& p) {
  p.magic = FLEET_PS4_ACK_MAGIC;
  p.version = FLEET_PS4_VERSION;
  p.checksum = 0;
  p.checksum = fleetPs4XorChecksum(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}

static inline bool fleetPs4AckValid(const FleetPs4AckPkt& p) {
  if (p.magic != FLEET_PS4_ACK_MAGIC || p.version != FLEET_PS4_VERSION) {
    return false;
  }
  FleetPs4AckPkt tmp = p;
  tmp.checksum = 0;
  return fleetPs4XorChecksum(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp) - 1) == p.checksum;
}

static inline float fleetPs4AxisNorm(int8_t v) {
  const int a = v < 0 ? -v : v;
  if (a <= FLEET_PS4_DEADZONE) {
    return 0.0f;
  }
  const float n = (float)v / 127.0f;
  const float dz = (float)FLEET_PS4_DEADZONE / 127.0f;
  if (n > 0) {
    return (n - dz) / (1.0f - dz);
  }
  return (n + dz) / (1.0f - dz);
}

static inline bool fleetPs4LinkFailsafe(uint32_t now_ms, uint32_t last_pkt_ms, bool manual_active,
                                        bool ctrl_connected) {
  if (!manual_active || !ctrl_connected) {
    return false;
  }
  if (last_pkt_ms == 0) {
    return true;
  }
  return (uint32_t)(now_ms - last_pkt_ms) > FLEET_PS4_FAILSAFE_MS;
}