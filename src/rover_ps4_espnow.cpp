#include "rover_ps4_espnow.h"

#if defined(ROVER_MICROROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#include "fleet_ps4_drive.h"
#include "fleet_ps4_pkt.h"
#include "wifi_roam.h"

#ifndef ROVER_PS4_MAX_LIN
#define ROVER_PS4_MAX_LIN 0.28f
#endif

#ifndef ROVER_PS4_MAX_ANG
#define ROVER_PS4_MAX_ANG 0.45f
#endif

#ifndef ROVER_PS4_STEER_SIGN
#define ROVER_PS4_STEER_SIGN 1.0f
#endif

#ifndef ROVER_PS4_SWAP_LIN_ANG
#define ROVER_PS4_SWAP_LIN_ANG 1
#endif

#ifndef ROVER_PS4_TASK_STACK
#define ROVER_PS4_TASK_STACK 4096
#endif

#ifndef ROVER_PS4_TASK_PRIO
#define ROVER_PS4_TASK_PRIO 6
#endif

#ifndef ROVER_PS4_TASK_MS
#define ROVER_PS4_TASK_MS 10
#endif

static const uint8_t kBridgeMac[6] = {FLEET_PS4_BRIDGE_MAC};

struct RxSlot {
  FleetPs4TxPkt pkt;
  volatile bool fresh;
};

static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ack_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_out_mux = portMUX_INITIALIZER_UNLOCKED;
static RxSlot s_rx;
static uint8_t s_peer_ch = 0;
static bool s_espnow_on = false;
static bool s_manual = false;
static bool s_ctrl_ok = false;
static bool s_speed_low = true;
static bool s_steer_low = false;
static bool s_pending_rumble = false;
static uint32_t s_last_pkt_ms = 0;
static uint32_t s_last_edge_ms = 0;
static int8_t s_lx = 0;
static uint8_t s_l2 = 0;
static uint8_t s_r2 = 0;
static uint16_t s_buttons_prev = 0;
static bool s_opt_prev = false;
static RoverPs4DriveOut s_drive_out;
static float s_throttle_now = 0.0f;
static uint32_t s_ack_tx = 0;
static volatile uint32_t s_rx_count = 0;
static TaskHandle_t s_task = nullptr;
static RoverPs4Hooks s_hooks = {};

static constexpr size_t kAckRingLen = 16;
static volatile uint32_t s_ack_ring[kAckRingLen];
static volatile uint8_t s_ack_r = 0;
static volatile uint8_t s_ack_w = 0;

static bool macEq(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

static bool pushAckSeqLocked(uint32_t seq) {
  const uint8_t w = s_ack_w;
  const uint8_t n = static_cast<uint8_t>((w + 1) % kAckRingLen);
  if (n == s_ack_r) {
    return false;
  }
  s_ack_ring[w] = seq;
  s_ack_w = n;
  return true;
}

static bool popAckSeqLocked(uint32_t* seq_out) {
  if (s_ack_r == s_ack_w) {
    return false;
  }
  *seq_out = s_ack_ring[s_ack_r];
  s_ack_r = static_cast<uint8_t>((s_ack_r + 1) % kAckRingLen);
  return true;
}

static void sendAck(uint32_t seq) {
  FleetPs4AckPkt ack{};
  const uint8_t ch = (uint8_t)WiFi.channel();
  ack.channel = ch ? ch : s_peer_ch;
  ack.seq = seq;
  WiFi.macAddress(ack.mac);
  ack.feedback = 0;
  if (s_speed_low) {
    ack.feedback |= FLEET_PS4_ACK_FB_SPEED_LOW;
  }
  if (s_steer_low) {
    ack.feedback |= FLEET_PS4_ACK_FB_STEER_LOW;
  }
  if (s_pending_rumble) {
    ack.feedback |= FLEET_PS4_ACK_FB_RUMBLE;
    s_pending_rumble = false;
  }
  fleetPs4AckSeal(ack);
  if (esp_now_send(kBridgeMac, reinterpret_cast<uint8_t*>(&ack), sizeof(ack)) == ESP_OK) {
    s_ack_tx++;
  }
}

static void drainAcks() {
  for (;;) {
    uint32_t seq = 0;
    bool got = false;
    portENTER_CRITICAL(&s_ack_mux);
    got = popAckSeqLocked(&seq);
    portEXIT_CRITICAL(&s_ack_mux);
    if (!got) {
      break;
    }
    sendAck(seq);
  }
}

// Arduino-ESP32 2.x (IDF 4.4): recv_cb(mac, data, len). 3.x+: esp_now_recv_info_t*.
static void IRAM_ATTR ingestEspNowRx(const uint8_t* mac, const uint8_t* data, int len) {
  if (!mac || !data || len != (int)sizeof(FleetPs4TxPkt)) {
    return;
  }
  if (!macEq(mac, kBridgeMac)) {
    return;
  }
  FleetPs4TxPkt pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (!fleetPs4TxValid(pkt)) {
    return;
  }
  portENTER_CRITICAL_ISR(&s_ack_mux);
  pushAckSeqLocked(pkt.seq);
  portEXIT_CRITICAL_ISR(&s_ack_mux);
  s_rx_count++;
  portENTER_CRITICAL_ISR(&s_rx_mux);
  s_rx.pkt = pkt;
  s_rx.fresh = true;
  portEXIT_CRITICAL_ISR(&s_rx_mux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void IRAM_ATTR onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info) {
    return;
  }
  ingestEspNowRx(info->src_addr, data, len);
}
#else
static void IRAM_ATTR onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  ingestEspNowRx(mac, data, len);
}
#endif

static bool ensurePeer() {
  if (!s_espnow_on) {
    if (esp_now_init() != ESP_OK) {
      Serial.println(F("PS4: ESP-NOW init failed"));
      return false;
    }
    esp_now_register_recv_cb(onEspNowRecv);
    s_espnow_on = true;
  }
  uint8_t ch = (uint8_t)WiFi.channel();
  if (ch == 0) {
    ch = s_peer_ch;
  }
  if (ch == 0) {
    return s_peer_ch != 0;
  }
  if (s_peer_ch == ch) {
    return true;
  }
  if (s_peer_ch != 0) {
    esp_now_del_peer(kBridgeMac);
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, kBridgeMac, 6);
  peer.channel = ch;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    return false;
  }
  s_peer_ch = ch;
  Serial.printf("PS4: ESP-NOW peer on WiFi ch %u\n", (unsigned)ch);
  return true;
}

static bool ps4DebouncedEdge(uint32_t now_ms) {
  if ((uint32_t)(now_ms - s_last_edge_ms) < 300) {
    return false;
  }
  s_last_edge_ms = now_ms;
  return true;
}

static float effectiveMaxThrottle() { return s_speed_low ? ROVER_PS4_MAX_LIN * 0.5f : ROVER_PS4_MAX_LIN; }

static float effectiveMaxSteer() { return s_steer_low ? ROVER_PS4_MAX_ANG * 0.5f : ROVER_PS4_MAX_ANG; }

static void ps4SetTwistFromInputs(float* linear_x, float* angular_z) {
  if (!linear_x || !angular_z) {
    return;
  }
#if ROVER_PS4_SWAP_LIN_ANG
  *linear_x = fleetPs4SteerExpo(s_lx, effectiveMaxThrottle());
  const float turn = fleetPs4ThrottleFromTriggers(s_l2, s_r2, effectiveMaxSteer(), s_throttle_now);
  s_throttle_now = turn;
  *angular_z = turn * ROVER_PS4_STEER_SIGN;
#else
  const auto sp = fleetPs4ScaleManual(s_l2, s_r2, s_lx, effectiveMaxThrottle(), effectiveMaxSteer(),
                                       s_throttle_now);
  s_throttle_now = sp.throttle;
  *linear_x = sp.throttle;
  *angular_z = sp.steering * ROVER_PS4_STEER_SIGN;
#endif
}

static void applyMotors(const RoverPs4DriveOut& out) {
  if (!s_hooks.apply_twist || !s_hooks.motor_stop) {
    return;
  }
  if (s_hooks.drive_cut && s_hooks.drive_cut()) {
    s_hooks.motor_stop();
    return;
  }
  if (out.override_ros) {
    s_hooks.apply_twist(out.linear_x, out.angular_z);
  } else if (out.failsafe) {
    s_hooks.motor_stop();
  }
}

static void processOneTick(uint32_t now_ms) {
  if (WiFi.status() == WL_CONNECTED) {
    (void)ensurePeer();
  }

  drainAcks();

  FleetPs4TxPkt pkt;
  bool got = false;
  portENTER_CRITICAL(&s_rx_mux);
  if (s_rx.fresh) {
    pkt = s_rx.pkt;
    s_rx.fresh = false;
    got = true;
  }
  portEXIT_CRITICAL(&s_rx_mux);

  if (got) {
    s_last_pkt_ms = now_ms;
    s_ctrl_ok = (pkt.flags & FLEET_PS4_FLAG_CTRL_OK) != 0;
    s_lx = pkt.lx;
    s_l2 = pkt.l2;
    s_r2 = pkt.r2;

    const uint16_t edge = (uint16_t)(pkt.buttons & ~s_buttons_prev);
    s_buttons_prev = pkt.buttons;

    const bool opt_now = (pkt.buttons & FLEET_PS4_BTN_OPTIONS) != 0;
    if (opt_now && !s_opt_prev) {
      s_manual = !s_manual;
      Serial.printf("PS4: manual %s\n", s_manual ? "ON" : "OFF");
    }
    s_opt_prev = opt_now;

    if ((edge & FLEET_PS4_BTN_SQUARE) && ps4DebouncedEdge(now_ms)) {
      s_steer_low = !s_steer_low;
      s_pending_rumble = true;
      Serial.printf("PS4: steer mode %s\n", s_steer_low ? "LOW" : "HIGH");
    }
    if ((edge & FLEET_PS4_BTN_CIRCLE) && ps4DebouncedEdge(now_ms)) {
      s_speed_low = !s_speed_low;
      s_pending_rumble = true;
      Serial.printf("PS4: speed mode %s\n", s_speed_low ? "LOW" : "HIGH");
    }
  }

  const bool pkt_fresh =
      s_last_pkt_ms != 0 && (uint32_t)(now_ms - s_last_pkt_ms) < 2000;
  rover_wifi_roam_set_suppress(s_manual || pkt_fresh);

  RoverPs4DriveOut out{};

  const bool ctrl_lost = s_manual && !s_ctrl_ok;
  const bool link_failsafe = fleetPs4LinkFailsafe(now_ms, s_last_pkt_ms, s_manual, s_ctrl_ok);

  if (s_manual && !ctrl_lost && !link_failsafe) {
    out.override_ros = true;
    ps4SetTwistFromInputs(&out.linear_x, &out.angular_z);
  } else if (s_manual && ctrl_lost) {
    s_throttle_now = 0.0f;
    out.override_ros = true;
    out.linear_x = 0.0f;
    out.angular_z = 0.0f;
  } else if (s_manual && link_failsafe) {
    s_throttle_now = 0.0f;
    out.failsafe = true;
    out.override_ros = false;
  } else {
    s_throttle_now = 0.0f;
  }

  portENTER_CRITICAL(&s_out_mux);
  s_drive_out = out;
  portEXIT_CRITICAL(&s_out_mux);

  if (s_manual) {
    applyMotors(out);
  }
}

static void ps4Task(void* arg) {
  (void)arg;
  for (;;) {
    const uint32_t t0 = millis();
    processOneTick(t0);
    const uint32_t elapsed = millis() - t0;
    const uint32_t wait = elapsed >= ROVER_PS4_TASK_MS ? 1 : (ROVER_PS4_TASK_MS - elapsed);
    vTaskDelay(pdMS_TO_TICKS(wait));
  }
}

void rover_ps4_set_hooks(const RoverPs4Hooks* hooks) {
  if (hooks) {
    s_hooks = *hooks;
  } else {
    s_hooks = {};
  }
}

void rover_ps4_espnow_begin() {
  s_manual = false;
  s_last_pkt_ms = 0;
  s_throttle_now = 0.0f;
  s_speed_low = true;
  s_steer_low = false;
  s_ack_r = s_ack_w = 0;
  if (s_task == nullptr) {
    xTaskCreatePinnedToCore(ps4Task, "ps4_espnow", ROVER_PS4_TASK_STACK, nullptr, ROVER_PS4_TASK_PRIO, &s_task,
                            0);
    Serial.println("PS4: ESP-NOW task core0 ~10ms");
  }
}

bool rover_ps4_manual_active() { return s_manual; }

bool rover_ps4_straight_eligible(float* linear_out) {
  const uint32_t now = millis();
  if (!s_manual || !s_ctrl_ok) {
    return false;
  }
  if (fleetPs4LinkFailsafe(now, s_last_pkt_ms, s_manual, s_ctrl_ok)) {
    return false;
  }
#if ROVER_PS4_SWAP_LIN_ANG
  if (fabsf(s_throttle_now) > 0.02f) {
    return false;
  }
  const float lin_fwd = fleetPs4SteerExpo(s_lx, effectiveMaxThrottle());
  if (fabsf(lin_fwd) < 0.06f) {
    return false;
  }
  if (linear_out) {
    *linear_out = lin_fwd;
  }
#else
  if (fabsf(fleetPs4AxisNorm(s_lx)) > 0.02f) {
    return false;
  }
  if (fabsf(s_throttle_now) < 0.06f) {
    return false;
  }
  if (linear_out) {
    *linear_out = s_throttle_now;
  }
#endif
  return true;
}

void rover_ps4_link_status(RoverPs4LinkStatus* out) {
  if (!out) {
    return;
  }
  const uint32_t now = millis();
  out->espnow_ready = s_espnow_on && s_peer_ch != 0;
  out->linked = s_last_pkt_ms != 0 && (uint32_t)(now - s_last_pkt_ms) < 2000;
  out->manual = s_manual;
  out->ctrl_ok = s_ctrl_ok;
  out->speed_low = s_speed_low;
  out->steer_low = s_steer_low;
  out->channel = s_peer_ch;
  out->last_pkt_age_ms = s_last_pkt_ms ? (uint32_t)(now - s_last_pkt_ms) : UINT32_MAX;
  out->ack_tx = s_ack_tx;
  out->rx_count = s_rx_count;
}

void rover_ps4_espnow_tick(uint32_t now_ms) {
  // Legacy: task owns the link; loop may call this as a no-op fallback.
  (void)now_ms;
}

RoverPs4DriveOut rover_ps4_espnow_poll(uint32_t now_ms) {
  (void)now_ms;
  RoverPs4DriveOut out{};
  portENTER_CRITICAL(&s_out_mux);
  out = s_drive_out;
  portEXIT_CRITICAL(&s_out_mux);
  return out;
}

#endif