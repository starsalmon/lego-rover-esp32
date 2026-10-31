/**
 * Unexpected Maker Pro S3 — ToF / sonar micro-ROS agent client.
 *
 * Publishes the brain's expected topics under hostname rover-tof.local.
 * Drive / IMU / motors stay on T-Display (rover-esp.local).
 * Dual I2C: Wire=L8 @ ~800 kHz, Wire1=side VL53 + PCA9685.
 */

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <math.h>
#include <limits>
#include <esp_system.h>
#include <esp_wifi.h>

// ---- Wi-Fi diag build flags (all reversible) ----
#ifndef ROVER_WIFI_DIAG
#define ROVER_WIFI_DIAG 0
#endif
#ifndef ROVER_WIFI_TXPOWER          // wifi_power_t enum, 0 = leave default
#define ROVER_WIFI_TXPOWER 0
#endif
#ifndef ROVER_WIFI_FIRST            // join Wi-Fi before I2C / L8 firmware upload
#define ROVER_WIFI_FIRST 0
#endif
#ifndef ROVER_UROS_PING_MISS        // consecutive missed pings before agent lost
#define ROVER_UROS_PING_MISS 1
#endif
#ifndef ROVER_UROS_PING_MS
#define ROVER_UROS_PING_MS 100
#endif

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <driver/rmt.h>
#include "fleet_ir_proto.h"

#include <VL53L0X.h>
#include <VL53L1X.h>
#include <vl53l8cx.h>

#include "ir_tx.h"
#include "rover_ota.h"
#include "rover_pins_pros3.h"
#include "ultrasonic.h"

#ifndef ROVER_WIFI_SSID
#define ROVER_WIFI_SSID ""
#endif
#ifndef ROVER_WIFI_PASS
#define ROVER_WIFI_PASS ""
#endif
#ifndef ROVER_AGENT_IP
#define ROVER_AGENT_IP "192.168.3.172"
#endif
#ifndef ROVER_AGENT_PORT
#define ROVER_AGENT_PORT 8888
#endif
#ifndef ROVER_L8_SWAP_LR
#define ROVER_L8_SWAP_LR 1
#endif


// micro-ROS WiFi transport is patched to call these (shared with DIAG on T-Display).
// Pro S3 has no concurrent DIAG UDP — provide always-ok stubs.
extern "C" bool rover_net_lock(uint32_t timeout_ms) {
  (void)timeout_ms;
  return true;
}
extern "C" void rover_net_unlock(void) {}

namespace {

TwoWire& bus_l8 = Wire;
TwoWire bus_side = TwoWire(1);

VL53L8CX g_l8(&bus_l8, L8_LPN_PIN);
VL53L8CX_ResultsData g_l8_results;
bool g_l8_ok = false;
float g_l8_cols[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
// Full 8x8 grid in mm (0 = invalid), index row*8+col, same column order as l8_cols (LR swap applied).
uint16_t g_l8_grid_mm[64] = {0};
float g_l8_front = -1.0f;
float g_l8_front_l = -1.0f;
float g_l8_front_r = -1.0f;

enum class SideChip : uint8_t { kNone, kL1x, kL0x };
SideChip g_side_chip = SideChip::kNone;
VL53L1X g_side_l1_l;
VL53L1X g_side_l1_r;
VL53L0X g_side_l0_l;
VL53L0X g_side_l0_r;
bool g_side_l_ok = false;
bool g_side_r_ok = false;
float g_side_l_m = -1.0f;
float g_side_r_m = -1.0f;

Ultrasonic g_sonar;
bool g_sonar_ok = false;
float g_sonar_m = -1.0f;

IrTx g_ir_tx;
#ifndef ROVER_IR_BEACON_ON
#define ROVER_IR_BEACON_ON 1
#endif
// IR RX (TSOP on IR_RX_PIN, active-low): falling edges counted in an ISR.
volatile uint32_t g_ir_rx_edges = 0;
void IRAM_ATTR on_ir_rx_edge() { g_ir_rx_edges++; }
// IR TX: RMT channel 3 with hardware 38 kHz carrier. One fleet frame
// (sync + (id+1) bursts) every FLEET_IR_FRAME_US from a small task.
volatile uint32_t g_ir_tx_frames = 0;
bool g_ir_rmt_ok = false;
constexpr rmt_channel_t kIrRmtCh = RMT_CHANNEL_3;
rmt_item32_t g_ir_items[1 + FLEET_IR_MAX_ID + 2];
size_t g_ir_nitems = 0;
void ir_rmt_task(void*) {
  for (;;) {
    if (rmt_write_items(kIrRmtCh, g_ir_items, g_ir_nitems, true) == ESP_OK) g_ir_tx_frames++;
    vTaskDelay(pdMS_TO_TICKS(FLEET_IR_FRAME_US / 1000));
  }
}
bool begin_ir_rmt() {
  rmt_config_t c = RMT_DEFAULT_CONFIG_TX(static_cast<gpio_num_t>(IR_TX_PIN), kIrRmtCh);
  c.clk_div = 80;  // 1 us ticks
  c.tx_config.carrier_en = true;
  c.tx_config.carrier_freq_hz = IR_TX_HZ;
  c.tx_config.carrier_duty_percent = 33;
  c.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  c.tx_config.idle_output_en = true;
  c.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  if (rmt_config(&c) != ESP_OK || rmt_driver_install(kIrRmtCh, 0, 0) != ESP_OK) return false;
  g_ir_nitems = 0;
  g_ir_items[g_ir_nitems++] = {{{FLEET_IR_SYNC_US, 1, FLEET_IR_BIT_GAP_US, 0}}};
  for (int i = 0; i <= FLEET_IR_ID; i++) {
    g_ir_items[g_ir_nitems++] = {{{FLEET_IR_BURST_US, 1, FLEET_IR_BIT_GAP_US, 0}}};
  }
  g_ir_items[g_ir_nitems++] = {{{0, 0, 0, 0}}};
  return xTaskCreatePinnedToCore(ir_rmt_task, "irtx", 3072, nullptr, 2, nullptr, 1) == pdPASS;
}
RoverOta g_ota;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t tof_left_pub;
rcl_publisher_t tof_right_pub;
rcl_publisher_t tof_front_pub;
rcl_publisher_t tof_front_left_pub;
rcl_publisher_t tof_front_right_pub;
rcl_publisher_t tof_l8_cols_pub;
rcl_publisher_t tof_l8_grid_pub;
std_msgs__msg__UInt16MultiArray tof_l8_grid_msg;
uint16_t tof_l8_grid_data[64];
rcl_publisher_t sonar_range_pub;
// Pan servo on PCA9685 (bus_side) + combined status [sonar_m, servo_deg, pca_ok].
rcl_publisher_t tof_status_pub;
std_msgs__msg__Float32MultiArray tof_status_msg;
float tof_status_data[11];
// L8 diag: frames read, poll errors, valid zones / zones with no target in last frame, most common target_status
uint32_t g_l8_frames = 0, g_l8_poll_err = 0;
uint8_t g_l8_valid_z = 0, g_l8_notgt_z = 0, g_l8_top_status = 255;
rcl_subscription_t servo_angle_sub;
std_msgs__msg__Float32 servo_angle_msg;
bool g_pca_ok = false;
float g_servo_deg = 90.0f;   // logical: 90 = centre
volatile bool g_servo_dirty = false;
rcl_publisher_t ir_detected_pub;

sensor_msgs__msg__Range tof_left_msg;
sensor_msgs__msg__Range tof_right_msg;
sensor_msgs__msg__Range tof_front_msg;
sensor_msgs__msg__Range tof_front_left_msg;
sensor_msgs__msg__Range tof_front_right_msg;
sensor_msgs__msg__Range sonar_range_msg;
std_msgs__msg__Float32MultiArray tof_l8_cols_msg;
float tof_l8_cols_data[8];
std_msgs__msg__Int32 ir_detected_msg;

bool entities_ok = false;
bool uros_wifi_transport_ok = false;
IPAddress g_agent_ip;

enum class RosState { kWaitAgent, kConnect, kConnected, kDisconnect };
RosState ros_state = RosState::kWaitAgent;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t rccheck_ret = (fn);  \
    if (rccheck_ret != RCL_RET_OK) \
      return false;               \
  }
#define RCSOFTCHECK(fn) (void)(fn)

bool i2c_present(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

void scan_bus(TwoWire& bus, const char* name) {
  uint8_t found = 0;
  Serial.printf("I2C scan %s:", name);
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.print(" (empty)");
  }
  Serial.println();
}

void enable_ldo2() {
  pinMode(PIN_PROS3_LDO2, OUTPUT);
  digitalWrite(PIN_PROS3_LDO2, HIGH);
  delay(50);
  Serial.printf("LDO2 GPIO%d HIGH\n", PIN_PROS3_LDO2);
}

void set_antenna_external(bool external) {
  pinMode(PIN_PROS3_ANT_SEL, OUTPUT);
  digitalWrite(PIN_PROS3_ANT_SEL, external ? HIGH : LOW);
  Serial.printf("RF ant GPIO%d -> %s\n", PIN_PROS3_ANT_SEL,
                external ? "u.FL external" : "onboard ceramic");
}


void set_xshut(bool left_on, bool right_on) {
  digitalWrite(VL53_XSHUT_L, left_on ? HIGH : LOW);
  digitalWrite(VL53_XSHUT_R, right_on ? HIGH : LOW);
}

bool side_is_l0x_at_default() {
  // VL53L0X IDENTIFICATION_MODEL_ID @ 0xC0 reads 0xEE
  bus_side.beginTransmission(0x29);
  bus_side.write(0xC0);
  if (bus_side.endTransmission(false) != 0) {
    return false;
  }
  if (bus_side.requestFrom((uint8_t)0x29, (uint8_t)1) != 1) {
    return false;
  }
  return bus_side.read() == 0xEE;
}

bool setup_side_l1(VL53L1X& sensor, uint8_t addr) {
  sensor.setBus(&bus_side);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.printf("Side L1X init fail (want 0x%02X)\n", addr);
    return false;
  }
  if (addr != 0x29) {
    sensor.setAddress(addr);
  }
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(50);
  delay(60);
  return true;
}

bool setup_side_l0(VL53L0X& sensor, uint8_t addr) {
  sensor.setBus(&bus_side);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.printf("Side L0X init fail (want 0x%02X)\n", addr);
    return false;
  }
  if (addr != 0x29) {
    sensor.setAddress(addr);
  }
  sensor.startContinuous(50);
  delay(60);
  return true;
}

bool begin_sides_l1_dual() {
  set_xshut(true, false);
  delay(80);
  g_side_l_ok = setup_side_l1(g_side_l1_l, VL53L_ADDR_L);
  if (!g_side_l_ok) {
    set_xshut(false, false);
    delay(20);
  }
  set_xshut(g_side_l_ok, true);
  delay(80);
  g_side_r_ok = setup_side_l1(g_side_l1_r, VL53L_ADDR_R);
  if (!g_side_r_ok) {
    set_xshut(g_side_l_ok, false);
  } else {
    set_xshut(g_side_l_ok, true);
  }
  return g_side_l_ok || g_side_r_ok;
}

bool begin_sides_l0_dual() {
  set_xshut(true, false);
  delay(80);
  g_side_l_ok = setup_side_l0(g_side_l0_l, VL53L_ADDR_L);
  if (!g_side_l_ok) {
    set_xshut(false, false);
    delay(20);
  }
  set_xshut(g_side_l_ok, true);
  delay(80);
  g_side_r_ok = setup_side_l0(g_side_l0_r, VL53L_ADDR_R);
  if (!g_side_r_ok) {
    set_xshut(g_side_l_ok, false);
  } else {
    set_xshut(g_side_l_ok, true);
  }
  return g_side_l_ok || g_side_r_ok;
}

bool begin_sides() {
  pinMode(VL53_XSHUT_L, OUTPUT);
  pinMode(VL53_XSHUT_R, OUTPUT);
  bus_side.setTimeOut(500);
  set_xshut(false, false);
  delay(50);

  const bool at29_after_shutdown = i2c_present(bus_side, 0x29);
  Serial.printf("Side I2C pre-init: 0x29=%s (after XSHUT low)\n",
                at29_after_shutdown ? "yes" : "no");

  g_side_l_ok = false;
  g_side_r_ok = false;
  g_side_chip = SideChip::kNone;

  const bool prefer_l0 = at29_after_shutdown && side_is_l0x_at_default();
  if (prefer_l0) {
    Serial.println("Side VL53: detected VL53L0X at 0x29");
  }

  if (at29_after_shutdown) {
    // XSHUT ineffective — use lone default-address sensor as left, no addr change.
    set_xshut(true, true);
    delay(50);
    if (prefer_l0 || side_is_l0x_at_default()) {
      Serial.println("Side VL53: XSHUT ineffective; L0X @ 0x29 -> left");
      g_side_l_ok = setup_side_l0(g_side_l0_l, 0x29);
      if (g_side_l_ok) {
        g_side_chip = SideChip::kL0x;
      }
    } else {
      Serial.println("Side VL53: XSHUT ineffective; L1X @ 0x29 -> left");
      g_side_l_ok = setup_side_l1(g_side_l1_l, 0x29);
      if (g_side_l_ok) {
        g_side_chip = SideChip::kL1x;
      }
    }
  } else {
    // Dual-sensor XSHUT dance to 0x30 / 0x31. Prefer L1X, fall back to L0X.
    if (begin_sides_l1_dual()) {
      g_side_chip = SideChip::kL1x;
    } else {
      set_xshut(false, false);
      delay(50);
      if (begin_sides_l0_dual()) {
        g_side_chip = SideChip::kL0x;
        Serial.println("Side VL53: using VL53L0X driver");
      }
    }

    if (g_side_chip == SideChip::kNone) {
      set_xshut(true, true);
      delay(80);
      const bool at29 = i2c_present(bus_side, 0x29);
      const bool at30 = i2c_present(bus_side, VL53L_ADDR_L);
      const bool at31 = i2c_present(bus_side, VL53L_ADDR_R);
      Serial.printf("Side fallback scan: 0x29=%s 0x30=%s 0x31=%s\n",
                    at29 ? "yes" : "no", at30 ? "yes" : "no", at31 ? "yes" : "no");
      if (at29 && !at30 && !at31) {
        if (side_is_l0x_at_default()) {
          Serial.println("Side VL53: single L0X at 0x29 -> left");
          g_side_l_ok = setup_side_l0(g_side_l0_l, 0x29);
          if (g_side_l_ok) {
            g_side_chip = SideChip::kL0x;
          }
        } else {
          Serial.println("Side VL53: single L1X at 0x29 -> left");
          g_side_l_ok = setup_side_l1(g_side_l1_l, 0x29);
          if (g_side_l_ok) {
            g_side_chip = SideChip::kL1x;
          }
        }
      }
    }
  }

  bus_side.setTimeOut(80);
  const char* chip = g_side_chip == SideChip::kL0x ? "L0X"
                     : g_side_chip == SideChip::kL1x ? "L1X"
                                                     : "none";
  Serial.printf("Side VL53 (%s) L=%s R=%s\n", chip, g_side_l_ok ? "OK" : "--",
                g_side_r_ok ? "OK" : "--");
  return g_side_l_ok || g_side_r_ok;
}

bool begin_l8() {
  g_l8_ok = false;
  for (int i = 0; i < 8; i++) {
    g_l8_cols[i] = -1.0f;
  }
  g_l8_front = g_l8_front_l = g_l8_front_r = -1.0f;

  bus_l8.setClock(L8_I2C_INIT_HZ);
  bus_l8.setTimeOut(400);
  g_l8.begin();
  g_l8.on();
  delay(20);

  uint8_t alive = 0;
  if (g_l8.is_alive(&alive) != 0 || alive == 0) {
    Serial.println("VL53L8CX not alive at 0x29");
    bus_l8.setClock(L8_I2C_HZ);
    bus_l8.setTimeOut(50);
    return false;
  }

  Serial.println("VL53L8CX init (firmware load)...");
  if (g_l8.init() != 0) {
    Serial.println("VL53L8CX init failed");
    bus_l8.setClock(L8_I2C_HZ);
    bus_l8.setTimeOut(50);
    return false;
  }

  g_l8.set_resolution(VL53L8CX_RESOLUTION_8X8);
  g_l8.set_ranging_frequency_hz(10);
  g_l8.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
  if (g_l8.start_ranging() != 0) {
    Serial.println("VL53L8CX start_ranging failed");
    bus_l8.setClock(L8_I2C_HZ);
    bus_l8.setTimeOut(50);
    return false;
  }

  bus_l8.setClock(L8_I2C_HZ);
  bus_l8.setTimeOut(80);
  g_l8_ok = true;
  Serial.printf("VL53L8CX 8x8 OK bus %d/%d @ %u Hz\n", L8_SDA, L8_SCL,
                (unsigned)L8_I2C_HZ);
  return true;
}

bool zone_ok(const VL53L8CX_ResultsData& d, uint8_t z) {
  if (z >= VL53L8CX_RESOLUTION_8X8) {
    return false;
  }
  if (d.nb_target_detected[z] < 1) {
    return false;
  }
  const uint8_t st = d.target_status[VL53L8CX_NB_TARGET_PER_ZONE * z];
  return st == 5 || st == 6 || st == 9;
}

float zone_m(const VL53L8CX_ResultsData& d, uint8_t z) {
  const uint16_t mm = d.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * z];
  if (mm < 20 || mm > 4000) {
    return -1.0f;
  }
  return static_cast<float>(mm) * 0.001f;
}

void poll_l8() {
  if (!g_l8_ok) {
    return;
  }
  uint8_t ready = 0;
  if (g_l8.check_data_ready(&ready) != 0 || ready == 0) {
    return;
  }
  if (g_l8.get_ranging_data(&g_l8_results) != 0) {
    g_l8_poll_err++;
    return;
  }
  g_l8_frames++;
  {
    uint8_t hist[256] = {0};
    uint8_t valid = 0, notgt = 0;
    for (uint8_t z = 0; z < 64; z++) {
      if (g_l8_results.nb_target_detected[z] < 1) { notgt++; continue; }
      hist[g_l8_results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * z]]++;
      if (zone_ok(g_l8_results, z) && zone_m(g_l8_results, z) > 0.0f) valid++;
    }
    uint8_t top = 255;
    for (int i = 0, best = 0; i < 256; i++) if (hist[i] > best) { best = hist[i]; top = (uint8_t)i; }
    g_l8_valid_z = valid; g_l8_notgt_z = notgt; g_l8_top_status = top;
  }

  for (uint8_t z = 0; z < 64; z++) {
    uint8_t col = z % 8;
#if ROVER_L8_SWAP_LR
    col = 7 - col;
#endif
    const float v = zone_ok(g_l8_results, z) ? zone_m(g_l8_results, z) : -1.0f;
    g_l8_grid_mm[(z / 8) * 8 + col] = (v > 0.02f) ? static_cast<uint16_t>(v * 1000.0f + 0.5f) : 0;
  }

  float cols[8];
  for (uint8_t col = 0; col < 8; col++) {
    float obst = -1.0f;
    for (uint8_t row = 0; row < 4; row++) {
      const uint8_t z = static_cast<uint8_t>(row * 8 + col);
      if (!zone_ok(g_l8_results, z)) {
        continue;
      }
      const float v = zone_m(g_l8_results, z);
      if (v > 0.02f) {
        obst = (obst < 0.0f) ? v : fminf(obst, v);
      }
    }
    cols[col] = obst;
  }

#if ROVER_L8_SWAP_LR
  for (uint8_t i = 0; i < 4; i++) {
    const float t = cols[i];
    cols[i] = cols[7 - i];
    cols[7 - i] = t;
  }
#endif

  float front = -1.0f;
  float left = -1.0f;
  float right = -1.0f;
  for (uint8_t col = 2; col <= 5; col++) {
    if (cols[col] > 0.02f) {
      front = (front < 0.0f) ? cols[col] : fminf(front, cols[col]);
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    if (cols[i] > 0.02f) {
      left = (left < 0.0f) ? cols[i] : fminf(left, cols[i]);
    }
    if (cols[7 - i] > 0.02f) {
      right = (right < 0.0f) ? cols[7 - i] : fminf(right, cols[7 - i]);
    }
  }

  for (uint8_t i = 0; i < 8; i++) {
    g_l8_cols[i] = cols[i];
  }
  g_l8_front = front;
  g_l8_front_l = left;
  g_l8_front_r = right;
}

float side_mm_to_m(uint16_t mm, bool l0x) {
  if (mm == 0) {
    return -1.0f;
  }
  // L0X "no target" often returns ~8190 mm
  if (l0x && (mm < 40 || mm > 2000)) {
    return -1.0f;
  }
  return static_cast<float>(mm) * 0.001f;
}

void poll_sides() {
  const bool l0 = g_side_chip == SideChip::kL0x;
  if (g_side_l_ok) {
    const uint16_t mm = l0 ? g_side_l0_l.readRangeContinuousMillimeters()
                           : g_side_l1_l.read(false);
    g_side_l_m = side_mm_to_m(mm, l0);
  }
  if (g_side_r_ok) {
    const uint16_t mm = l0 ? g_side_l0_r.readRangeContinuousMillimeters()
                           : g_side_l1_r.read(false);
    g_side_r_m = side_mm_to_m(mm, l0);
  }
}

void fill_range(sensor_msgs__msg__Range& msg, float m, float max_m) {
  msg.range = (m > 0.0f) ? m : std::numeric_limits<float>::quiet_NaN();
  msg.min_range = 0.02f;
  msg.max_range = max_m;
  msg.field_of_view = 0.47f;
  msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
}

bool pca_write(uint8_t reg, uint8_t v) {
  bus_side.beginTransmission(0x40);
  bus_side.write(reg);
  bus_side.write(v);
  return bus_side.endTransmission() == 0;
}
void pca_servo_write(float logical_deg) {
  if (!g_pca_ok) return;
  float d = 180.0f - logical_deg - 5.0f;  // same invert + trim as the old T-Display pan
  if (d < 0.0f) d = 0.0f;
  if (d > 180.0f) d = 180.0f;
  const float us = 500.0f + (d / 180.0f) * 2000.0f;
  const uint16_t off = static_cast<uint16_t>(us * 4096.0f / 20000.0f);
  bus_side.beginTransmission(0x40);
  bus_side.write(0x06);  // LED0_ON_L (channel 0)
  bus_side.write(0);
  bus_side.write(0);
  bus_side.write(off & 0xFF);
  bus_side.write(off >> 8);
  bus_side.endTransmission();
}
bool begin_pca() {
  if (!i2c_present(bus_side, 0x40)) {
    Serial.println("PCA9685 0x40 not found on side bus");
    return false;
  }
  const uint8_t prescale = static_cast<uint8_t>(25000000UL / (4096UL * 50) - 1);
  pca_write(0x00, 0x10);  // sleep
  pca_write(0xFE, prescale);
  pca_write(0x00, 0x00);
  delay(5);
  pca_write(0x00, 0xA0);  // restart + auto-increment
  pca_write(0x01, 0x04);
  g_pca_ok = true;
  pca_servo_write(g_servo_deg);
  Serial.printf("PCA9685 OK, servo ch0 @ %.0f deg\n", g_servo_deg);
  return true;
}
void on_servo_angle(const void* msgin) {
  const auto* m = static_cast<const std_msgs__msg__Float32*>(msgin);
  float d = m->data;
  if (d < 0.0f) d = 0.0f;
  if (d > 180.0f) d = 180.0f;
  g_servo_deg = d;
  g_servo_dirty = true;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "lego_rover_tof", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &tof_left_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/tof/left"));
  RCCHECK(rclc_publisher_init_default(
      &tof_right_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/tof/right"));
  RCCHECK(rclc_publisher_init_default(
      &tof_front_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/tof/front"));
  RCCHECK(rclc_publisher_init_default(
      &tof_front_left_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/tof/front_left"));
  RCCHECK(rclc_publisher_init_default(
      &tof_front_right_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/tof/front_right"));
  RCCHECK(rclc_publisher_init_default(
      &tof_l8_cols_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "rover/tof/l8_cols"));
  RCCHECK(rclc_publisher_init_default(
      &tof_l8_grid_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
      "rover/tof/l8_grid"));
  RCCHECK(rclc_publisher_init_default(
      &sonar_range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "rover/sonar/range"));
  RCCHECK(rclc_publisher_init_default(
      &tof_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "rover/tof/status"));
  RCCHECK(rclc_subscription_init_default(
      &servo_angle_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "rover/servo/angle"));
  tof_status_msg.layout.dim.data = nullptr;
  tof_status_msg.layout.dim.size = 0;
  tof_status_msg.layout.dim.capacity = 0;
  tof_status_msg.layout.data_offset = 0;
  tof_status_msg.data.data = tof_status_data;
  tof_status_msg.data.size = 11;
  tof_status_msg.data.capacity = 6;
  RCCHECK(rclc_publisher_init_default(
      &ir_detected_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "rover/tof/ir_rx_edges"));

  fill_range(tof_left_msg, -1.0f, 1.3f);
  tof_right_msg = tof_left_msg;
  fill_range(tof_front_msg, -1.0f, 4.0f);
  tof_front_msg.field_of_view = 0.80f;
  tof_front_left_msg = tof_front_msg;
  tof_front_right_msg = tof_front_msg;
  fill_range(sonar_range_msg, -1.0f, 4.0f);
  sonar_range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  sonar_range_msg.field_of_view = 0.26f;

  tof_l8_cols_msg.layout.dim.data = nullptr;
  tof_l8_cols_msg.layout.dim.size = 0;
  tof_l8_cols_msg.layout.dim.capacity = 0;
  tof_l8_cols_msg.layout.data_offset = 0;
  tof_l8_cols_msg.data.data = tof_l8_cols_data;
  tof_l8_cols_msg.data.size = 8;
  tof_l8_cols_msg.data.capacity = 8;
  tof_l8_grid_msg.layout.dim.data = nullptr;
  tof_l8_grid_msg.layout.dim.size = 0;
  tof_l8_grid_msg.layout.dim.capacity = 0;
  tof_l8_grid_msg.layout.data_offset = 0;
  tof_l8_grid_msg.data.data = tof_l8_grid_data;
  tof_l8_grid_msg.data.size = 64;
  tof_l8_grid_msg.data.capacity = 64;
  memset(tof_l8_grid_data, 0, sizeof(tof_l8_grid_data));
  for (uint8_t i = 0; i < 8; i++) {
    tof_l8_cols_data[i] = std::numeric_limits<float>::quiet_NaN();
  }
  ir_detected_msg.data = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_angle_sub, &servo_angle_msg,
                                         &on_servo_angle, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  if (!entities_ok) {
    return;
  }
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_publisher_fini(&tof_left_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_right_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_front_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_front_left_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_front_right_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_l8_cols_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_l8_grid_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_range_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_status_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&servo_angle_sub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&ir_detected_pub, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
  entities_ok = false;
}

uint32_t g_diag_disc = 0, g_diag_up = 0, g_diag_lost = 0, g_diag_miss = 0;
uint8_t g_diag_last_reason = 0;
int g_rssi_min = 0, g_rssi_max = -127;
uint8_t g_ping_miss = 0;

const char* reset_reason_str(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

void diag_wifi_events() {
#if ROVER_WIFI_DIAG
  WiFi.onEvent([](WiFiEvent_t ev, WiFiEventInfo_t info) {
    if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      g_diag_disc++;
      g_diag_last_reason = info.wifi_sta_disconnected.reason;
      Serial.printf("[diag %lu] WIFI_DISC #%lu reason=%u\n", (unsigned long)millis(),
                    (unsigned long)g_diag_disc, (unsigned)g_diag_last_reason);
    } else if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("[diag %lu] WIFI_GOT_IP %s rssi=%d\n", (unsigned long)millis(),
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
  });
#endif
}

void wifi_start() {
  g_agent_ip.fromString(ROVER_AGENT_IP);
  diag_wifi_events();
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ROVER_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(ROVER_WIFI_SSID, ROVER_WIFI_PASS);
#if ROVER_WIFI_DIAG
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif
#if ROVER_WIFI_TXPOWER
  WiFi.setTxPower((wifi_power_t)ROVER_WIFI_TXPOWER);
#endif
  Serial.printf("WiFi connecting SSID=%s ", ROVER_WIFI_SSID);
  {
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000UL) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
  }
#if ROVER_WIFI_DIAG
  {
    wifi_ps_type_t ps; esp_wifi_get_ps(&ps);
    int8_t txp = 0; esp_wifi_get_max_tx_power(&txp);
    Serial.printf("[diag %lu] ps=%d txpower_q=%d (%.1f dBm)\n", (unsigned long)millis(),
                  (int)ps, (int)txp, txp / 4.0f);
  }
#endif
  if (WiFi.status() == WL_CONNECTED) {
    set_microros_wifi_transports(ROVER_WIFI_SSID, ROVER_WIFI_PASS, g_agent_ip,
                                 ROVER_AGENT_PORT);
    uros_wifi_transport_ok = true;
    Serial.printf("WiFi %s IP=%s -> agent %s:%d\n", ROVER_HOSTNAME,
                  WiFi.localIP().toString().c_str(), ROVER_AGENT_IP, ROVER_AGENT_PORT);
  } else {
    Serial.println("WiFi not up in 20s — sensors keep running; micro-ROS waits for link");
  }
}

void handle_ros_state() {
  switch (ros_state) {
    case RosState::kWaitAgent:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        ros_state = RosState::kConnect;
      }
      break;
    case RosState::kConnect:
      if (create_entities()) {
        entities_ok = true;
        ros_state = RosState::kConnected;
        g_ping_miss = 0;
        g_diag_up++;
        Serial.printf("[diag %lu] UROS_UP #%lu micro-ROS connected (lego_rover_tof)\n",
                      (unsigned long)millis(), (unsigned long)g_diag_up);
      } else {
        destroy_entities();
        ros_state = RosState::kWaitAgent;
      }
      break;
    case RosState::kConnected:
      {
        static uint32_t last_ping_ms = 0;
        const uint32_t n = millis();
        if ((n - last_ping_ms) >= 500) {   // ping 2 Hz instead of every loop
          last_ping_ms = n;
          if (rmw_uros_ping_agent(ROVER_UROS_PING_MS, 1) != RMW_RET_OK) {
            g_ping_miss++;
            g_diag_miss++;
          } else {
            g_ping_miss = 0;
          }
        }
      }
      if (g_ping_miss >= ROVER_UROS_PING_MISS) {
        g_diag_lost++;
        Serial.printf("[diag %lu] UROS_LOST #%lu after %u misses wifi=%d rssi=%d\n",
                      (unsigned long)millis(), (unsigned long)g_diag_lost,
                      (unsigned)g_ping_miss, (int)WiFi.status(), WiFi.RSSI());
        g_ping_miss = 0;
        destroy_entities();
        ros_state = RosState::kWaitAgent;
      } else {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case RosState::kDisconnect:
      destroy_entities();
      ros_state = RosState::kWaitAgent;
      break;
  }
}

void publish_sensors(uint32_t now) {
  if (ros_state != RosState::kConnected || !entities_ok) {
    return;
  }
  const int32_t sec = static_cast<int32_t>(now / 1000);
  const uint32_t nsec = static_cast<uint32_t>((now % 1000) * 1000000UL);

  if (g_side_l_ok) {
    fill_range(tof_left_msg, g_side_l_m, 1.3f);
    tof_left_msg.header.stamp.sec = sec;
    tof_left_msg.header.stamp.nanosec = nsec;
    (void)rcl_publish(&tof_left_pub, &tof_left_msg, NULL);
  }
  if (g_side_r_ok) {
    fill_range(tof_right_msg, g_side_r_m, 1.3f);
    tof_right_msg.header.stamp.sec = sec;
    tof_right_msg.header.stamp.nanosec = nsec;
    (void)rcl_publish(&tof_right_pub, &tof_right_msg, NULL);
  }

  if (g_l8_ok) {
    for (uint8_t i = 0; i < 8; i++) {
      tof_l8_cols_data[i] =
          (g_l8_cols[i] > 0.0f) ? g_l8_cols[i]
                                : std::numeric_limits<float>::quiet_NaN();
    }
    fill_range(tof_front_msg, g_l8_front, 4.0f);
    tof_front_msg.field_of_view = 0.80f;
    tof_front_msg.header.stamp.sec = sec;
    tof_front_msg.header.stamp.nanosec = nsec;
    fill_range(tof_front_left_msg, g_l8_front_l, 4.0f);
    tof_front_left_msg.field_of_view = 0.80f;
    tof_front_left_msg.header.stamp.sec = sec;
    tof_front_left_msg.header.stamp.nanosec = nsec;
    fill_range(tof_front_right_msg, g_l8_front_r, 4.0f);
    tof_front_right_msg.field_of_view = 0.80f;
    tof_front_right_msg.header.stamp.sec = sec;
    tof_front_right_msg.header.stamp.nanosec = nsec;
    (void)rcl_publish(&tof_front_pub, &tof_front_msg, NULL);
    (void)rcl_publish(&tof_front_left_pub, &tof_front_left_msg, NULL);
    (void)rcl_publish(&tof_front_right_pub, &tof_front_right_msg, NULL);
    (void)rcl_publish(&tof_l8_cols_pub, &tof_l8_cols_msg, NULL);
    memcpy(tof_l8_grid_data, g_l8_grid_mm, sizeof(tof_l8_grid_data));
    (void)rcl_publish(&tof_l8_grid_pub, &tof_l8_grid_msg, NULL);
  }

  if (g_sonar_ok) {
    fill_range(sonar_range_msg, g_sonar_m, 4.0f);
    sonar_range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    sonar_range_msg.field_of_view = 0.26f;
    sonar_range_msg.header.stamp.sec = sec;
    sonar_range_msg.header.stamp.nanosec = nsec;
    (void)rcl_publish(&sonar_range_pub, &sonar_range_msg, NULL);
  }

  tof_status_data[0] = g_sonar_m;
  tof_status_data[1] = g_servo_deg;
  tof_status_data[2] = g_pca_ok ? 1.0f : 0.0f;
  // [3] IR TX frames sent, [4] IR RX edges, [5] RMT ok
  tof_status_data[3] = static_cast<float>(g_ir_tx_frames);
  tof_status_data[4] = static_cast<float>(g_ir_rx_edges);
  tof_status_data[5] = g_ir_rmt_ok ? 1.0f : 0.0f;
  // [6] L8 frames read, [7] L8 poll errors, [8] valid zones (last frame), [9] no-target zones, [10] most common target_status (255 = none)
  tof_status_data[6] = static_cast<float>(g_l8_frames);
  tof_status_data[7] = static_cast<float>(g_l8_poll_err);
  tof_status_data[8] = static_cast<float>(g_l8_valid_z);
  tof_status_data[9] = static_cast<float>(g_l8_notgt_z);
  tof_status_data[10] = static_cast<float>(g_l8_top_status);
  (void)rcl_publish(&tof_status_pub, &tof_status_msg, NULL);

  ir_detected_msg.data = static_cast<int32_t>(g_ir_rx_edges);
  (void)rcl_publish(&ir_detected_pub, &ir_detected_msg, NULL);
}

}  // namespace

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(600);

  Serial.println();
  Serial.println("=== Pro S3 ToF micro-ROS ===");
  Serial.printf("host=%s agent=%s:%d\n", ROVER_HOSTNAME, ROVER_AGENT_IP,
                ROVER_AGENT_PORT);

  {
    const esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("[diag] RESET_REASON=%d %s\n", (int)rr, reset_reason_str(rr));
  }

  enable_ldo2();
  set_antenna_external(ROVER_PROS3_ANT_EXTERNAL != 0);
#if ROVER_WIFI_FIRST
  wifi_start();
#endif

  bus_l8.begin(L8_SDA, L8_SCL);
  bus_l8.setClock(L8_I2C_INIT_HZ);
  bus_l8.setTimeOut(50);
  delay(10);
  bus_side.begin(SIDE_SDA, SIDE_SCL);
  bus_side.setClock(SIDE_I2C_HZ);
  bus_side.setTimeOut(50);
  delay(10);

  scan_bus(bus_l8, "L8(8/9)");
  scan_bus(bus_side, "side(1/2)");

  begin_l8();
  begin_sides();
  begin_pca();

  g_sonar_ok = g_sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
  Serial.printf("Sonar TRIG=%d ECHO=%d %s\n", SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                g_sonar_ok ? "armed" : "fail");

  g_ir_rmt_ok = ROVER_IR_BEACON_ON && begin_ir_rmt();
  Serial.printf("IR TX GPIO%d RMT fleet beacon id %d: %s\n", IR_TX_PIN, FLEET_IR_ID,
                g_ir_rmt_ok ? "ON" : "FAILED/off");
  pinMode(IR_RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN), on_ir_rx_edge, FALLING);

#if !ROVER_WIFI_FIRST
  wifi_start();
#endif

#if defined(ENABLE_OTA)
  g_ota.beginConnected(ROVER_HOSTNAME);
#endif
}

void loop() {
  static uint32_t last_pub_ms = 0;
  static uint32_t last_sonar_ms = 0;
  static uint32_t last_hb_ms = 0;
  const uint32_t now = millis();

  poll_l8();
  poll_sides();

  if (g_sonar_ok && (now - last_sonar_ms) >= 200) {
    last_sonar_ms = now;
    g_sonar_m = g_sonar.read_range_m();
  }

  if (!uros_wifi_transport_ok && WiFi.status() == WL_CONNECTED) {
    set_microros_wifi_transports(ROVER_WIFI_SSID, ROVER_WIFI_PASS, g_agent_ip,
                                 ROVER_AGENT_PORT);
    uros_wifi_transport_ok = true;
    Serial.printf("WiFi late OK IP=%s -> agent %s:%d\n",
                  WiFi.localIP().toString().c_str(), ROVER_AGENT_IP, ROVER_AGENT_PORT);
  }

  if (uros_wifi_transport_ok) {
    handle_ros_state();
  }

  if (g_servo_dirty) {
    g_servo_dirty = false;
    pca_servo_write(g_servo_deg);
  }

  if ((now - last_pub_ms) >= 100) {
    last_pub_ms = now;
    publish_sensors(now);
  }

#if defined(ENABLE_OTA)
  g_ota.tick();
  if (g_ota.busy()) {
    while (g_ota.busy()) {
      g_ota.tick();
      yield();
    }
    return;
  }
#endif

  if ((now - last_hb_ms) >= 2000) {
    last_hb_ms = now;
    const char* st = (ros_state == RosState::kConnected) ? "up"
                     : (ros_state == RosState::kWaitAgent) ? "wait"
                                                           : "link";
    Serial.printf("hb [%s] l8=%s front=%.2f sideL=%.2f sideR=%.2f sonar=%.2f\n",
                  st, g_l8_ok ? "ok" : "--", g_l8_front, g_side_l_m, g_side_r_m,
                  g_sonar_m);
#if ROVER_WIFI_DIAG
    if (WiFi.status() == WL_CONNECTED) {
      const int r = WiFi.RSSI();
      if (r < g_rssi_min) g_rssi_min = r;
      if (r > g_rssi_max) g_rssi_max = r;
    }
    Serial.printf("[diag %lu] wifi=%d rssi=%d (min %d max %d) disc=%lu last_reason=%u "
                  "uros_up=%lu lost=%lu ping_miss=%lu heap=%u\n",
                  (unsigned long)now, (int)WiFi.status(), WiFi.RSSI(), g_rssi_min,
                  g_rssi_max, (unsigned long)g_diag_disc, (unsigned)g_diag_last_reason,
                  (unsigned long)g_diag_up, (unsigned long)g_diag_lost,
                  (unsigned long)g_diag_miss, (unsigned)ESP.getFreeHeap());
#endif
  }

  delay(5);
}
