/**
 * Unexpected Maker Pro S3 — sensor-board bring-up (USB serial DIAG).
 *
 * No motors, no explore, no T-Display OTA path.
 * Dual I2C: Wire=L8 (8/9), Wire1=side VL53 + PCA9685 (1/2).
 * Boots and logs empty I2C scans if sensors are not wired yet.
 *
 * Next step: um_pros3_tof_microros env publishing /rover/tof/* to the brain
 * under hostname rover-tof.local (T-Display keeps rover-esp.local).
 */

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <vl53l8cx.h>

#include "ir_tx.h"
#include "rover_pins_pros3.h"
#include "ultrasonic.h"

#ifndef ROVER_WIFI_SSID
#define ROVER_WIFI_SSID ""
#endif
#ifndef ROVER_WIFI_PASS
#define ROVER_WIFI_PASS ""
#endif
#ifndef ROVER_TELEM_HOST
#define ROVER_TELEM_HOST "192.168.3.172"
#endif
#ifndef ROVER_TELEM_PORT
#define ROVER_TELEM_PORT 4243
#endif

namespace {

TwoWire& bus_l8 = Wire;
TwoWire bus_side = TwoWire(1);

VL53L8CX g_l8(&bus_l8, L8_LPN_PIN);
VL53L8CX_ResultsData g_l8_results;
bool g_l8_ok = false;
float g_l8_cols[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
float g_l8_front = -1.0f;

VL53L1X g_side_l;
VL53L1X g_side_r;
bool g_side_l_ok = false;
bool g_side_r_ok = false;
float g_side_l_m = -1.0f;
float g_side_r_m = -1.0f;

bool g_pca_ok = false;
Ultrasonic g_sonar;
bool g_sonar_ok = false;
float g_sonar_m = -1.0f;

IrTx g_ir_tx;
bool g_wifi_ok = false;
WiFiUDP g_udp;

uint32_t g_boot_ms = 0;

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

void diag_line(const char* line) {
  Serial.println(line);
  if (!g_wifi_ok || WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (!g_udp.beginPacket(ROVER_TELEM_HOST, ROVER_TELEM_PORT)) {
    return;
  }
  g_udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
  g_udp.endPacket();
}

void diag_printf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  diag_line(buf);
}

void enable_ldo2() {
  pinMode(PIN_PROS3_LDO2, OUTPUT);
  digitalWrite(PIN_PROS3_LDO2, HIGH);
  delay(50);
  Serial.printf("LDO2 GPIO%d HIGH (external 3V3 on)\n", PIN_PROS3_LDO2);
}

void set_xshut(bool left_on, bool right_on) {
  digitalWrite(VL53_XSHUT_L, left_on ? HIGH : LOW);
  digitalWrite(VL53_XSHUT_R, right_on ? HIGH : LOW);
}

bool setup_side_one(VL53L1X& sensor, uint8_t addr) {
  sensor.setBus(&bus_side);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    return false;
  }
  sensor.setAddress(addr);
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(50);
  delay(60);
  return true;
}

bool begin_sides() {
  pinMode(VL53_XSHUT_L, OUTPUT);
  pinMode(VL53_XSHUT_R, OUTPUT);
  set_xshut(false, false);
  delay(50);

  Serial.printf("Side I2C pre-init: 0x29=%s\n",
                i2c_present(bus_side, 0x29) ? "yes" : "no");

  set_xshut(true, false);
  delay(50);
  g_side_l_ok = setup_side_one(g_side_l, VL53L_ADDR_L);
  if (!g_side_l_ok) {
    set_xshut(false, false);
    delay(20);
  }

  set_xshut(g_side_l_ok, true);
  delay(50);
  g_side_r_ok = setup_side_one(g_side_r, VL53L_ADDR_R);
  if (!g_side_r_ok) {
    set_xshut(g_side_l_ok, false);
    delay(10);
  } else {
    set_xshut(g_side_l_ok, true);
  }

  Serial.printf("Side VL53L1X L=%s (0x%02X XSHUT=%d) R=%s (0x%02X XSHUT=%d)\n",
                g_side_l_ok ? "OK" : "--", VL53L_ADDR_L, VL53_XSHUT_L,
                g_side_r_ok ? "OK" : "--", VL53L_ADDR_R, VL53_XSHUT_R);
  return g_side_l_ok || g_side_r_ok;
}

bool begin_l8() {
  g_l8_ok = false;
  for (int i = 0; i < 8; i++) {
    g_l8_cols[i] = -1.0f;
  }
  g_l8_front = -1.0f;

  bus_l8.setClock(L8_I2C_INIT_HZ);
  bus_l8.setTimeOut(400);

  // Library drives LPn via on()/off(); wake explicitly.
  g_l8.begin();
  g_l8.on();
  delay(20);

  uint8_t alive = 0;
  if (g_l8.is_alive(&alive) != 0 || alive == 0) {
    Serial.println("VL53L8CX not alive at 0x29 (not wired or LPn/SPI_I2C_N?)");
    bus_l8.setClock(L8_I2C_HZ);
    bus_l8.setTimeOut(50);
    return false;
  }

  Serial.println("VL53L8CX init (firmware load, a few seconds)...");
  const uint8_t st = g_l8.init();
  if (st != 0) {
    Serial.printf("VL53L8CX init failed status=%u\n", (unsigned)st);
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
  Serial.printf("VL53L8CX 8x8 ranging OK (bus %d/%d @ %u Hz, LPn=%d)\n", L8_SDA, L8_SCL,
                (unsigned)L8_I2C_HZ, L8_LPN_PIN);
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
    return;
  }

  // 8x8: rows 0–3 forward cone; skip lower floor-heavy rows for "front".
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

  float front = -1.0f;
  for (uint8_t col = 2; col <= 5; col++) {
    if (cols[col] > 0.02f) {
      front = (front < 0.0f) ? cols[col] : fminf(front, cols[col]);
    }
  }

  for (uint8_t i = 0; i < 8; i++) {
    g_l8_cols[i] = cols[i];
  }
  g_l8_front = front;
}

void poll_sides() {
  if (g_side_l_ok) {
    const uint16_t mm = g_side_l.read(false);
    g_side_l_m = (mm == 0) ? -1.0f : static_cast<float>(mm) * 0.001f;
  }
  if (g_side_r_ok) {
    const uint16_t mm = g_side_r.read(false);
    g_side_r_m = (mm == 0) ? -1.0f : static_cast<float>(mm) * 0.001f;
  }
}

bool begin_pca() {
  bus_side.beginTransmission(PCA9685_ADDR);
  if (bus_side.endTransmission() != 0) {
    Serial.println("PCA9685 not on side bus (0x40)");
    return false;
  }
  // Soft wake / 50 Hz prescale (same as RoverPca9685::begin) — presence only for bring-up.
  auto write_reg = [](uint8_t reg, uint8_t val) {
    bus_side.beginTransmission(PCA9685_ADDR);
    bus_side.write(reg);
    bus_side.write(val);
    return bus_side.endTransmission() == 0;
  };
  write_reg(0x00, 0x00);
  write_reg(0x01, 0x04);
  const uint8_t prescale = static_cast<uint8_t>(25000000UL / (4096UL * 50UL) - 1);
  write_reg(0x00, 0x10);
  write_reg(0xFE, prescale);
  write_reg(0x00, 0x00);
  delay(5);
  write_reg(0x00, 0xA0);
  Serial.printf("PCA9685 OK @ 0x%02X (servo V+ needs separate 5V)\n", PCA9685_ADDR);
  return true;
}

bool begin_wifi() {
  if (ROVER_WIFI_SSID[0] == '\0') {
    Serial.println("WiFi SSID empty — serial-only bring-up");
    return false;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ROVER_HOSTNAME);
  WiFi.begin(ROVER_WIFI_SSID, ROVER_WIFI_PASS);
  Serial.printf("WiFi connecting to %s as %s...", ROVER_WIFI_SSID, ROVER_HOSTNAME);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed — continuing serial-only");
    return false;
  }
  Serial.printf("WiFi OK %s  http://%s.local\n", WiFi.localIP().toString().c_str(),
                ROVER_HOSTNAME);
  if (MDNS.begin(ROVER_HOSTNAME)) {
    Serial.printf("mDNS %s.local ready\n", ROVER_HOSTNAME);
  }
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(800);
  g_boot_ms = millis();

  Serial.println();
  Serial.println("=== Pro S3 ToF bring-up ===");
  Serial.println("board=um_pros3  role=sensor  motors=off  explore=off");
  Serial.printf("pins L8 SDA/SCL/LPn=%d/%d/%d  side SDA/SCL/XSHUT=%d/%d/%d+%d\n", L8_SDA,
                L8_SCL, L8_LPN_PIN, SIDE_SDA, SIDE_SCL, VL53_XSHUT_L, VL53_XSHUT_R);
  Serial.printf("sonar TRIG/ECHO=%d/%d  IR RX/TX=%d/%d\n", SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                IR_RX_PIN, IR_TX_PIN);

  enable_ldo2();

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
  g_pca_ok = begin_pca();

  g_sonar_ok = g_sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
  Serial.printf("Sonar TRIG=%d ECHO=%d %s\n", SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                g_sonar_ok ? "armed" : "fail");

  if (g_ir_tx.begin(IR_TX_PIN, IR_TX_HZ, IR_TX_LEDC_CHANNEL)) {
    g_ir_tx.set_enabled(false);  // quiet until chase; pin reserved
    Serial.printf("IR TX GPIO%d armed (off)\n", IR_TX_PIN);
  } else {
    Serial.printf("IR TX GPIO%d failed\n", IR_TX_PIN);
  }
  pinMode(IR_RX_PIN, INPUT);
  Serial.printf("IR RX GPIO%d input\n", IR_RX_PIN);

  g_wifi_ok = begin_wifi();
  diag_printf("EVENT pros3_tof boot l8=%d sideL=%d sideR=%d pca=%d sonar=%d wifi=%d",
              g_l8_ok ? 1 : 0, g_side_l_ok ? 1 : 0, g_side_r_ok ? 1 : 0, g_pca_ok ? 1 : 0,
              g_sonar_ok ? 1 : 0, g_wifi_ok ? 1 : 0);
  Serial.println("loop: poll ranges every 500 ms (empty sensors stay --)");
}

void loop() {
  static uint32_t last_log_ms = 0;
  static uint32_t last_sonar_ms = 0;
  const uint32_t now = millis();

  poll_l8();
  poll_sides();

  if (g_sonar_ok && (now - last_sonar_ms) >= 200) {
    last_sonar_ms = now;
    g_sonar_m = g_sonar.read_range_m();
  }

  if ((now - last_log_ms) >= 500) {
    last_log_ms = now;
    char cols[96];
    snprintf(cols, sizeof(cols), "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
             g_l8_cols[0], g_l8_cols[1], g_l8_cols[2], g_l8_cols[3], g_l8_cols[4],
             g_l8_cols[5], g_l8_cols[6], g_l8_cols[7]);
    diag_printf(
        "DIAG pros3 t=%lu l8=%s front=%.2f cols=[%s] sideL=%.2f sideR=%.2f sonar=%.2f "
        "rx=%d",
        static_cast<unsigned long>(now - g_boot_ms), g_l8_ok ? "ok" : "--",
        g_l8_front, cols, g_side_l_m, g_side_r_m, g_sonar_m,
        digitalRead(IR_RX_PIN));
  }

  delay(20);
}
