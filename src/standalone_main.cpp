/**
 * Standalone rover firmware — ESP is the brain. Pi UART = speaker + LED ring only.
 * See ../lego-rover-ros2/PI_PERIPHERAL.md
 */
#include <Arduino.h>

#include "drive.h"
#include "heading_hold.h"
#include "imu_axes.h"
#include "motion_detect.h"
#include "mpu.h"
#include "rover_board_power.h"
#include "rover_go_button.h"
#include "rover_pi_periph.h"
#include "rover_wander.h"
#include "status_led.h"

#include "battery.h"
#include "estop.h"
#include "rover_button_events.h"
#include "rover_display.h"
#include "rover_ota.h"
#include "rover_pins_s3.h"
#include "rover_mcp_ir.h"
#include "rover_pca9685.h"
#include "ir_tx.h"
#include "rover_sonar.h"
#include "rover_telem.h"
#include "ultrasonic.h"

#if defined(ENABLE_OTA)
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

#ifndef PI_UART_PORT
#define PI_UART_PORT 1
#endif
#ifndef PI_PERIPH_BAUD
#define PI_PERIPH_BAUD 115200
#endif

static HardwareSerial PiSerial(PI_UART_PORT);

static SmoothDrive drv;
static Mpu6050 mpu;
static HeadingHold heading;
static MotionDetect motion;
static StatusLed status_led;
static BatteryMonitor battery;
static EStop estop;
static GoButton go_button;
static RoverDisplay display;
static RoverOta ota;
static RoverUiData ui;
static RoverPiPeriph pi_periph;
static RoverWander wander;
static RoverMcpIr mcp_ir;
static RoverPca9685 pca9685;
static IrTx ir_beacon;
static Ultrasonic sonar;
static RoverSonar rover_sonar;

#ifndef FRONT_IR_WALL_MS
#define FRONT_IR_WALL_MS 3000
#endif

static uint32_t stall_block_until = 0;
static uint32_t front_ir_wall_since = 0;
static uint32_t front_ir_wall_cooldown_until = 0;
static bool local_session = false;
static float last_lin = 0.0f;
static float last_ang = 0.0f;
static uint32_t last_drive_ms = 0;
static bool sonar_drive_override = false;
static float sonar_drive_lin = 0.0f;
static float sonar_drive_ang = 0.0f;
static bool mode_menu_open = false;
static I2cScanResult ir_i2c_scan;
static uint32_t last_ir_retry_ms = 0;
static bool ir_wheels_on = false;

#ifndef ROVER_RING_COUNT
#define ROVER_RING_COUNT 8
#endif
#ifndef ROVER_SONAR_PAN_CENTER
#define ROVER_SONAR_PAN_CENTER 90.0f
#endif
#ifndef ROVER_RING_ZERO_BEARING
#define ROVER_RING_ZERO_BEARING 270.0f
#endif

static uint8_t g_sonar_bins[ROVER_RING_COUNT];
static uint32_t g_sonar_hold_until_ms = 0;
static bool g_sonar_scanning = false;

static uint8_t sonar_m_to_cm(float m) {
  if (m <= 0.0f) {
    return 255;
  }
  int cm = static_cast<int>(m * 100.0f);
  if (cm < 0) {
    cm = 0;
  }
  if (cm > 254) {
    cm = 254;
  }
  return static_cast<uint8_t>(cm);
}

static int sonar_pan_to_led(float pan_deg) {
  const float bearing = fmodf(ROVER_SONAR_PAN_CENTER - pan_deg + 360.0f, 360.0f);
  const float step = 360.0f / static_cast<float>(ROVER_RING_COUNT);
  const float delta = fmodf(bearing - ROVER_RING_ZERO_BEARING + 360.0f, 360.0f);
  int led = static_cast<int>(lroundf(delta / step)) % ROVER_RING_COUNT;
  if (led < 0) {
    led += ROVER_RING_COUNT;
  }
  return led;
}

static void reset_sonar_ring() {
  for (int i = 0; i < ROVER_RING_COUNT; i++) {
    g_sonar_bins[i] = 255;
  }
  g_sonar_hold_until_ms = 0;
  g_sonar_scanning = false;
}

static void tick_sonar_ring(uint32_t now_ms) {
  static uint32_t last_ms = 0;
  if (now_ms - last_ms < 50) {
    return;
  }
  last_ms = now_ms;

  const float pan = rover_sonar.pan_deg();
  const float rng = rover_sonar.range_m();
  const int sweep = sonar_pan_to_led(pan);

  if (rng > 0.0f) {
    g_sonar_bins[sweep] = sonar_m_to_cm(rng);
    g_sonar_hold_until_ms = now_ms + 4000;
  }

  const bool off_center = fabsf(pan - ROVER_SONAR_PAN_CENTER) > 10.0f;
  if (off_center) {
    g_sonar_scanning = true;
    g_sonar_hold_until_ms = now_ms + 4000;
  } else if (g_sonar_scanning && now_ms < g_sonar_hold_until_ms) {
    // Keep scan bins while pan returns to centre.
  } else {
    g_sonar_scanning = false;
  }

  if (!g_sonar_scanning) {
    for (int i = 0; i < ROVER_RING_COUNT; i++) {
      if (g_sonar_bins[i] < 255) {
        g_sonar_bins[i]++;
      }
    }
  }

  pi_periph.sonar_frame(g_sonar_bins, ROVER_RING_COUNT, static_cast<uint8_t>(sweep));

  uint8_t esc_id = 0;
  if (rover_sonar.take_escape_event(&esc_id)) {
    pi_periph.notify_escape();
  }
}

#if defined(ENABLE_OTA)
static WiFiUDP g_telem_udp;
static WiFiUDP g_cmd_udp;
static bool g_cmd_udp_bound = false;
static IPAddress g_telem_host;
static bool g_telem_host_ok = false;

static void telem_udp_init() {
  if (g_telem_host_ok) {
    return;
  }
  g_telem_host_ok = g_telem_host.fromString(ROVER_TELEM_HOST);
  if (!g_telem_host_ok) {
    Serial.printf("TELEM: bad ROVER_TELEM_HOST %s\n", ROVER_TELEM_HOST);
  }
}

static void telem_udp_send(const char* line) {
  if (WiFi.status() != WL_CONNECTED || line == nullptr) {
    return;
  }
  telem_udp_init();
  if (!g_telem_host_ok) {
    return;
  }
  g_telem_udp.beginPacket(g_telem_host, ROVER_TELEM_PORT);
  g_telem_udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
  g_telem_udp.endPacket();
}

static void telem_emit_line(const char* line) {
  if (line == nullptr) {
    return;
  }
  telem_udp_send(line);
}

static void telem_event(uint32_t now, const char* tag, const char* detail) {
  char buf[200];
  snprintf(buf, sizeof(buf), "EVT t=%lu tag=%s %s\n", (unsigned long)now, tag,
           detail != nullptr ? detail : "");
  telem_emit_line(buf);
}

static void bind_cmd_udp() {
  if (!g_cmd_udp_bound && WiFi.status() == WL_CONNECTED) {
    g_cmd_udp.begin(4244);
    g_cmd_udp_bound = true;
  }
}
#endif

static bool drive_blocked() {
  if (estop.active()) return true;
  if (battery.critical()) return true;
  if (millis() < stall_block_until) return true;
  return false;
}

static void force_stop(const char* reason) {
  last_lin = 0.0f;
  last_ang = 0.0f;
  heading.disarm();
  sonar_drive_override = false;
  sonar_drive_lin = 0.0f;
  sonar_drive_ang = 0.0f;
  if (reason != nullptr) {
    rover_sonar.abort_escape(millis());
  }
  drv.stop();
  if (reason) {
    Serial.printf("DRIVE STOP: %s\n", reason);
#if defined(ENABLE_OTA)
    telem_event(millis(), "STOP", reason);
#endif
  }
}

static void apply_drive(float yaw_rate, float dt) {
  if (drive_blocked()) {
    force_stop(nullptr);
    return;
  }

  constexpr uint32_t CMD_TIMEOUT_MS = 400;
  const bool cmd_fresh = (millis() - last_drive_ms) <= CMD_TIMEOUT_MS;
  if (!local_session && !sonar_drive_override && !cmd_fresh) {
    force_stop(nullptr);
    return;
  }

  float v = sonar_drive_lin;
  float turn = sonar_drive_ang;
  float trim = 0.0f;

  // Standalone explore: wander + sonar steer — no heading lock (was causing PID wobble).
  heading.disarm();

  float l, r;
  constexpr float STEER_DEAD = 0.02f;
  constexpr float LIN_SPIN_MAX = 0.08f;
  const bool spin_mode = fabsf(turn) > STEER_DEAD && fabsf(v) < LIN_SPIN_MAX;

  if (spin_mode) {
    trim = 0.0f;
    v = 0.0f;
    l = -turn;
    r = turn;
  } else if (fabsf(turn) < STEER_DEAD) {
    l = v + trim;
    r = v - trim;
  } else {
    constexpr float MIN_WHEEL = 0.06f;
    if (fabsf(v) > MIN_WHEEL) {
      const float max_turn = fabsf(v) - MIN_WHEEL;
      if (fabsf(turn) > max_turn) {
        turn = (turn > 0.0f) ? max_turn : -max_turn;
      }
    }
    l = v - turn;
    r = v + turn;
    trim = 0.0f;
  }

  const float peak = fmaxf(fabsf(l), fabsf(r));
  if (peak > 1.0f) {
    l /= peak;
    r /= peak;
  }
  drv.set_target(constrain(l, -1.0f, 1.0f), constrain(r, -1.0f, 1.0f));
}

static void calibrate_gyro() {
  if (!mpu.ok()) return;
  Serial.println("MPU gyro cal — hold rover still...");
  heading.begin();
  uint32_t prev_ms = millis();
  const uint32_t t0 = prev_ms;
  while (millis() - t0 < 2800) {
    float ax, ay, az, gx, gy, gz;
    const uint32_t now = millis();
    float dt = (now - prev_ms) / 1000.0f;
    prev_ms = now;
    if (dt < 0.001f) {
      dt = 0.001f;
    }
    if (mpu.read(ax, ay, az, gx, gy, gz)) {
      heading.update(imu_yaw_rate(gx, gy, gz), dt, true);
    }
    delay(5);
    yield();
  }
}

#if ROVER_BOOT_SPIN_TRICK
/** Non-blocking boot spin — runs from loop() so setup/WiFi/serial stay alive. */
static bool tick_boot_spin_trick(uint32_t now) {
  if (!mpu.ok()) {
    return false;
  }

  enum class Phase : uint8_t { kIdle, kLeft, kPause, kRight, kDone };
  static Phase phase = Phase::kIdle;
  static uint32_t spin_t0 = 0;
  static uint32_t pause_until = 0;
  static uint32_t prev_ms = 0;
  static float spin_start_yaw = 0.0f;

  if (phase == Phase::kDone) {
    return false;
  }

  if (phase == Phase::kIdle) {
    phase = Phase::kLeft;
    heading.reset_angle();
    spin_start_yaw = heading.angle_deg();
    spin_t0 = now;
    prev_ms = now;
    Serial.println("BOOT: left turn");
  }

  if (phase == Phase::kPause) {
    drv.stop();
    if (now < pause_until) {
      return true;
    }
    phase = Phase::kRight;
    heading.reset_angle();
    spin_start_yaw = heading.angle_deg();
    spin_t0 = now;
    prev_ms = now;
    Serial.println("BOOT: right turn");
  }

  if (phase != Phase::kLeft && phase != Phase::kRight) {
    return true;
  }

  float dt = (now - prev_ms) / 1000.0f;
  prev_ms = now;
  if (dt < 0.002f) {
    dt = 0.002f;
  }
  if (dt > 0.04f) {
    dt = 0.04f;
  }

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (mpu.read(ax, ay, az, gx, gy, gz)) {
    heading.update(imu_yaw_rate(gx, gy, gz), dt, false);
  }

  constexpr uint32_t kSpinTimeoutMs = 12000;
  constexpr float kTargetDeg = 360.0f;
  constexpr float kYawTolDeg = 4.0f;
  constexpr float kSpin = ROVER_BOOT_SPIN_LIN;
  const float yaw_delta = fabsf(heading.angle_deg() - spin_start_yaw);
  const bool timed_out = (now - spin_t0) >= kSpinTimeoutMs;
  const bool reached = yaw_delta >= (kTargetDeg - kYawTolDeg);

  if (phase == Phase::kLeft) {
    if (!reached && !timed_out) {
      drv.set_target(-kSpin, kSpin);
      drv.tick();
      return true;
    }
    drv.stop();
    Serial.printf("BOOT: left done yaw=%.1f\n", heading.angle_deg());
    phase = Phase::kPause;
    pause_until = now + 350;
    return true;
  }

  if (phase == Phase::kRight) {
    if (!reached && !timed_out) {
      drv.set_target(kSpin, -kSpin);
      drv.tick();
      return true;
    }
    drv.stop();
    Serial.printf("BOOT: right done yaw=%.1f\n", heading.angle_deg());
    heading.reset_angle();
    phase = Phase::kDone;
    return false;
  }

  return false;
}
#endif

static void tick_reverse_rear_ir(uint32_t now) {
  static uint32_t last_check_ms = 0;
  if (sonar_drive_lin > -0.04f) {
    last_check_ms = 0;
    return;
  }
  if (!mcp_ir.body_ok()) {
    return;
  }
  if (last_check_ms != 0 && (now - last_check_ms) < 130) {
    return;
  }
  last_check_ms = now;
  if (mcp_ir.rear_obstacle()) {
    Serial.println("EVENT rear IR — reverse blocked");
    ui.bump = true;
    pi_periph.notify_bump();
    rover_sonar.abort_escape(now);
    sonar_drive_lin = 0.0f;
    sonar_drive_ang = 0.0f;
    sonar_drive_override = false;
    drv.stop();
  }
}

static void toggle_session(uint32_t now) {
  local_session = !local_session;
  if (local_session) {
    wander.reset();
    mcp_ir.reset_wheel_ticks();
    reset_sonar_ring();
    pi_periph.notify_session(true);
    pi_periph.play(RoverPiPeriph::kMelodyReady);
    Serial.println("SESSION start (ESP wander)");
  } else {
    force_stop("session stop");
    reset_sonar_ring();
    front_ir_wall_since = 0;
    front_ir_wall_cooldown_until = 0;
    pi_periph.notify_session(false);
    Serial.println("SESSION stop");
  }
  (void)now;
}

#if defined(ENABLE_OTA)
static void poll_udp_cmd(uint32_t now) {
  bind_cmd_udp();
  const int pkt = g_cmd_udp.parsePacket();
  if (pkt <= 0) {
    return;
  }
  char buf[20];
  const int n = g_cmd_udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) {
    return;
  }
  buf[n] = '\0';
  if (strcmp(buf, "!go") == 0) {
    if (!local_session) {
      toggle_session(now);
    }
  } else if (strcmp(buf, "!stop") == 0) {
    if (local_session) {
      toggle_session(now);
    } else {
      force_stop("udp stop");
    }
  }
}
#endif

static void handle_go_button(RoverButtonEvent ev, uint32_t now) {
  if (ev == kBtnGoShort) {
    toggle_session(now);
    pi_periph.play(RoverPiPeriph::kMelodyButton);
  } else if (ev == kBtnGoLong) {
    mode_menu_open = !mode_menu_open;
    pi_periph.play(RoverPiPeriph::kMelodyMenu);
  }
}

static void poll_serial_cmd(uint32_t now) {
  static char line[24];
  static uint8_t n = 0;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        line[n] = '\0';
        if (strcmp(line, "!go") == 0) {
          if (!local_session) {
            toggle_session(now);
          }
        } else if (strcmp(line, "!stop") == 0) {
          if (local_session) {
            toggle_session(now);
          } else {
            force_stop("serial stop");
          }
        }
        n = 0;
      }
      continue;
    }
    if (n + 1 < sizeof(line)) {
      line[n++] = c;
    }
  }
}

void setup() {
  rover_board_power_on();
  SmoothDrive::gpio_safe_idle(DIR_L, PWM_L, DIR_R, PWM_R);

  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);

  PiSerial.begin(PI_PERIPH_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
  pi_periph.begin(PiSerial, PI_PERIPH_BAUD);
  status_led.begin(STATUS_LED_PIN);
  heading.begin();
  motion.begin();
  wander.begin();

  estop.begin(PIN_BUTTON_ESTOP);
  go_button.begin(PIN_BUTTON_GO);
  battery.begin(PIN_BAT_ADC, BAT_ADC_SCALE, BAT_VOLT_LOW, BAT_VOLT_CRITICAL);
  mcp_ir.begin(MPU_SDA, MPU_SCL);
  if (pca9685.begin(PCA9685_ADDR)) {
    pca9685.setAngle(ROVER_AUX_SERVO_CHANNEL, 90.0f, 0.0f, 180.0f, 500, 2500);
  }
  if (sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN)) {
    rover_sonar.begin(&sonar, pca9685.ok() ? &pca9685 : nullptr);
    if (mcp_ir.body_ok()) {
      rover_sonar.set_aux_scan(&mcp_ir);
    }
  }
  ir_beacon.begin(IR_TX_PIN, IR_TX_HZ, IR_TX_LEDC_CHANNEL);
  ir_beacon.set_enabled(IR_TX_DEFAULT_ON);

  drv.begin(DIR_L, PWM_L, DIR_R, PWM_R);
  display.begin();
  ota.begin("rover-esp");
  ui.mode = "standby";
#if defined(ENABLE_OTA)
  rover_telem_set_sink(telem_emit_line);
#endif

#if ROVER_BOOT_PAN_SWEEP
  if (pca9685.ok()) {
    rover_sonar.boot_full_sweep();
  }
#endif
  if (mpu.begin(MPU_SDA, MPU_SCL)) {
    Serial.println("MPU6050 OK");
    calibrate_gyro();
  } else {
    Serial.println("MPU6050 not found — no heading / stall IMU path");
  }

  pi_periph.play(RoverPiPeriph::kMelodyReady);
  pi_periph.ring_mode(RoverPiPeriph::kRingStandby);
  Serial.printf("Rover standalone — tap Go to explore. Pi link @%u on UART\n",
                (unsigned)PI_PERIPH_BAUD);
}

void loop() {
  const uint32_t now = millis();
  static uint32_t prev = now;
  const float dt = (now - prev) / 1000.0f;
  prev = now;

  estop.poll(now);
  go_button.poll(now);
  battery.update(now);
  ir_beacon.tick();
  ota.tick();
  pi_periph.tick(now);

#if ROVER_BOOT_SPIN_TRICK
  if (tick_boot_spin_trick(now)) {
    if (estop.active() || battery.critical()) {
      force_stop(estop.active() ? "E-STOP" : "low battery");
    }
    ui.mode = "BOOT spin";
    display.draw(ui, now);
    return;
  }
#endif

  if (estop.active() || battery.critical()) {
    force_stop(estop.active() ? "E-STOP" : "low battery");
  }

  handle_go_button(go_button.take_event(), now);
  poll_serial_cmd(now);
#if defined(ENABLE_OTA)
  poll_udp_cmd(now);
#endif

  if (local_session) {
    if (!drive_blocked()) {
      float w_lin = 0.0f;
      float w_ang = 0.0f;
      if (wander.tick(now, &w_lin, &w_ang)) {
        last_lin = w_lin;
        last_ang = w_ang;
      }
    }
    last_drive_ms = now;
  }

  const bool want_wheels = local_session || (now - last_drive_ms < 1000);
  if (want_wheels != ir_wheels_on) {
    ir_wheels_on = want_wheels;
    mcp_ir.set_wheels_enabled(want_wheels);
  }
  mcp_ir.tick(now);

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  const bool imu_ok = mpu.ok() && mpu.read(ax, ay, az, gx, gy, gz);
  const float yaw_rate = imu_ok ? imu_yaw_rate(gx, gy, gz) : 0.0f;
  const bool imu_still =
      (now - last_drive_ms >= 400) && fabsf(drv.cur_left()) <= 0.04f &&
      fabsf(drv.cur_right()) <= 0.04f && fabsf(drv.tgt_left()) <= 0.04f &&
      fabsf(drv.tgt_right()) <= 0.04f;
  if (imu_ok) {
    heading.update(yaw_rate, dt, imu_still);
  }
  const float yaw_deg = imu_ok ? heading.angle_deg_wrapped() : 0.0f;

  {
    const bool cmd_fresh = (now - last_drive_ms) <= 400;
    float s_lin = cmd_fresh ? last_lin : 0.0f;
    float s_ang = cmd_fresh ? last_ang : 0.0f;
    const bool body_moving =
        fabsf(drv.cur_left()) > 0.04f || fabsf(drv.cur_right()) > 0.04f;
    sonar_drive_override =
        rover_sonar.tick(now, s_lin, s_ang, body_moving, yaw_deg, &s_lin, &s_ang);
    sonar_drive_lin = s_lin;
    sonar_drive_ang = s_ang;
  }

  if (local_session) {
    tick_reverse_rear_ir(now);
    tick_sonar_ring(now);

    if (mcp_ir.front_ok() && now >= front_ir_wall_cooldown_until) {
      const float motor_l =
          fmaxf(fabsf(drv.tgt_left()), fabsf(drv.cur_left()));
      const float motor_r =
          fmaxf(fabsf(drv.tgt_right()), fabsf(drv.cur_right()));
      const bool driving_fwd =
          fmaxf(motor_l, motor_r) > 0.10f && drv.tgt_left() * drv.tgt_right() >= 0.0f;
      if (mcp_ir.front_hit() && driving_fwd) {
        if (front_ir_wall_since == 0) {
          front_ir_wall_since = now;
        } else if (now - front_ir_wall_since >= FRONT_IR_WALL_MS) {
          Serial.println("EVENT front IR wall — sustained");
          ui.bump = true;
          pi_periph.notify_bump();
          if (!rover_sonar.avoid_active()) {
            rover_sonar.trigger_escape(now);
          }
          front_ir_wall_since = 0;
          front_ir_wall_cooldown_until = now + 6000;
        }
      } else {
        front_ir_wall_since = 0;
      }
    } else if (!mcp_ir.front_hit()) {
      front_ir_wall_since = 0;
    }
  }

  apply_drive(yaw_rate, dt);
  drv.tick();

  if (local_session || sonar_drive_override) {
    static uint32_t last_telem_ms = 0;
    if (now - last_telem_ms >= 100) {
      last_telem_ms = now;
      const float rng = rover_sonar.range_m();
      char telem_line[320];
      snprintf(
          telem_line, sizeof(telem_line),
          "TELEM t=%lu slin=%.3f sang=%.3f wlin=%.3f wang=%.3f tL=%.2f tR=%.2f L=%.2f R=%.2f "
          "pan=%.0f rng=%.2f gl=%.2f gr=%.2f yaw=%.1f ovr=%d ph=%d gs=%u blk=%d wtl=%lu wtr=%lu "
          "sess=%d\n",
          (unsigned long)now, sonar_drive_lin, sonar_drive_ang, wander.cmd_lin(), wander.cmd_ang(),
          drv.tgt_left(), drv.tgt_right(), drv.cur_left(), drv.cur_right(), rover_sonar.pan_deg(),
          (rng > 0.0f) ? rng : -1.0f, rover_sonar.glance_left_m(), rover_sonar.glance_right_m(),
          heading.angle_deg(), sonar_drive_override ? 1 : 0, rover_sonar.phase_code(),
          (unsigned)rover_sonar.glance_state(), drive_blocked() ? 1 : 0,
          (unsigned long)mcp_ir.wheel_left_ticks(), (unsigned long)mcp_ir.wheel_right_ticks(),
          local_session ? 1 : 0);
      rover_telem_printf("%s", telem_line);
    }
  }

  if (imu_ok && (local_session || sonar_drive_override)) {
    const float motor_l = fmaxf(fabsf(drv.tgt_left()), fabsf(drv.cur_left()));
    const float motor_r = fmaxf(fabsf(drv.tgt_right()), fabsf(drv.cur_right()));
    motion.update(ax, ay, az, gx, gy, gz, motor_l, motor_r,
                  mcp_ir.wheel_left_ticks(), mcp_ir.wheel_right_ticks(), now);
    if (motion.stall()) {
      Serial.println("EVENT stall — motors cut locally");
#if defined(ENABLE_OTA)
      telem_event(now, "STALL", "motors_cut");
#endif
      ui.stall = true;
      force_stop("stall");
      stall_block_until = now + 2500;
      pi_periph.notify_stall();
    }
    motion.clear_events();
  }

  ui.bat_v = battery.voltage();
  ui.bat_low = battery.low();
  ui.bat_critical = battery.critical();
  ui.session_active = local_session;
  ui.pi_bridge_live = false;
  ui.estop = estop.active();
  ui.mode_menu = mode_menu_open;
  ui.cmd_lin = last_lin;
  ui.cmd_ang = last_ang;
  ui.motor_l = drv.cur_left();
  ui.motor_r = drv.cur_right();
  ui.mode = estop.active()      ? "E-STOP"
            : battery.critical() ? "low batt"
            : local_session      ? "Explore"
                                 : "Standby";
  display.draw(ui, now);
  ui.bump = false;
  ui.stall = false;

  delay(local_session ? 8 : 20);
}
