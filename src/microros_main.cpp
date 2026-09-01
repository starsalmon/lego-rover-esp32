#include <Arduino.h>
#include <esp_system.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/ping.h>
#include <geometry_msgs/msg/twist.h>
#include <limits>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int32.h>

#include "drive.h"
#include "heading_hold.h"
#include "imu_axes.h"
#include "motion_detect.h"
#include "mpu.h"
#include "rover_board_power.h"
#include "status_led.h"
#include "rover_diag.h"

#ifdef ROVER_TDISPLAY_S3
#include "battery.h"
#include "estop.h"
#include "rover_button_events.h"
#include "rover_go_button.h"
#include "rover_display.h"
#include "rover_ota.h"
#include "rover_pins_s3.h"
#if defined(ROVER_MCP_IR)
#include "rover_mcp_ir.h"
#endif
#include "rover_pca9685.h"
#include "ir_tx.h"
#if defined(ROVER_SONAR)
#include "rover_sonar.h"
#include "ultrasonic.h"
#endif
#if defined(ROVER_VL53L)
#include "rover_vl53.h"
#endif
#if defined(ROVER_PERIPH)
#include "rover_periph.h"
#endif
#endif

#ifndef DIR_L
#define DIR_L 0
#define PWM_L 1
#define DIR_R 3
#define PWM_R 2
#define MPU_SDA 8
#define MPU_SCL 9
#define PI_UART_RX 20
#define PI_UART_TX 21
#endif

#ifndef PI_UART_PORT
#define PI_UART_PORT 0
#endif
#ifndef PI_UART_BAUD
#define PI_UART_BAUD 460800
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 10
#endif

static HardwareSerial PiSerial(PI_UART_PORT);

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t sub;
static rcl_subscription_t session_sub;
static rcl_subscription_t heartbeat_sub;
#if defined(ROVER_SONAR)
static rcl_subscription_t sonar_cal_sweep_sub;
#endif
static rcl_publisher_t imu_pub;
static rcl_publisher_t bump_pub;
static rcl_publisher_t stall_pub;
static rcl_publisher_t button_pub;
static rcl_publisher_t button_event_pub;
static rcl_publisher_t drive_mode_pub;
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
static rcl_publisher_t ir_front_pub;
#endif
static rcl_publisher_t ir_aux_result_pub;
static rcl_publisher_t wheel_l_ticks_pub;
static rcl_publisher_t wheel_r_ticks_pub;
static rcl_publisher_t sonar_range_pub;
static rcl_publisher_t sonar_pan_pub;
static rcl_publisher_t sonar_avoid_pub;
static rcl_publisher_t sonar_escape_event_pub;
static rcl_publisher_t sonar_escape_turn_pub;
static rcl_publisher_t sonar_escape_best_deg_pub;
static rcl_publisher_t sonar_escape_best_m_pub;
#if defined(ROVER_VL53L)
static rcl_publisher_t tof_left_pub;
static rcl_publisher_t tof_right_pub;
#endif
static rcl_subscription_t ir_aux_sample_sub;
static rcl_subscription_t servo_angle_sub;
static rclc_executor_t executor;
static geometry_msgs__msg__Twist cmd_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Bool bump_msg;
static std_msgs__msg__Bool stall_msg;
static std_msgs__msg__Bool button_msg;
static std_msgs__msg__UInt8 button_event_msg;
static std_msgs__msg__UInt8 drive_mode_msg;
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
static std_msgs__msg__Bool ir_front_msg;
#endif
static std_msgs__msg__UInt8 ir_aux_result_msg;
static std_msgs__msg__UInt8 ir_aux_sample_msg;
static std_msgs__msg__Float32 servo_angle_msg;
static std_msgs__msg__UInt32 wheel_l_ticks_msg;
static std_msgs__msg__UInt32 wheel_r_ticks_msg;
static sensor_msgs__msg__Range sonar_range_msg;
static std_msgs__msg__Float32 sonar_pan_msg;
static std_msgs__msg__Bool sonar_avoid_msg;
static std_msgs__msg__UInt8 sonar_escape_event_msg;
static std_msgs__msg__Float32 sonar_escape_turn_msg;
static std_msgs__msg__Float32 sonar_escape_best_deg_msg;
static std_msgs__msg__Float32 sonar_escape_best_m_msg;
#if defined(ROVER_VL53L)
static sensor_msgs__msg__Range tof_left_msg;
static sensor_msgs__msg__Range tof_right_msg;
#endif
static std_msgs__msg__Bool session_msg;
static std_msgs__msg__UInt32 heartbeat_msg;
#if defined(ROVER_SONAR)
static std_msgs__msg__Bool sonar_cal_sweep_msg;
#endif

static SmoothDrive drv;
static Mpu6050 mpu;
static HeadingHold heading;
static MotionDetect motion;
static StatusLed status_led;

#ifdef ROVER_TDISPLAY_S3
static BatteryMonitor battery;
static EStop estop;
static GoButton go_button;
static RoverDisplay display;
static RoverOta ota;
static RoverUiData ui;
static uint32_t stall_block_until = 0;
static bool pi_session_sub_value = false;
static uint32_t last_pi_traffic_ms = 0;
static bool bridge_live_latched = false;
static uint32_t ros_connected_at_ms = 0;
static bool mode_menu_open = false;
static bool power_menu_open = false;
static RoverDriveMode selected_drive_mode = kDriveExplore;
static RoverPowerAction selected_power_action = kPowerShutdown;
#endif

#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
static RoverMcpIr mcp_ir;
static bool ir_aux_sample_pending = false;
static bool ir_aux_sample_front = false;
static bool ir_pub_front = false;
static uint32_t ir_front_flash_until = 0;
static bool ir_wheels_on = false;
static I2cScanResult ir_i2c_scan;
static uint32_t last_ir_retry_ms = 0;
static uint32_t last_ir_scan_ms = 0;

// Aux IR sweep (rear 180°) for ring visualization during scans.
static uint8_t ir_sweep_step = 0;
static uint32_t ir_sweep_step_ms = 0;
static bool ir_sweep_waiting = false;
static float ir_sweep_deg = 90.0f;
#endif

#ifdef ROVER_TDISPLAY_S3
static RoverPca9685 pca9685;
static float servo_target_deg = 90.0f;
static IrTx ir_beacon;
#if defined(ROVER_SONAR)
static Ultrasonic sonar;
static RoverSonar rover_sonar;
static TaskHandle_t pan_task_handle = nullptr;
#endif
#if defined(ROVER_VL53L)
static RoverVl53 side_tof;
#endif
#if defined(ROVER_PERIPH)
static RoverPeriph rover_periph;
static bool periph_ready_played = false;
#endif
#endif

#ifdef ROVER_TDISPLAY_S3
static char boot_reason_buf[24] = {};
static uint32_t boot_reason_until_ms = 0;
#endif

static float last_lin = 0, last_ang = 0;
static uint32_t last_drive_ms = 0;
static float last_heading_trim = 0.0f;
static float last_imu_yaw_deg = 0.0f;
static float last_imu_gz_dps = 0.0f;
static float last_imu_az_g = 0.0f;
static bool imu_rest_logged = false;
#if defined(ROVER_SONAR)
static bool sonar_drive_override = false;
static float sonar_drive_lin = 0.0f;
static float sonar_drive_ang = 0.0f;
static bool session_cal_sweep_triggered = false;
static bool sonar_scan_hold_logged = false;
static uint32_t sonar_cal_summary_until_ms = 0;
static bool sonar_cal_was_active = false;
static bool go_cal_sweep_pending = false;

static bool explore_motion_armed() {
  return pi_session_sub_value || go_cal_sweep_pending ||
         (millis() - last_drive_ms < 1500);
}

static bool sonar_scan_hold() {
  return explore_motion_armed() && rover_sonar.cal_sweep_active();
}

static void update_session_cal_sweep(uint32_t now_ms) {
  const bool cal_active = rover_sonar.cal_sweep_active();
  if (sonar_cal_was_active && !cal_active) {
    sonar_cal_summary_until_ms = now_ms + 30000;
    go_cal_sweep_pending = false;
    rover_diag_ckpt(kCkptCalDone, rover_sonar.cal_snap_count());
    rover_diag_event("cal_done snaps=%u", rover_sonar.cal_snap_count());
  }
  sonar_cal_was_active = cal_active;

  if (!explore_motion_armed()) {
    session_cal_sweep_triggered = false;
    sonar_scan_hold_logged = false;
    sonar_cal_summary_until_ms = 0;
    sonar_cal_was_active = false;
    go_cal_sweep_pending = false;
    rover_sonar.set_cal_sweep(false);
    rover_sonar.set_pan_scan_enabled(false);
    return;
  }
  // Cain request (Aug 2026): disable cruise wiggle while driving.
  // Hold pan forward; only move during explicit scans (cal_sweep).
  rover_sonar.set_pan_scan_enabled(false);
  if (!session_cal_sweep_triggered && !rover_sonar.cal_sweep_active()) {
    if (go_cal_sweep_pending) {
      rover_sonar.set_cal_sweep(true);
      session_cal_sweep_triggered = true;
      go_cal_sweep_pending = false;
      rover_diag_event("cal_start");
    }
  }
  if (sonar_scan_hold()) {
    if (!sonar_scan_hold_logged) {
      sonar_scan_hold_logged = true;
      rover_diag_event("cal_hold motors=off");
    }
  } else if (sonar_scan_hold_logged) {
    sonar_scan_hold_logged = false;
    rover_diag_event("cal_hold motors=on");
  }
}
#endif

static const char *reset_reason_str(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_EXT:
      return "external";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "int_wdt";
    case ESP_RST_TASK_WDT:
      return "task_wdt";
    case ESP_RST_WDT:
      return "wdt";
    case ESP_RST_DEEPSLEEP:
      return "deepsleep";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "unknown";
  }
}

enum class RosState { kWaitAgent, kConnect, kConnected, kDisconnect };

static RosState ros_state = RosState::kWaitAgent;
static bool entities_ok = false;

#define RCCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      Serial.printf("micro-ROS init failed rc=%d at %s:%d\n", (int)rc, __FILE__, \
                    __LINE__); \
      return false; \
    } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    (void)rc; \
  }

void on_cmd(const void *msgin);

#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
void on_ir_aux_sample(const void *msgin) {
  const auto *m = static_cast<const std_msgs__msg__UInt8 *>(msgin);
  if (m->data == 0) return;
  ir_aux_sample_pending = true;
  ir_aux_sample_front = (m->data == 2);
}
#endif

#ifdef ROVER_TDISPLAY_S3
void on_servo_angle(const void *msgin) {
  const auto *m = static_cast<const std_msgs__msg__Float32 *>(msgin);
  servo_target_deg = m->data;
  if (pca9685.ok()) {
    pca9685.setAngle(ROVER_AUX_SERVO_CHANNEL, servo_target_deg, 0.0f, 180.0f, 500, 2500);
  }
}
#endif

void on_session(const void *msgin) {
  const auto *m = static_cast<const std_msgs__msg__Bool *>(msgin);
  const bool prev = pi_session_sub_value;
  pi_session_sub_value = m->data;
  last_pi_traffic_ms = millis();
  bridge_live_latched = true;
  if (prev != pi_session_sub_value) {
    rover_diag_event("session %s", pi_session_sub_value ? "on" : "off");
    if (!pi_session_sub_value) {
      drv.set_target(0.0f, 0.0f);
      last_lin = 0.0f;
      last_ang = 0.0f;
      heading.disarm();
#if defined(ROVER_SONAR)
      rover_sonar.set_cal_sweep(false);
      rover_sonar.set_pan_scan_enabled(false);
#endif
    }
#if defined(ROVER_PERIPH)
    rover_periph.notify_session(pi_session_sub_value);
#endif
  }
}

#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_SONAR)
void on_sonar_cal_sweep(const void *msgin) {
  const auto *m = static_cast<const std_msgs__msg__Bool *>(msgin);
  rover_sonar.set_cal_sweep(m->data);
}
#endif

void on_heartbeat(const void *msgin) {
  (void)msgin;
  // Session state comes only from /rover/session — do NOT mirror the
  // heartbeat driving bit into pi_session_sub_value (that cleared session
  // whenever the rover slowed/stopped and killed pan scan + UI explore mode).
  last_pi_traffic_ms = millis();
  bridge_live_latched = true;
}

static void spin_ros_input() {
  if (ros_state != RosState::kConnected) return;
  const int spins = pi_session_sub_value ? 2 : 8;
  for (int i = 0; i < spins; i++) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  }
}

static void update_pi_bridge_state(uint32_t now) {
  (void)now;
  ui.pi_bridge_live = (ros_state == RosState::kConnected);
  ui.session_active = ui.pi_bridge_live && pi_session_sub_value;
  bridge_live_latched = ui.pi_bridge_live;
}

static bool drive_blocked() {
#ifdef ROVER_TDISPLAY_S3
  if (estop.active()) return true;
  if (battery.critical()) return true;
  if (millis() < stall_block_until) return true;
#endif
  return false;
}

#ifdef ROVER_TDISPLAY_S3
static void publish_button_event(uint8_t code) {
  if (ros_state != RosState::kConnected) return;
  button_event_msg.data = code;
  RCSOFTCHECK(rcl_publish(&button_event_pub, &button_event_msg, NULL));
  Serial.printf("EVENT button_event -> %u\n", (unsigned)code);
}

static void publish_drive_mode() {
  if (ros_state != RosState::kConnected) return;
  drive_mode_msg.data = static_cast<uint8_t>(selected_drive_mode);
  RCSOFTCHECK(rcl_publish(&drive_mode_pub, &drive_mode_msg, NULL));
  Serial.printf("EVENT drive_mode -> %s\n", drive_mode_label(selected_drive_mode));
}

static void cycle_drive_mode() {
  selected_drive_mode = static_cast<RoverDriveMode>(
    (static_cast<uint8_t>(selected_drive_mode) + 1) % kDriveModeCount);
  publish_drive_mode();
  publish_button_event(kBtnMenuCycle);
}

static void cycle_power_action() {
  selected_power_action = static_cast<RoverPowerAction>(
    (static_cast<uint8_t>(selected_power_action) + 1) % kPowerActionCount);
  publish_button_event(kBtnMenuCycle);
}

static void confirm_power_action() {
  power_menu_open = false;
  switch (selected_power_action) {
    case kPowerShutdown:
      publish_button_event(kBtnPowerShutdown);
      break;
    case kPowerReboot:
      publish_button_event(kBtnPowerReboot);
      break;
    default:
      publish_button_event(kBtnGoLong);
      break;
  }
}

static uint32_t menu_saved_flash_until = 0;
static uint32_t key_flash_until = 0;
static uint32_t button_priority_until = 0;

#if defined(ROVER_SONAR)
static char ui_note_buf[64] = {};
static uint32_t ui_note_until_ms = 0;
static Preferences prefs;
static bool prefs_ok = false;

// Declared later in this file.
static void force_stop(const char *reason);

static float read_stable_range_m(uint32_t timeout_ms,
                                 float eps_m,
                                 uint8_t need_samples,
                                 float *out_range_m) {
  uint32_t t0 = millis();
  float last = -1.0f;
  uint8_t stable = 0;
  float best = -1.0f;
  while (millis() - t0 < timeout_ms) {
    const float r = sonar.read_range_m();
    if (r > 0.0f) {
      best = r;
      if (last > 0.0f && fabsf(r - last) <= eps_m) {
        stable++;
      } else {
        stable = 0;
      }
      last = r;
      if (stable + 1 >= need_samples) {
        if (out_range_m) {
          *out_range_m = best;
        }
        return best;
      }
    } else {
      stable = 0;
      last = -1.0f;
    }
    // Keep micro-ROS responsive while we bench-calibrate.
    spin_ros_input();
#if defined(ROVER_PERIPH)
    rover_periph.tick(millis());
#endif
  }
  if (out_range_m) {
    *out_range_m = best;
  }
  return best;
}

static uint32_t measure_pan_settle_ms(float target_deg,
                                      float eps_m = 0.03f,
                                      uint8_t need_samples = 3,
                                      uint32_t timeout_ms = 2500) {
  const uint32_t t0 = millis();
  rover_sonar.set_pan_deg(target_deg);
  float out = -1.0f;
  (void)read_stable_range_m(timeout_ms, eps_m, need_samples, &out);
  return millis() - t0;
}

static void run_pan_timing_cal(uint32_t now_ms) {
  (void)now_ms;

  // Safety: never run during an active driving session.
  if (pi_session_sub_value) {
    return;
  }

  rover_diag_event("pan_cal start");

  // Stop wheels, stop any scan mode.
  force_stop("pan_cal");
  rover_sonar.set_cal_sweep(false);
  rover_sonar.set_pan_scan_enabled(false);

  // Pause the dedicated pan task so this routine exclusively owns pan writes.
  if (pan_task_handle) {
    vTaskSuspend(pan_task_handle);
  }

  // Move to center and allow a quick settle.
  rover_sonar.set_pan_deg(90.0f);
  delay(220);

  // User setup: front open; obstacle hard-left (0°) and hard-right (180°).
  const uint32_t left_ms = measure_pan_settle_ms(0.0f);
  delay(120);
  const uint32_t right_ms = measure_pan_settle_ms(180.0f);
  delay(120);
  rover_sonar.set_pan_deg(90.0f);

  if (pan_task_handle) {
    vTaskResume(pan_task_handle);
  }

  const float ms_per_deg = fmaxf(static_cast<float>(left_ms), static_cast<float>(right_ms)) / 90.0f;
  rover_sonar.set_cal_ms_per_deg(ms_per_deg);
  if (prefs_ok) {
    prefs.putFloat("pan_ms_deg", ms_per_deg);
    prefs.putUInt("pan_left_ms", left_ms);
    prefs.putUInt("pan_right_ms", right_ms);
  }

  snprintf(ui_note_buf, sizeof(ui_note_buf), "PAN CAL L=%lums R=%lums (%.2fms/deg)",
           static_cast<unsigned long>(left_ms), static_cast<unsigned long>(right_ms), ms_per_deg);
  ui_note_until_ms = millis() + 15000;
  rover_diag_event("%s", ui_note_buf);

  // Short confirm.
#if defined(ROVER_PERIPH)
  rover_periph.play(RoverPeriph::kMelodyMenuDone);
#endif
}
#endif

static void flash_key_feedback(uint32_t now_ms) {
  key_flash_until = now_ms + 220;
  button_priority_until = now_ms + 500;
}

static void handle_go_button_event(RoverButtonEvent ev, uint32_t now_ms) {
  if (ev == kBtnNone) return;
  flash_key_feedback(now_ms);

  if (ev == kBtnGoTriple) {
    if (mode_menu_open) {
      return;
    }
    power_menu_open = true;
    mode_menu_open = false;
    selected_power_action = kPowerShutdown;
    rover_periph.play(RoverPeriph::kMelodyMenu);
    publish_button_event(kBtnGoTriple);
    Serial.println("EVENT power menu open");
    return;
  }

  if (ev == kBtnGoLong) {
    if (power_menu_open) {
      confirm_power_action();
      return;
    }
    if (pi_session_sub_value) {
      return;
    }
    mode_menu_open = !mode_menu_open;
    power_menu_open = false;
    publish_button_event(kBtnGoLong);
    if (mode_menu_open) {
      rover_periph.play(RoverPeriph::kMelodyMenu);
      publish_drive_mode();
    } else {
      rover_periph.play(RoverPeriph::kMelodyMenuDone);
      menu_saved_flash_until = millis() + 6000;
    }
    return;
  }

  if (ev == kBtnGoShort) {
    if (power_menu_open) {
      cycle_power_action();
      rover_periph.play(RoverPeriph::kMelodyButton);
      return;
    }
    if (mode_menu_open && !pi_session_sub_value) {
      cycle_drive_mode();
      rover_periph.play(RoverPeriph::kMelodyButton);
      return;
    }
    power_menu_open = false;
    mode_menu_open = false;
    menu_saved_flash_until = 0;
    rover_periph.play(RoverPeriph::kMelodyButton);

#if defined(ROVER_SONAR)
    // Bench utility: when "ServoCal" mode is selected and we're in standby,
    // use Go short to run a local pan timing calibration instead of starting a session.
    if (!pi_session_sub_value && selected_drive_mode == kDriveServoCal) {
      run_pan_timing_cal(now_ms);
      return;
    }
#endif

    if (ros_state == RosState::kConnected) {
      button_msg.data = true;
      RCSOFTCHECK(rcl_publish(&button_pub, &button_msg, NULL));
      Serial.println("EVENT button -> /rover/button");
      for (int i = 0; i < 12; i++) {
        spin_ros_input();
      }
    }
  }
}
#endif

static void force_stop(const char *reason) {
  last_lin = 0;
  last_ang = 0;
  heading.disarm();
#if defined(ROVER_SONAR)
  sonar_drive_override = false;
  sonar_drive_lin = 0.0f;
  sonar_drive_ang = 0.0f;
  if (reason != nullptr) {
    rover_sonar.abort_escape(millis());
  }
#endif
  drv.stop();
  if (reason) {
    Serial.printf("DRIVE STOP: %s\n", reason);
  }
}

static uint32_t bump_event_cooldown_until = 0;

static void publish_bump_event(const char* source) {
  const uint32_t now = millis();
  if (now < bump_event_cooldown_until) {
    return;
  }
  bump_event_cooldown_until = now + BUMP_COOLDOWN_MS;

  if (ros_state == RosState::kConnected) {
    bump_msg.data = true;
    RCSOFTCHECK(rcl_publish(&bump_pub, &bump_msg, NULL));
  }
#if defined(ROVER_PERIPH)
  rover_periph.notify_bump();
#endif
#ifdef ROVER_TDISPLAY_S3
  ui.bump = true;
#endif
  Serial.printf("EVENT bump — %s\n", source ? source : "?");
}

#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
static void poll_mcp_ir(uint32_t now) {
  const bool session_driving =
      pi_session_sub_value && (now - last_drive_ms < 1000);
  if (!session_driving && now - last_ir_scan_ms >= 5000) {
    last_ir_scan_ms = now;
    ir_i2c_scan = scanI2cBus();
  }

  if (!mcp_ir.front_ok() || !mcp_ir.body_ok()) {
    if (now - last_ir_retry_ms >= 2000) {
      last_ir_retry_ms = now;
      mcp_ir.begin(MPU_SDA, MPU_SCL);
    }
  }
  if (!mcp_ir.ok()) {
    return;
  }

  const bool want_wheels =
      pi_session_sub_value || (now - last_drive_ms < 1000);
  if (want_wheels != ir_wheels_on) {
    ir_wheels_on = want_wheels;
    mcp_ir.set_wheels_enabled(want_wheels);
  }

  mcp_ir.tick(now);

  const bool front = mcp_ir.front_hit();
  const bool rover_moving =
      pi_session_sub_value || (now - last_drive_ms < 1000);
  static bool last_front_bumper = false;
  if (mcp_ir.front_ok() && rover_moving && front && !last_front_bumper) {
    publish_bump_event("front IR bumper");
  }
  last_front_bumper = front;

#if defined(ROVER_DISABLE_FRONT_IR)
  (void)front;
  const bool front_for_ros = false;
#else
  const bool front_for_ros = front;
#endif
  if (front_for_ros) {
    ir_front_flash_until = now + 500;
  }

#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
  static uint32_t last_ir_front_pub = 0;
  static bool last_ir_front_sent = false;
  if (ros_state == RosState::kConnected) {
    const uint32_t pub_period = front_for_ros ? 25u : 1000u;
    const bool edge = front_for_ros != last_ir_front_sent;
    if (edge || (now - last_ir_front_pub >= pub_period)) {
      last_ir_front_pub = now;
      last_ir_front_sent = front_for_ros;
      ir_pub_front = front_for_ros;
      ir_front_msg.data = front_for_ros;
      RCSOFTCHECK(rcl_publish(&ir_front_pub, &ir_front_msg, NULL));
    }
  }
#endif

  if (ir_aux_sample_pending) {
    bool process_aux = true;
#if defined(ROVER_SONAR)
    static uint32_t aux_defer_since = 0;
    if (rover_sonar.avoid_active()) {
      if (aux_defer_since == 0) {
        aux_defer_since = now;
      }
      if (now - aux_defer_since < 600) {
        process_aux = false;
      }
    } else {
      aux_defer_since = 0;
    }
#endif
    if (!process_aux) {
      // Keep pending — wheel ticks etc. still run below.
    } else {
    ir_aux_sample_pending = false;
    const bool want_front = ir_aux_sample_front;
    ir_aux_sample_front = false;

    bool aux_off = false;
    bool aux_on = false;
    bool front_off = false;
    bool front_on = false;

    if (want_front) {
      mcp_ir.sample_front(MCP_IR_FRONT_SAMPLE_OFF_MS, MCP_IR_FRONT_SAMPLE_ON_MS, front_off,
                          front_on);
    } else {
      mcp_ir.sample_aux(MCP_IR_AUX_OFF_MS, MCP_IR_AUX_ON_MS, aux_off, aux_on);
    }

    ir_aux_result_msg.data = static_cast<uint8_t>(
        (aux_off ? 1u : 0u) | (aux_on ? 2u : 0u) | (front_off ? 4u : 0u) | (front_on ? 8u : 0u));
    if (ros_state == RosState::kConnected) {
      RCSOFTCHECK(rcl_publish(&ir_aux_result_pub, &ir_aux_result_msg, NULL));
    }
    Serial.printf("IR sample aux off=%d on=%d front off=%d on=%d\n", (int)aux_off, (int)aux_on,
                  (int)front_off, (int)front_on);
    }
  }

  static uint32_t last_wheel_pub = 0;
  if (ros_state == RosState::kConnected && (now - last_wheel_pub >= 100)) {
    last_wheel_pub = now;
    wheel_l_ticks_msg.data = mcp_ir.wheel_left_ticks();
    wheel_r_ticks_msg.data = mcp_ir.wheel_right_ticks();
    RCSOFTCHECK(rcl_publish(&wheel_l_ticks_pub, &wheel_l_ticks_msg, NULL));
    RCSOFTCHECK(rcl_publish(&wheel_r_ticks_pub, &wheel_r_ticks_msg, NULL));
  }
}
#endif

// micro-ROS hard limits (see metas/colcon_rover.meta): keep pub/sub counts within budget.
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "lego_rover_esp32", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_publisher_init_default(
    &bump_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rover/bump"));
  RCCHECK(rclc_publisher_init_default(
    &stall_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rover/stall"));
  RCCHECK(rclc_publisher_init_default(
    &button_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rover/button"));
  RCCHECK(rclc_publisher_init_default(
    &button_event_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rover/button_event"));
  RCCHECK(rclc_publisher_init_default(
    &drive_mode_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rover/drive_mode"));
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
  RCCHECK(rclc_publisher_init_default(
    &ir_front_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rover/ir/front"));
#endif
  RCCHECK(rclc_publisher_init_default(
    &ir_aux_result_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rover/ir/aux_result"));
  RCCHECK(rclc_subscription_init_default(
    &ir_aux_sample_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rover/ir/aux_sample"));
  RCCHECK(rclc_publisher_init_default(
    &wheel_l_ticks_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "rover/wheel/left_ticks"));
  RCCHECK(rclc_publisher_init_default(
    &wheel_r_ticks_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "rover/wheel/right_ticks"));
#endif
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_SONAR)
  RCCHECK(rclc_publisher_init_default(
    &sonar_range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "rover/sonar/range"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_pan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rover/sonar/pan_deg"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_avoid_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "rover/sonar/avoid_active"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_escape_event_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "rover/sonar/escape_event"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_escape_turn_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rover/sonar/escape_turn"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_escape_best_deg_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rover/sonar/escape_best_deg"));
  RCCHECK(rclc_publisher_init_default(
    &sonar_escape_best_m_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rover/sonar/escape_best_m"));
  RCCHECK(rclc_subscription_init_default(
    &sonar_cal_sweep_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "rover/sonar/cal_sweep"));
#endif
#if defined(ROVER_VL53L)
  RCCHECK(rclc_publisher_init_default(
    &tof_left_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "rover/tof/left"));
  RCCHECK(rclc_publisher_init_default(
    &tof_right_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "rover/tof/right"));
#endif
#ifdef ROVER_TDISPLAY_S3
  RCCHECK(rclc_subscription_init_default(
    &servo_angle_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rover/servo/angle"));
#endif
  RCCHECK(rclc_subscription_init_default(
    &session_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rover/session"));
  RCCHECK(rclc_subscription_init_default(
    &heartbeat_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "rover/heartbeat"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &heartbeat_sub, &heartbeat_msg, &on_heartbeat, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &session_sub, &session_msg, &on_session, ON_NEW_DATA));
#if defined(ROVER_SONAR)
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sonar_cal_sweep_sub, &sonar_cal_sweep_msg, &on_sonar_cal_sweep,
    ON_NEW_DATA));
#endif
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
  RCCHECK(rclc_executor_add_subscription(
    &executor, &ir_aux_sample_sub, &ir_aux_sample_msg, &on_ir_aux_sample, ON_NEW_DATA));
#endif
#ifdef ROVER_TDISPLAY_S3
  RCCHECK(rclc_executor_add_subscription(
    &executor, &servo_angle_sub, &servo_angle_msg, &on_servo_angle, ON_NEW_DATA));
#endif
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub, &cmd_msg, &on_cmd, ON_NEW_DATA));
  bump_msg.data = false;
  stall_msg.data = false;
  button_msg.data = false;
  button_event_msg.data = 0;
  drive_mode_msg.data = static_cast<uint8_t>(kDriveExplore);
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
  ir_front_msg.data = false;
#endif
  ir_aux_result_msg.data = 0;
  ir_aux_sample_msg.data = 0;
  wheel_l_ticks_msg.data = 0;
  wheel_r_ticks_msg.data = 0;
  servo_angle_msg.data = 90.0f;
#if defined(ROVER_SONAR)
  sonar_range_msg.range = std::numeric_limits<float>::quiet_NaN();
  sonar_range_msg.min_range = 0.02f;
  sonar_range_msg.max_range = 4.0f;
  sonar_range_msg.field_of_view = 0.26f;
  sonar_range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  sonar_pan_msg.data = static_cast<float>(SONAR_PAN_CENTER_DEG);
  sonar_avoid_msg.data = false;
  sonar_escape_event_msg.data = 0;
  sonar_escape_turn_msg.data = 0.0f;
  sonar_escape_best_deg_msg.data = static_cast<float>(SONAR_PAN_CENTER_DEG);
  sonar_escape_best_m_msg.data = 0.0f;
#endif
#if defined(ROVER_VL53L)
  tof_left_msg.range = std::numeric_limits<float>::quiet_NaN();
  tof_left_msg.min_range = 0.02f;
  tof_left_msg.max_range = 1.3f;
  tof_left_msg.field_of_view = 0.47f;
  tof_left_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  tof_right_msg = tof_left_msg;
#endif
  session_msg.data = false;
  pi_session_sub_value = false;
  return true;
}

static void destroy_entities() {
  if (!entities_ok) return;

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_subscription_fini(&sub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&session_sub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&heartbeat_sub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&imu_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&bump_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&stall_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&button_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&button_event_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&drive_mode_pub, &node));
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR)
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
  RCSOFTCHECK(rcl_publisher_fini(&ir_front_pub, &node));
#endif
  RCSOFTCHECK(rcl_publisher_fini(&ir_aux_result_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&wheel_l_ticks_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&wheel_r_ticks_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&ir_aux_sample_sub, &node));
#endif
#if defined(ROVER_TDISPLAY_S3) && defined(ROVER_SONAR)
  RCSOFTCHECK(rcl_publisher_fini(&sonar_range_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_pan_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_avoid_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_escape_event_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_escape_turn_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_escape_best_deg_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&sonar_escape_best_m_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sonar_cal_sweep_sub, &node));
#endif
#if defined(ROVER_VL53L)
  RCSOFTCHECK(rcl_publisher_fini(&tof_left_pub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tof_right_pub, &node));
#endif
#ifdef ROVER_TDISPLAY_S3
  RCSOFTCHECK(rcl_subscription_fini(&servo_angle_sub, &node));
#endif
  rclc_executor_fini(&executor);
  RCSOFTCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
  entities_ok = false;
}

static void on_session_lost() {
  force_stop("agent lost");
  destroy_entities();
  ros_state = RosState::kWaitAgent;
  ros_connected_at_ms = 0;
  Serial.println("micro-ROS agent lost — waiting to reconnect...");
}

static void apply_drive(float yaw_rate, float dt) {
  if (drive_blocked()) {
    force_stop(nullptr);
    return;
  }

  constexpr uint32_t CMD_TIMEOUT_MS = 400;
  const bool cmd_fresh = (millis() - last_drive_ms) <= CMD_TIMEOUT_MS;
#if defined(ROVER_SONAR)
  if (!sonar_drive_override && (!cmd_fresh || !pi_session_sub_value)) {
#else
  if (!cmd_fresh || !pi_session_sub_value) {
#endif
    drv.set_target(0.0f, 0.0f);
    return;
  }

#if defined(ROVER_SONAR)
  if (sonar_scan_hold()) {
    drv.set_target(0.0f, 0.0f);
    return;
  }
#endif

#if defined(ROVER_SONAR)
  float v = sonar_drive_lin;
  float turn = sonar_drive_ang;
#else
  float v = last_lin;
  float turn = last_ang;
#endif
  float trim = 0;

#if defined(ROVER_SONAR)
  const bool sonar_override = sonar_drive_override;
#else
  const bool sonar_override = false;
#endif
  if (!sonar_override) {
#if defined(ROVER_MCP_IR) && !defined(ROVER_DISABLE_FRONT_IR)
#ifndef FORWARD_LINEAR_SIGN
#define FORWARD_LINEAR_SIGN 1.0f
#endif
    if (mcp_ir.front_ok() && mcp_ir.front_hit()) {
      const bool reversing =
          (FORWARD_LINEAR_SIGN > 0.0f) ? (v < -0.03f) : (v > 0.03f);
      if (!reversing) {
        v = 0.0f;
        turn = 0.0f;
      }
    }
#endif
  }

  if (mpu.ok() && !sonar_override) {
#ifndef HEADING_HOLD
#define HEADING_HOLD 1
#endif
#if HEADING_HOLD
    trim = heading.correct(yaw_rate, dt, v, turn);
#endif
  } else {
    heading.disarm();
  }
  static float trim_out = 0.0f;
  const float trim_step = 0.35f * ((dt > 0.0001f && dt < 0.05f) ? dt : 0.01f);
  if (trim > trim_out + trim_step) {
    trim_out += trim_step;
  } else if (trim < trim_out - trim_step) {
    trim_out -= trim_step;
  } else {
    trim_out = trim;
  }
  last_heading_trim = trim_out;

  float l, r;

  constexpr float STEER_DEAD = 0.02f;
  constexpr float MIN_WHEEL = 0.05f;

  // Never in-place spin. Opposite-wheel commands are how the waggle happens;
  // clamp turn so both wheels keep the sign of v (or both sit at ~0).
  if (fabsf(turn) < STEER_DEAD) {
    l = v + trim_out;
    r = v - trim_out;
  } else if (fabsf(v) < MIN_WHEEL) {
    l = v;
    r = v;
  } else {
    const float max_turn = fabsf(v) - MIN_WHEEL;
    if (fabsf(turn) > max_turn) {
      turn = (turn > 0.0f) ? max_turn : -max_turn;
    }
    l = v - turn;
    r = v + turn;
  }

  const float peak = fmaxf(fabsf(l), fabsf(r));
  if (peak > 1.0f) {
    l /= peak;
    r /= peak;
  }

  l = constrain(l, -1.0f, 1.0f);
  r = constrain(r, -1.0f, 1.0f);
  drv.set_target(l, r);
}

void on_cmd(const void *msgin) {
  if (drive_blocked()) {
    return;
  }
  if (!pi_session_sub_value) {
    return;
  }
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
#ifndef CMD_VEL_CROSS_FIX
#define CMD_VEL_CROSS_FIX 0
#endif
#if CMD_VEL_CROSS_FIX
  last_lin = msg->angular.z;
  last_ang = msg->linear.x;
#else
  last_lin = msg->linear.x;
  last_ang = msg->angular.z;
#endif
  last_drive_ms = millis();
  last_pi_traffic_ms = millis();
  bridge_live_latched = true;
}

static void handle_ros_state() {
  switch (ros_state) {
    case RosState::kWaitAgent:
      if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
        Serial.println("micro-ROS agent found — connecting...");
        ros_state = RosState::kConnect;
      }
      break;

    case RosState::kConnect:
      if (create_entities()) {
        entities_ok = true;
        ros_state = RosState::kConnected;
        ros_connected_at_ms = millis();
        Serial.println("micro-ROS ready");
#if defined(ROVER_PERIPH)
        if (!periph_ready_played) {
          periph_ready_played = true;
          rover_periph.notify_ready();
        }
#endif
#if defined(ROVER_VL53L)
        rover_diag_event("vl53_link L=%s R=%s", side_tof.left_ok() ? "OK" : "--",
                         side_tof.right_ok() ? "OK" : "--");
#endif
#ifdef ROVER_TDISPLAY_S3
        publish_drive_mode();
#endif
      } else {
        Serial.println("micro-ROS init failed — retrying...");
        destroy_entities();
        ros_state = RosState::kWaitAgent;
        delay(500);
      }
      break;

    case RosState::kConnected:
      if (RMW_RET_OK != rmw_uros_ping_agent(800, 6)) {
        ros_state = RosState::kDisconnect;
      }
      break;

    case RosState::kDisconnect:
      on_session_lost();
      break;
  }
}

static int link_state_code() {
  switch (ros_state) {
    case RosState::kConnect:
      return 1;
    case RosState::kConnected:
      return 2;
    case RosState::kDisconnect:
      return 3;
    default:
      return 0;
  }
}

static void update_status_led(uint32_t now_ms) {
  constexpr uint32_t CMD_TIMEOUT_MS = 400;
  const bool cmd_fresh = (now_ms - last_drive_ms) <= CMD_TIMEOUT_MS;
  const bool moving =
      cmd_fresh && (fabsf(last_lin) > 0.03f || fabsf(last_ang) > 0.03f);

  switch (ros_state) {
    case RosState::kWaitAgent:
      status_led.set_state(LedState::kWaitAgent);
      break;
    case RosState::kConnect:
      status_led.set_state(LedState::kConnecting);
      break;
    case RosState::kConnected:
      status_led.set_state(moving ? LedState::kDriving : LedState::kReady);
      break;
    case RosState::kDisconnect:
      status_led.set_state(LedState::kLinkLost);
      break;
  }
  status_led.tick(now_ms);
}

static void calibrate_gyro() {
  if (!mpu.ok()) return;
  Serial.println("MPU gyro cal — hold rover still...");
  heading.begin();
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int n = 0;
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
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;
      n++;
    }
    status_led.tick(now);
    delay(5);
  }
  if (n > 0) {
    Serial.printf("Gyro bias=%.4f rad/s ready=%d\n", heading.bias_rad_s(),
                  heading.ready() ? 1 : 0);
    Serial.printf("MPU at rest: ax=%.2f ay=%.2f az=%.2f g  ", sum_ax / n, sum_ay / n,
                  sum_az / n);
    Serial.println("(flat + Z-up: |az|~1g; spin CW from above: yaw rate should go negative)");
    rover_diag_event("imu_cal bias=%.4f ready=%d", heading.bias_rad_s(), heading.ready() ? 1 : 0);
  }
}

static bool imu_stationary() {
  constexpr uint32_t CMD_IDLE_MS = 400;
  if (millis() - last_drive_ms < CMD_IDLE_MS) {
    return false;
  }
  if (fabsf(drv.cur_left()) > 0.04f || fabsf(drv.cur_right()) > 0.04f) {
    return false;
  }
  if (fabsf(drv.tgt_left()) > 0.04f || fabsf(drv.tgt_right()) > 0.04f) {
    return false;
  }
  return true;
}

void setup() {
  rover_board_power_on();
  SmoothDrive::gpio_safe_idle(DIR_L, PWM_L, DIR_R, PWM_R);

  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);
  rover_diag_begin(reset_reason_str(esp_reset_reason()));
  Serial.printf("Reset reason: %s (%d)\n", reset_reason_str(esp_reset_reason()),
                static_cast<int>(esp_reset_reason()));
#ifdef ROVER_TDISPLAY_S3
  snprintf(boot_reason_buf, sizeof(boot_reason_buf), "RST:%s",
           reset_reason_str(esp_reset_reason()));
  boot_reason_until_ms = millis() + 15000;
#endif

#if !defined(ROVER_MICROROS_WIFI)
  PiSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
#endif
  status_led.begin(STATUS_LED_PIN);
  status_led.set_state(LedState::kBoot);
  heading.begin();
  motion.begin();

#ifdef ROVER_TDISPLAY_S3
  estop.begin(PIN_BUTTON_ESTOP);
  go_button.begin(PIN_BUTTON_GO);
  battery.begin(PIN_BAT_ADC, BAT_ADC_SCALE, BAT_VOLT_LOW, BAT_VOLT_CRITICAL);
#if defined(ROVER_MCP_IR)
  mcp_ir.begin(MPU_SDA, MPU_SCL);
  ir_i2c_scan = scanI2cBus();
  Serial.print("I2C scan:");
  for (uint8_t i = 0; i < ir_i2c_scan.count; i++) {
    Serial.printf(" 0x%02x", ir_i2c_scan.addrs[i]);
  }
  if (ir_i2c_scan.count == 0) {
    Serial.print(" (none)");
  }
  Serial.println();
  if (mcp_ir.front_ok()) {
    mcp_ir.logFrontIoState();
  }
  if (mcp_ir.body_ok()) {
    mcp_ir.logBodyIoState();
  }
  if (mcp_ir.ok()) {
    Serial.printf("MCP IR front=%s body=%s gates_live=%s (I2C SDA=%d SCL=%d)\n",
                  mcp_ir.front_ok() ? "OK" : "--", mcp_ir.body_ok() ? "OK" : "--",
                  mcp_ir.front_emitting() ? "yes" : "NO", MPU_SDA, MPU_SCL);
  } else {
    Serial.printf("MCP IR partial front=%s body=%s SDA=%d SCL=%d scan:",
                  mcp_ir.front_ok() ? "OK" : "--", mcp_ir.body_ok() ? "OK" : "--", MPU_SDA, MPU_SCL);
    for (uint8_t i = 0; i < ir_i2c_scan.count; i++) {
      Serial.printf(" 0x%02x", ir_i2c_scan.addrs[i]);
    }
    Serial.println();
  }
#endif
  if (pca9685.begin(PCA9685_ADDR)) {
    Serial.printf("PCA9685 OK @ 0x%02x aux ch %d sonar ch %d\n", (unsigned)PCA9685_ADDR,
                  (int)ROVER_AUX_SERVO_CHANNEL, (int)ROVER_SERVO_CHANNEL);
    pca9685.setAngle(ROVER_AUX_SERVO_CHANNEL, 90.0f, 0.0f, 180.0f, 500, 2500);
  } else {
    Serial.println("PCA9685 not found — aux servo unavailable");
  }
#if defined(ROVER_SONAR)
  if (sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN)) {
    rover_sonar.begin(&sonar, pca9685.ok() ? &pca9685 : nullptr);
    // Load persisted pan timing calibration (optional).
    if (!prefs_ok) {
      prefs_ok = prefs.begin("rover", false);
    }
    if (prefs_ok) {
      const float ms_deg = prefs.getFloat("pan_ms_deg", NAN);
      if (isfinite(ms_deg) && ms_deg > 0.5f && ms_deg < 12.0f) {
        rover_sonar.set_cal_ms_per_deg(ms_deg);
        rover_diag_event("pan_cal load ms/deg=%.2f", ms_deg);
      }
    }
#if ROVER_BOOT_PAN_SWEEP
    rover_sonar.boot_full_sweep();
#endif
    Serial.printf("Sonar OK TRIG=%d ECHO=%d pan ch %d\n", SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                  (int)ROVER_SERVO_CHANNEL);
    // The pan sweep gets its own task on the other core, on a fixed
    // schedule, so it never inherits the main loop's TFT/WiFi/ROS jitter
    // (measured at 10-150+ ms per iteration). Arduino's loopTask runs on
    // core 1, so pin this to core 0.
    xTaskCreatePinnedToCore(
        [](void*) {
          for (;;) {
            rover_sonar.update_pan_wiggle(millis());
            vTaskDelay(pdMS_TO_TICKS(20));
          }
        },
        "pan_wiggle", 3072, nullptr, 1, &pan_task_handle, 0);
    rover_diag_set_pan_task(pan_task_handle);
  } else {
    Serial.println("WARN: sonar init failed");
  }
#endif
#if defined(ROVER_VL53L)
  if (!side_tof.begin(&mcp_ir)) {
    Serial.println("WARN: VL53 init failed (check SDA/SCL, MCP GPA6/7 XSHUT, chip=L1X not L0X)");
  }
#endif
#if defined(ROVER_PERIPH)
  rover_periph.begin(PIN_ROVER_SPEAKER, PIN_ROVER_RING);
#endif
#endif

#ifdef ROVER_TDISPLAY_S3
  if (ir_beacon.begin(IR_TX_PIN, IR_TX_HZ, IR_TX_LEDC_CHANNEL)) {
    ir_beacon.set_enabled(IR_TX_DEFAULT_ON);
    Serial.printf("IR beacon %u Hz GPIO %d (%s, mod=%d)\n", (unsigned)IR_TX_HZ, IR_TX_PIN,
                  IR_TX_DEFAULT_ON ? "on" : "off", (int)IR_TX_MODULATE);
  } else {
    Serial.printf("WARN: IR beacon PWM failed on GPIO %d\n", IR_TX_PIN);
  }
#endif

  Serial.printf("Motors: A/LEFT dir=%d pwm=%d  B/RIGHT dir=%d pwm=%d @%uHz\n", DIR_L, PWM_L,
                DIR_R, PWM_R, (unsigned)PWM_FREQ_HZ);
  if (!drv.begin(DIR_L, PWM_L, DIR_R, PWM_R)) {
    Serial.println("FAIL: LEDC attach — check PWM pins");
  }

#ifdef ROVER_TDISPLAY_S3
  display.begin();
#if !defined(ROVER_MICROROS_WIFI)
  ota.begin("rover-esp");
#endif
  ui.mode = "boot";
#endif

  if (mpu.begin(MPU_SDA, MPU_SCL)) {
    Serial.println("MPU6050 OK");
    calibrate_gyro();
  } else {
    Serial.println("MPU6050 not found — no heading hold / bump detect");
    status_led.set_state(LedState::kMpuError);
  }

#if defined(ROVER_MICROROS_WIFI)
#ifndef ROVER_AGENT_IP
#define ROVER_AGENT_IP "192.168.2.31"
#endif
#ifndef ROVER_AGENT_PORT
#define ROVER_AGENT_PORT 8888
#endif
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("rover-esp");
  IPAddress rover_agent_ip;
  rover_agent_ip.fromString(ROVER_AGENT_IP);
  set_microros_wifi_transports(ROVER_WIFI_SSID, ROVER_WIFI_PASS, rover_agent_ip, ROVER_AGENT_PORT);
  WiFi.setSleep(WIFI_PS_NONE);
  Serial.printf("WiFi -> server agent %s:%d  ESP IP=%s\n", ROVER_AGENT_IP, ROVER_AGENT_PORT,
                WiFi.localIP().toString().c_str());
  rover_diag_wifi_ready();
#ifdef ROVER_TDISPLAY_S3
  ota.beginConnected("rover-esp");
#endif
#else
  set_microros_serial_transports(PiSerial);
  Serial.printf("Waiting for micro-ROS agent on UART (RX=%d TX=%d @%u)...\n", PI_UART_RX,
                PI_UART_TX, (unsigned)PI_UART_BAUD);
#endif
  status_led.set_state(LedState::kWaitAgent);
  delay(1000);
}

void loop() {
  const uint32_t now = millis();
  static uint32_t prev = now;
  const float dt = (now - prev) / 1000.0f;
  prev = now;

#ifdef ROVER_TDISPLAY_S3
  estop.poll(now);
  go_button.poll(now);
  battery.update(now);
  ir_beacon.tick();
  ota.tick();
  if (ota.busy()) {
    // Flash writes + micro-ROS/display in the same loop starve ArduinoOTA.
    while (ota.busy()) {
      spin_ros_input();
      ota.tick();
      yield();
    }
    return;
  }
  if (estop.active() || battery.critical()) {
    force_stop(estop.active() ? "E-STOP" : "low battery");
  }

  const bool menu_open = mode_menu_open || power_menu_open;
  go_button.set_multi_tap_enabled(!menu_open && !pi_session_sub_value);
  const RoverButtonEvent go_ev = go_button.take_event();
  handle_go_button_event(go_ev, now);

  const RoverButtonEvent estop_ev = estop.take_event();
  if (estop_ev == kBtnEstopLong) {
    flash_key_feedback(now);
    mode_menu_open = false;
    power_menu_open = false;
    publish_button_event(kBtnEstopLong);
  }
#endif

  handle_ros_state();

#if defined(ROVER_MCP_IR)
  poll_mcp_ir(now);
#endif

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  const bool imu_ok = mpu.ok() && mpu.read(ax, ay, az, gx, gy, gz);
  const float yaw_rate = imu_ok ? imu_yaw_rate(gx, gy, gz) : 0.0f;
  if (imu_ok) {
    heading.update(yaw_rate, dt, imu_stationary());
    last_imu_yaw_deg = heading.angle_deg_wrapped();
    last_imu_gz_dps = heading.rate_dps();
    last_imu_az_g = az;
    if (!imu_rest_logged && heading.ready() && imu_stationary()) {
      imu_rest_logged = true;
      rover_diag_event("mpu_rest ax=%.2f ay=%.2f az=%.2f bias=%.4f gz=%.2f", ax, ay, az,
                       heading.bias_rad_s(), heading.rate_dps());
    }
  }
  const float yaw_deg = imu_ok ? heading.angle_deg_wrapped() : 0.0f;

#if defined(ROVER_VL53L)
  static uint32_t last_tof_poll = 0;
  if (side_tof.ok() && now - last_tof_poll >= 100) {
    last_tof_poll = now;
    side_tof.poll();
  }
#endif

  if (ros_state == RosState::kConnected) {
#if defined(ROVER_SONAR)
    {
      constexpr uint32_t CMD_TIMEOUT_MS = 400;
      const bool cmd_fresh = (now - last_drive_ms) <= CMD_TIMEOUT_MS;
      float s_lin = cmd_fresh ? last_lin : 0.0f;
      float s_ang = cmd_fresh ? last_ang : 0.0f;
      if (sonar_scan_hold()) {
        s_lin = 0.0f;
        s_ang = 0.0f;
      }
      const bool body_moving =
          fabsf(drv.cur_left()) > 0.04f || fabsf(drv.cur_right()) > 0.04f;
      sonar_drive_override =
          rover_sonar.tick(now, s_lin, s_ang, body_moving, yaw_deg, &s_lin, &s_ang);
      sonar_drive_lin = s_lin;
      sonar_drive_ang = s_ang;
    }
#endif

#ifdef ROVER_TDISPLAY_S3
    if (now < button_priority_until) {
      for (int i = 0; i < 12; i++) {
        spin_ros_input();
      }
    } else {
      spin_ros_input();
    }
#else
    spin_ros_input();
#endif

    apply_drive(yaw_rate, dt);

#if defined(ROVER_SONAR)
    static uint32_t last_sonar_pub = 0;
    if (now - last_sonar_pub >= 100) {
      last_sonar_pub = now;
      const float rng = rover_sonar.range_m();
      sonar_range_msg.range = (rng > 0.0f) ? rng : std::numeric_limits<float>::quiet_NaN();
      sonar_range_msg.header.stamp.sec = static_cast<int32_t>(now / 1000);
      sonar_range_msg.header.stamp.nanosec = static_cast<uint32_t>((now % 1000) * 1000000UL);
      RCSOFTCHECK(rcl_publish(&sonar_range_pub, &sonar_range_msg, NULL));
      sonar_pan_msg.data = rover_sonar.pan_deg();
      RCSOFTCHECK(rcl_publish(&sonar_pan_pub, &sonar_pan_msg, NULL));
      sonar_avoid_msg.data = rover_sonar.avoid_active();
      RCSOFTCHECK(rcl_publish(&sonar_avoid_pub, &sonar_avoid_msg, NULL));
    }
    uint8_t escape_id = 0;
    if (rover_sonar.take_escape_event(&escape_id)) {
      sonar_escape_event_msg.data = escape_id;
      sonar_escape_turn_msg.data = rover_sonar.escape_turn_sign();
      sonar_escape_best_deg_msg.data = rover_sonar.escape_best_deg();
      sonar_escape_best_m_msg.data = rover_sonar.escape_best_m();
      RCSOFTCHECK(rcl_publish(&sonar_escape_event_pub, &sonar_escape_event_msg, NULL));
      RCSOFTCHECK(rcl_publish(&sonar_escape_turn_pub, &sonar_escape_turn_msg, NULL));
      RCSOFTCHECK(rcl_publish(&sonar_escape_best_deg_pub, &sonar_escape_best_deg_msg, NULL));
      RCSOFTCHECK(rcl_publish(&sonar_escape_best_m_pub, &sonar_escape_best_m_msg, NULL));
#if defined(ROVER_PERIPH)
      rover_periph.notify_escape();
#endif
    }
#endif

#if defined(ROVER_VL53L)
    static uint32_t last_tof_pub = 0;
    if (side_tof.ok() && now - last_tof_pub >= 100) {
      last_tof_pub = now;
      const int32_t sec = static_cast<int32_t>(now / 1000);
      const uint32_t nsec = static_cast<uint32_t>((now % 1000) * 1000000UL);
      bool okL = side_tof.left_ok();
      bool okR = side_tof.right_ok();
      float mL = okL ? side_tof.range_left_m() : -1.0f;
      float mR = okR ? side_tof.range_right_m() : -1.0f;
#if ROVER_VL53_SWAP_LR
      const bool t_ok = okL;
      okL = okR;
      okR = t_ok;
      const float t_m = mL;
      mL = mR;
      mR = t_m;
#endif
      if (okL) {
        const float rng = mL;
        tof_left_msg.range = (rng > 0.0f) ? rng : std::numeric_limits<float>::quiet_NaN();
        tof_left_msg.header.stamp.sec = sec;
        tof_left_msg.header.stamp.nanosec = nsec;
        RCSOFTCHECK(rcl_publish(&tof_left_pub, &tof_left_msg, NULL));
      }
      if (okR) {
        const float rng = mR;
        tof_right_msg.range = (rng > 0.0f) ? rng : std::numeric_limits<float>::quiet_NaN();
        tof_right_msg.header.stamp.sec = sec;
        tof_right_msg.header.stamp.nanosec = nsec;
        RCSOFTCHECK(rcl_publish(&tof_right_pub, &tof_right_msg, NULL));
      }
    }
#endif

#ifdef ROVER_TDISPLAY_S3
    const bool rover_active =
        pi_session_sub_value || (millis() - last_drive_ms < 1000);
#else
    const bool rover_active = true;
#endif

    static uint32_t last_imu_pub = 0;
    constexpr uint32_t CMD_TIMEOUT_MS = 400;
    const bool driving = (millis() - last_drive_ms) <= CMD_TIMEOUT_MS;
    const uint32_t imu_period = driving ? 25 : 250;
#ifdef ROVER_TDISPLAY_S3
    const bool imu_clear = now >= button_priority_until;
#else
    const bool imu_clear = true;
#endif

    if (imu_ok && rover_active && imu_clear && (now - last_imu_pub >= imu_period)) {
      last_imu_pub = now;
      imu_msg.linear_acceleration.x = ax * 9.80665f;
      imu_msg.linear_acceleration.y = ay * 9.80665f;
      imu_msg.linear_acceleration.z = az * 9.80665f;
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = heading.rate_rad_s();
      RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

      const float motor_l = fmaxf(fabsf(drv.tgt_left()), fabsf(drv.cur_left()));
      const float motor_r = fmaxf(fabsf(drv.tgt_right()), fabsf(drv.cur_right()));
      motion.update(ax, ay, az, gx, gy, gz, motor_l, motor_r,
                    mcp_ir.wheel_left_ticks(), mcp_ir.wheel_right_ticks(), now);
      if (motion.bump()) {
        publish_bump_event("IMU jerk");
      }
      if (motion.stall()) {
        stall_msg.data = true;
        RCSOFTCHECK(rcl_publish(&stall_pub, &stall_msg, NULL));
        status_led.trigger(LedEvent::kStall);
        Serial.println("EVENT stall — motors cut locally");
        force_stop("stall");
#ifdef ROVER_TDISPLAY_S3
        ui.stall = true;
        stall_block_until = now + 2500;
#if defined(ROVER_PERIPH)
        rover_periph.notify_stall();
#endif
#endif
      }
      motion.clear_events();
    }
  } else {
    drv.tick();
  }

  static uint32_t last_hb = 0;
  if (now - last_hb > 2000) {
    last_hb = now;
    const char *st = (ros_state == RosState::kConnected) ? "up"
                   : (ros_state == RosState::kWaitAgent) ? "wait"
                                                         : "link";
    Serial.printf("hb [%s] lin=%.2f ang=%.2f L=%.2f R=%.2f", st, last_lin, last_ang,
                  drv.cur_left(), drv.cur_right());
#ifdef ROVER_TDISPLAY_S3
    Serial.printf(" bat=%.2fV", battery.voltage());
    if (estop.active()) Serial.print(" ESTOP");
    Serial.printf(" pi=%lus", last_pi_traffic_ms ? (unsigned long)((now - last_pi_traffic_ms) / 1000) : 999UL);
#endif
    Serial.println();
  }

  if (ros_state == RosState::kConnected) {
    drv.tick();
  }

#if defined(ROVER_SONAR)
  {
    static uint8_t last_snap_count = 0;
    const uint8_t snaps = rover_sonar.cal_snap_count();
    if (snaps > last_snap_count) {
      const uint8_t i = snaps - 1;
      rover_diag_event("cal_snap pan=%.0f rng=%.2f", rover_sonar.cal_snap_pan(i),
                       rover_sonar.cal_snap_range(i));
      last_snap_count = snaps;
    } else if (snaps < last_snap_count) {
      last_snap_count = snaps;
    }
  }
#endif

#if defined(ROVER_SONAR)
  update_session_cal_sweep(now);
#endif

#if defined(ROVER_PERIPH)
  rover_periph.tick(now);
#if defined(ROVER_SONAR)
  if (pi_session_sub_value || rover_sonar.cal_sweep_active()) {
    // Side ToF overlay (left/right) onto the ring.
#if defined(ROVER_VL53L)
    if (side_tof.ok()) {
      bool okL = side_tof.left_ok();
      bool okR = side_tof.right_ok();
      float l = okL ? side_tof.range_left_m() : -1.0f;
      float r = okR ? side_tof.range_right_m() : -1.0f;
#if ROVER_VL53_SWAP_LR
      const float t_m = l;
      l = r;
      r = t_m;
#endif
      rover_periph.note_side_tof(now, l, r);
    }
#endif
    // IR overlay: front bumper hits + rear aux sweep samples.
#if defined(ROVER_MCP_IR)
    if (mcp_ir.front_ok()) {
      rover_periph.tick_ir_map(now, mcp_ir.front_left_hit(), mcp_ir.front_right_hit());
    }
    const bool want_ir_sweep = rover_sonar.cal_sweep_active() && mcp_ir.body_ok() && pca9685.ok();
    if (want_ir_sweep) {
      // Sweep rear aux IR over 0..180 while sonar scan is active.
      static constexpr float kAngles[] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 135.0f, 90.0f, 45.0f};
      static constexpr uint8_t kCount = sizeof(kAngles) / sizeof(kAngles[0]);
      static constexpr uint32_t kSettleMs = 120;
      static constexpr uint32_t kBetweenMs = 40;

      if (ir_sweep_step_ms == 0) {
        ir_sweep_step_ms = now;
        ir_sweep_step = 0;
        ir_sweep_waiting = false;
      }
      if (!ir_sweep_waiting) {
        ir_sweep_deg = kAngles[ir_sweep_step % kCount];
        pca9685.setAngle(ROVER_AUX_SERVO_CHANNEL, ir_sweep_deg, 0.0f, 180.0f, 500, 2500);
        ir_sweep_step_ms = now;
        ir_sweep_waiting = true;
      } else if (now - ir_sweep_step_ms >= kSettleMs) {
        bool off_hit = false;
        bool on_hit = false;
        mcp_ir.sample_aux(MCP_IR_AUX_OFF_MS, MCP_IR_AUX_ON_MS, off_hit, on_hit);
        const bool hit = on_hit && !off_hit;
        rover_periph.note_ir_rear_sample(now, ir_sweep_deg, hit);
        ir_sweep_step++;
        ir_sweep_step_ms = now + kBetweenMs;
        ir_sweep_waiting = false;
      }
    } else {
      ir_sweep_step_ms = 0;
      ir_sweep_waiting = false;
      ir_sweep_step = 0;
      ir_sweep_deg = 90.0f;
    }
#endif
    rover_periph.tick_sonar_map(now, rover_sonar.pan_deg(), rover_sonar.range_m(),
                                rover_sonar.cal_sweep_active());
  }
#endif
#endif

  update_status_led(now);

#ifdef ROVER_TDISPLAY_S3
  ui.bat_v = battery.voltage();
  ui.bat_low = battery.low();
  ui.bat_critical = battery.critical();
  ui.ax = ax;
  ui.ay = ay;
  ui.az = az;
  ui.yaw_rate = yaw_rate;
  ui.cmd_lin = last_lin;
  ui.cmd_ang = last_ang;
  ui.motor_l = drv.cur_left();
  ui.motor_r = drv.cur_right();
  ui.link_state = link_state_code();
  ui.estop = estop.active();
  ui.mode_menu = mode_menu_open;
  ui.power_menu = power_menu_open;
  ui.drive_mode = static_cast<uint8_t>(selected_drive_mode);
  ui.power_action = static_cast<uint8_t>(selected_power_action);
  ui.menu_hint_until_ms = menu_saved_flash_until;
  ui.key_flash_until_ms = key_flash_until;
  ui.tap_progress = go_button.active_tap_count();
  update_pi_bridge_state(now);
  if (pi_session_sub_value) {
    mode_menu_open = false;
    power_menu_open = false;
  }
#if defined(ENABLE_OTA)
  ui.ota_state = static_cast<int>(ota.state());
  ui.wifi_ip = ota.ip();
#endif
#if defined(ROVER_MCP_IR)
  ui.ir_enabled = true;
  ui.ir_sda_pin = MPU_SDA;
  ui.ir_scl_pin = MPU_SCL;
  ui.ir_ok = mcp_ir.ok();
  ui.ir_in_ok = mcp_ir.front_ok();
  ui.ir_out_ok = mcp_ir.body_ok();
  ui.ir_inputs = mcp_ir.read_inputs();
  ui.ir_front_hit = now < ir_front_flash_until;
  ui.ir_wheels_on = ir_wheels_on;
  ui.ir_front_emit = mcp_ir.front_emitting();
  ui.ir_body_aux_emit = mcp_ir.body_aux_emitting();
  mcp_ir.readFrontIoState(ui.ir_front_iodir, ui.ir_front_olat, ui.ir_front_gpio);
  mcp_ir.readBodyIoState(ui.ir_body_iodir, ui.ir_body_olat, ui.ir_body_gpio);
  ui.ir_i2c_count = ir_i2c_scan.count;
  for (uint8_t i = 0; i < 8; i++) {
    ui.ir_i2c_addrs[i] = (i < ir_i2c_scan.count) ? ir_i2c_scan.addrs[i] : 0;
  }
#endif
#if defined(ROVER_SONAR)
  ui.sonar_enabled = true;
  ui.sonar_pan_deg = rover_sonar.pan_deg();
  ui.sonar_range_m = rover_sonar.range_m();
  ui.sonar_cal_active = rover_sonar.cal_sweep_active();
  ui.sonar_cal_snap_count = rover_sonar.cal_snap_count();
  for (uint8_t i = 0; i < 4; i++) {
    if (i < ui.sonar_cal_snap_count) {
      ui.sonar_cal_snap_pan[i] = rover_sonar.cal_snap_pan(i);
      ui.sonar_cal_snap_range[i] = rover_sonar.cal_snap_range(i);
    } else {
      ui.sonar_cal_snap_pan[i] = 0.0f;
      ui.sonar_cal_snap_range[i] = -1.0f;
    }
  }
  ui.sonar_cal_summary_until_ms = sonar_cal_summary_until_ms;
#endif
#if defined(ROVER_VL53L)
  ui.tof_enabled = true;
  ui.tof_left_ok = side_tof.left_ok();
  ui.tof_right_ok = side_tof.right_ok();
  ui.tof_left_m = side_tof.range_left_m();
  ui.tof_right_m = side_tof.range_right_m();
#endif
  ui.boot_reason = boot_reason_buf[0] ? boot_reason_buf : nullptr;
  ui.boot_reason_until_ms = boot_reason_until_ms;
#if defined(ROVER_SONAR)
  if (ui_note_buf[0] && ui_note_until_ms > now) {
    ui.boot_reason = ui_note_buf;
    ui.boot_reason_until_ms = ui_note_until_ms;
  }
#endif
  if (estop.active()) {
    ui.mode = "E-STOP";
  } else if (battery.critical()) {
    ui.mode = "low batt";
  } else if (ros_state == RosState::kConnected) {
    if (power_menu_open) {
      ui.mode = "Power menu";
    } else if (mode_menu_open && !ui.session_active) {
      ui.mode = "Mode menu";
    } else if (!ui.session_active) {
      ui.mode = "Standby";
    } else if (ui.sonar_cal_active) {
      ui.mode = "SONAR SCAN";
    } else if (ui.sonar_cal_summary_until_ms > now) {
      ui.mode = "SCAN DONE";
    } else {
      constexpr uint32_t CMD_TIMEOUT_MS = 400;
      if (millis() - last_drive_ms > CMD_TIMEOUT_MS) {
        ui.mode = "Explore - waiting...";
      } else {
        ui.mode = "Explore";
      }
    }
  } else {
    ui.mode = "idle";
  }
  display.draw(ui, now);

#if defined(ROVER_SONAR) || defined(ROVER_VL53L)
  {
    RoverDiagSample diag{};
    diag.now_ms = now;
    diag.link_state = link_state_code();
    diag.session = pi_session_sub_value;
#if defined(ROVER_SONAR)
    diag.cal_active = rover_sonar.cal_sweep_active();
    diag.cal_step = rover_sonar.cal_step();
    diag.cal_snaps = rover_sonar.cal_snap_count();
    diag.pan_deg = rover_sonar.pan_deg();
    diag.range_m = rover_sonar.range_m();
#else
    diag.cal_active = false;
    diag.pan_deg = 90.0f;
    diag.range_m = -1.0f;
#endif
    diag.cmd_lin = last_lin;
    diag.cmd_ang = last_ang;
    diag.motor_l = drv.cur_left();
    diag.motor_r = drv.cur_right();
    diag.bat_v = battery.voltage();
    diag.yaw_deg = last_imu_yaw_deg;
    diag.gz_dps = last_imu_gz_dps;
    diag.htrim = last_heading_trim;
    diag.az_g = last_imu_az_g;
#if defined(ROVER_VL53L)
    diag.tof_l_m = side_tof.left_ok() ? side_tof.range_left_m() : -1.0f;
    diag.tof_r_m = side_tof.right_ok() ? side_tof.range_right_m() : -1.0f;
#endif
    rover_diag_tick(diag);
  }
#endif

  ui.bump = false;
  ui.stall = false;

  uint32_t loop_delay_ms = 20;
  if (pi_session_sub_value && (now - last_drive_ms) < 500) {
    loop_delay_ms = 5;
  } else if (ros_state == RosState::kConnected) {
    loop_delay_ms = 10;
  }
  delay(loop_delay_ms);
#else
  delay(5);
#endif
}
