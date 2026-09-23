#pragma once

// LilyGO T-Display-S3 (non-touch) — match board silkscreen / header labels.
// https://github.com/Xinyuan-LilyGO/T-Display-S3
//
// Motor naming: A = LEFT, B = RIGHT. DIR high + PWM = forward (TC78H660).

#ifndef PIN_POWER_ON
#define PIN_POWER_ON 15
#endif
#ifndef PIN_LCD_BL
#define PIN_LCD_BL 38
#endif
#ifndef PIN_BUTTON_ESTOP
#define PIN_BUTTON_ESTOP 0
#endif
#ifndef PIN_BUTTON_GO
#define PIN_BUTTON_GO 14
#endif

// VM divider tap — GPIO16 (header). Not GPIO4 (not on this board's header).
#ifndef PIN_BAT_ADC
#define PIN_BAT_ADC 16
#endif

#ifndef DIR_L
#define DIR_L 1   // motor A (left) direction
#define PWM_L 2   // motor A PWM
#define DIR_R 3   // motor B (right) direction
#define PWM_R 10  // motor B PWM
#endif

// MPU6050 — header I2C (silkscreen IO43 / IO44)
#ifndef MPU_SDA
#define MPU_SDA 43
#define MPU_SCL 44
#endif

// Speaker + WS2812 ring (ex-Pi UART header wires — see WIRING.md).
#ifndef PIN_ROVER_SPEAKER
#define PIN_ROVER_SPEAKER 17
#endif
#ifndef PIN_ROVER_RING
#define PIN_ROVER_RING 18
#endif
#ifndef ROVER_RING_COUNT
#define ROVER_RING_COUNT 8
#endif
#ifndef ROVER_RING_BRIGHTNESS
#define ROVER_RING_BRIGHTNESS 24
#endif
#ifndef ROVER_RING_ZERO_BEARING
#define ROVER_RING_ZERO_BEARING 270.0f
#endif

// Pi UART (legacy serial micro-ROS only — not used on WiFi rover).
#ifndef PI_UART_PORT
#define PI_UART_PORT 1
#define PI_UART_RX 17
#define PI_UART_TX 18
#endif
#ifndef PI_UART_BAUD
#define PI_UART_BAUD 460800
#endif
#ifndef PI_PERIPH_BAUD
#define PI_PERIPH_BAUD 115200
#endif

// Body MCP23008 @ 0x21 (A0 → 3.3 V). Wheel IR + ToF enables.
#ifndef MCP_BODY_ADDR
#define MCP_BODY_ADDR 0x21
#endif

#ifndef BODY_IN_SPARE
#define BODY_IN_SPARE 0    // GPA0 unused
#define BODY_IN_WHEEL_L 1  // GPA1 ← wheel left Schmitt
#define BODY_IN_WHEEL_R 2  // GPA2 ← wheel right Schmitt
#define BODY_OUT_VL53_L8 3 // GPA3 → VL53L8CX LPn (HIGH = awake)
#define BODY_OUT_WHEEL_L 4 // GPA4 → wheel left emitter (BC557 PNP, LOW = on)
#define BODY_OUT_WHEEL_R 5 // GPA5 → wheel right emitter
#define BODY_OUT_VL53_L 6  // GPA6 → left VL53L1X XSHUT
#define BODY_OUT_VL53_R 7  // GPA7 → right VL53L1X XSHUT
#endif

#ifndef SONAR_SERVO_CHANNEL
// Rear pan sonar on PCA9685 ch 0. Bracket rotated 180°: 90° = aft.
#define SONAR_SERVO_CHANNEL 0
#endif

#ifndef SONAR_GLANCE_MAG_DEG
// Rear + sides: ±70° around aft (logical 90°). Not a tiny front wiggle.
#define SONAR_GLANCE_MAG_DEG 70.0f
#endif
#ifndef SONAR_GLANCE_PERIOD_MS
#define SONAR_GLANCE_PERIOD_MS 3600
#endif
#ifndef SONAR_WIGGLE_HOLD_M
// Hold pan at center (no wiggle) when forward range below this.
#define SONAR_WIGGLE_HOLD_M 0.42f
#endif
#ifndef SONAR_FORWARD_READ_MS
#define SONAR_FORWARD_READ_MS 200
#endif
#ifndef SONAR_CLOSE_TREND_M
#define SONAR_CLOSE_TREND_M 0.04f
#endif
#ifndef SONAR_PROBE_SETTLE_MS
#define SONAR_PROBE_SETTLE_MS 70
#endif
#ifndef SONAR_PROBE_DODGE_MS
#define SONAR_PROBE_DODGE_MS 480
#endif
#ifndef SONAR_ESCAPE_DELAY_MS
#define SONAR_ESCAPE_DELAY_MS 750
#endif
#ifndef SONAR_ESCAPE_DELAY_CLOSING_MS
#define SONAR_ESCAPE_DELAY_CLOSING_MS 120
#endif
#ifndef SONAR_REVERSE_MS
#define SONAR_REVERSE_MS 1000
#endif
#ifndef SONAR_REVERSE_MAX_MS
#define SONAR_REVERSE_MAX_MS 1600
#endif
#ifndef SONAR_REVERSE_CLEAR_M
#define SONAR_REVERSE_CLEAR_M 0.40f
#endif
#ifndef SONAR_ESCAPE_MIN_M
#define SONAR_ESCAPE_MIN_M 0.35f
#endif
#ifndef SONAR_DRIVE_OUT_CLEAR_M
#define SONAR_DRIVE_OUT_CLEAR_M 0.42f
#endif
#ifndef SONAR_TURN_MIN_DEG
#define SONAR_TURN_MIN_DEG 35.0f
#endif
#ifndef SONAR_TURN_BOXED_DEG
#define SONAR_TURN_BOXED_DEG 55.0f
#endif
#ifndef SONAR_PAN_TRIM_DEG
// Fine offset if mechanical center drifts (0 = centered on horn). + = pan left.
// Was +5 (biased left) — bench sweep was visibly further left than right, so
// flipped to -5 (pan right) to bring the center back to true forward.
#define SONAR_PAN_TRIM_DEG -5.0f
#endif
#ifndef SONAR_PAN_INVERT
// Bracket rotated 180° (was nose, now tail). Swap 0↔180 so logical left/right match chassis.
#define SONAR_PAN_INVERT 1
#endif
#ifndef ROVER_L8_SWAP_LR
// Nose L8CX halves: col 0–3 vs 4–7. 1 = swap if peel always goes the wrong way.
#define ROVER_L8_SWAP_LR 1
#endif
// Distance from sensor face to rear bumper (m). Same order as the old front offset.
#ifndef SONAR_TO_BUMPER_M
#define SONAR_TO_BUMPER_M 0.09f
#endif
#ifndef SONAR_AVOID_M
// Slow / trend threshold (sonar reading, not bumper distance).
#define SONAR_AVOID_M 0.38f
#endif
#ifndef SONAR_STOP_M
// Hard stop reverse (aft cone). ~15 cm to rear bumper with 9 cm offset.
#define SONAR_STOP_M 0.24f
#endif
#ifndef SONAR_SPIN_STOP_M
// Zero angular when this close while driving (prevents wall-hugging spin).
#define SONAR_SPIN_STOP_M 0.30f
#endif
#ifndef SONAR_CLOSE_ANY_M
// Any pan angle — treat as emergency reverse stop when backing.
#define SONAR_CLOSE_ANY_M 0.20f
#endif
#ifndef SONAR_REVERSE_M
#define SONAR_REVERSE_M 0.08f
#endif

// HC-SR04 on pan bracket (header GPIO11/12).
#ifndef SONAR_TRIG_PIN
#define SONAR_TRIG_PIN 11
#endif
#ifndef SONAR_ECHO_PIN
#define SONAR_ECHO_PIN 12
#endif
#ifndef SONAR_PAN_CENTER_DEG
#define SONAR_PAN_CENTER_DEG 90
#endif
#ifndef SONAR_PAN_MIN_DEG
#define SONAR_PAN_MIN_DEG 0
#endif
#ifndef SONAR_PAN_MAX_DEG
#define SONAR_PAN_MAX_DEG 180
#endif
#ifndef SONAR_SCAN_STEP_DEG
#define SONAR_SCAN_STEP_DEG 30
#endif

// Explore cruise vs sonar-escape burst (normalized motor command 0..1).
#ifndef ROVER_CRUISE_MAX_LIN
#define ROVER_CRUISE_MAX_LIN 0.15f
#endif
#ifndef STALL_TICKS_REQUIRED
#define STALL_TICKS_REQUIRED 1
#endif
#ifndef STALL_EXPECTED_TICKS_CRUISE
// Wheel IR ticks in STALL_TICK_WINDOW at ROVER_CRUISE_MAX_LIN on hard floor.
#define STALL_EXPECTED_TICKS_CRUISE 10
#endif
#ifndef STALL_SLIP_RATIO
// Stall when tick rate falls below this fraction of motor-scaled expectation.
#define STALL_SLIP_RATIO 0.12f
#endif
#ifndef STALL_TICK_WINDOW_CRUISE_MS
#define STALL_TICK_WINDOW_CRUISE_MS 1500
#endif
#ifndef STALL_ARM_MS
#define STALL_ARM_MS 500
#endif
#ifndef STALL_COOLDOWN_MS
#define STALL_COOLDOWN_MS 2000
#endif

// IMU horizontal jerk bump (g) while motors are engaged.
#ifndef BUMP_JERK_G
#define BUMP_JERK_G 0.38f
#endif
#ifndef BUMP_COOLDOWN_MS
#define BUMP_COOLDOWN_MS 1200
#endif
#ifndef BUMP_LP_ALPHA
#define BUMP_LP_ALPHA 0.86f
#endif
#ifndef BUMP_GYRO_MAX
#define BUMP_GYRO_MAX 2.2f
#endif
#ifndef WHEEL_STALL_MOTOR
#define WHEEL_STALL_MOTOR 0.08f
#endif
#ifndef ROVER_BURST_MAX_LIN
#define ROVER_BURST_MAX_LIN 0.40f
#endif
#ifndef ROVER_BOOT_SPIN_LIN
#define ROVER_BOOT_SPIN_LIN 0.10f
#endif
#ifndef ROVER_BOOT_SPIN_TRICK
#define ROVER_BOOT_SPIN_TRICK 0
#endif
#ifndef ROVER_BOOT_PAN_SWEEP
// 1 = full left/right/center sweep at boot (bench). 0 = quiet center only.
#define ROVER_BOOT_PAN_SWEEP 0
#endif
#ifndef ROVER_TELEM_HOST
#define ROVER_TELEM_HOST "192.168.2.31"
#endif
#ifndef ROVER_TELEM_PORT
#define ROVER_TELEM_PORT 4243
#endif
#ifndef SONAR_BOOT_PAN_MS
#define SONAR_BOOT_PAN_MS 280
#endif

#ifndef PCA9685_ADDR
#define PCA9685_ADDR 0x40
#endif

// VL53L1X side ToF — XSHUT on body MCP GPA6/GPA7. Shared I2C 43/44.
#ifndef VL53L_ADDR_L
#define VL53L_ADDR_L 0x30
#endif
#ifndef VL53L_ADDR_R
#define VL53L_ADDR_R 0x31
#endif

// Some builds have the physical left/right VL53 modules (or XSHUT wires) swapped.
// When set to 1, firmware swaps published /rover/tof/left and /rover/tof/right (and ring overlay).
#ifndef ROVER_VL53_SWAP_LR
#define ROVER_VL53_SWAP_LR 0
#endif
#ifndef L8_STOP_M
#define L8_STOP_M 0.24f
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN (-1)
#endif

// 38 kHz IR beacon for TSOP chase (GPIO21 header spare — was shift-register latch).
// GPIO21 → 1 kΩ → BC547 base; IR LEDs + resistor from 5 V → collector; emitter → GND.
#ifndef IR_TX_PIN
#define IR_TX_PIN 21
#endif
#ifndef IR_TX_HZ
#define IR_TX_HZ 38000
#endif
#ifndef IR_TX_LEDC_CHANNEL
#define IR_TX_LEDC_CHANNEL 4
#endif
#ifndef IR_TX_DEFAULT_ON
#define IR_TX_DEFAULT_ON 1
#endif
#ifndef IR_TX_MODULATE
#define IR_TX_MODULATE 1
#endif
#ifndef IR_TX_BURST_US
#define IR_TX_BURST_US 560
#endif
#ifndef IR_TX_GAP_US
#define IR_TX_GAP_US 560
#endif

// 2S pack on VM → GPIO16 divider (200k + 100k nominal, ratio 3.0).
// DMM cal 2026-07 (switch ON): pack 7.28 V, tap 2.64 V, TFT was 7.6 @ 2.76 → 2.64.
#ifndef BAT_ADC_SCALE
#define BAT_ADC_SCALE 2.64f
#endif
#ifndef BAT_VOLT_LOW
#define BAT_VOLT_LOW 6.4f
#endif
#ifndef BAT_VOLT_CRITICAL
#define BAT_VOLT_CRITICAL 6.0f
#endif
