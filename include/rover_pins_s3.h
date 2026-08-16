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

// Pi micro-ROS UART — Serial1 per LilyGO SerialExample (RX=17, TX=18)
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

// ── MCP23008 split layout ────────────────────────────────────────────────
// Front bumper PCB (ribbon to ESP I2C bus): one MCP, mixed inputs + outputs.
// ESP breadboard: second MCP for aux + wheel channels.
//
// Strap A2 A1 A0 → address (all GND except A0 on second chip is common pattern):
//   Front board MCP: 0x20 (000)
//   Body board MCP:  0x21 (001) — A0 → 3.3 V

#ifndef MCP_FRONT_ADDR
#define MCP_FRONT_ADDR 0x20
#endif
#ifndef MCP_BODY_ADDR
#define MCP_BODY_ADDR 0x21
#endif

// Front bumper MCP @ MCP_FRONT_ADDR — LM358 outs + BC547 emitter gates
#ifndef FRONT_IN_L
#define FRONT_IN_L 0   // GPA0 ← LM358 ch A OUT
#define FRONT_IN_R 1   // GPA1 ← LM358 ch B OUT
#define FRONT_OUT_L 2  // GPA2 → BC547 gate (steady ON; 555 modulates on bumper PCB)
#define FRONT_OUT_R 3  // GPA3 → BC547 gate (steady ON; 555 modulates on bumper PCB)
#endif

// Body MCP @ MCP_BODY_ADDR — GPA0-2 Schmitt IN, GPA3-5 BC557 emitter OUT
#ifndef BODY_IN_AUX
#define BODY_IN_AUX 0    // GPA0 ← aux Schmitt OUT (INPUT)
#define BODY_IN_WHEEL_L 1  // GPA1 ← wheel left Schmitt OUT (INPUT)
#define BODY_IN_WHEEL_R 2  // GPA2 ← wheel right Schmitt OUT (INPUT)
#define BODY_OUT_AUX 3     // GPA3 → aux IR gate (OUTPUT, BC557 PNP, MCP low = on)
#define BODY_OUT_WHEEL_L 4 // GPA4 → wheel left emitter (OUTPUT, BC557 PNP)
#define BODY_OUT_WHEEL_R 5 // GPA5 → wheel right emitter (OUTPUT, BC557 PNP)
#endif

// Legacy aliases (old dual-chip in/out @ same GPA index on 0x20/0x21)
#define MCP_IR_IN_ADDR MCP_FRONT_ADDR
#define MCP_IR_OUT_ADDR MCP_BODY_ADDR
#define MCP_IN_FRONT_L FRONT_IN_L
#define MCP_IN_FRONT_R FRONT_IN_R
#define MCP_OUT_FRONT_L FRONT_OUT_L
#define MCP_OUT_FRONT_R FRONT_OUT_R
#define MCP_IN_WHEEL_L BODY_IN_WHEEL_L
#define MCP_IN_WHEEL_R BODY_IN_WHEEL_R
#define MCP_IN_AUX BODY_IN_AUX
#define MCP_OUT_WHEEL_L BODY_OUT_WHEEL_L
#define MCP_OUT_WHEEL_R BODY_OUT_WHEEL_R
#define MCP_OUT_AUX BODY_OUT_AUX

#ifndef ROVER_AUX_SERVO_CHANNEL
#define ROVER_AUX_SERVO_CHANNEL 0
#endif

#ifndef SONAR_SERVO_CHANNEL
// Front pan sonar only — PCA9685 ch 15 (silkscreen “OUT16”). Aux rear = ch 0.
#define SONAR_SERVO_CHANNEL 15
#endif
#ifndef ROVER_SERVO_CHANNEL
#define ROVER_SERVO_CHANNEL SONAR_SERVO_CHANNEL
#endif

#ifndef SONAR_GLANCE_MAG_DEG
#define SONAR_GLANCE_MAG_DEG 12.0f
#endif
#ifndef SONAR_GLANCE_PERIOD_MS
#define SONAR_GLANCE_PERIOD_MS 2800
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
#define SONAR_PAN_TRIM_DEG 5.0f
#endif
#ifndef SONAR_PAN_INVERT
// 0 = normal mount. Set 1 only if horn is reversed on the spline (not a bracket flip).
#define SONAR_PAN_INVERT 0
#endif
#ifndef SONAR_AVOID_M
#define SONAR_AVOID_M 0.55f
#endif
#ifndef SONAR_STOP_M
#define SONAR_STOP_M 0.20f
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
#define STALL_TICKS_REQUIRED 2
#endif
#ifndef STALL_TICK_WINDOW_CRUISE_MS
#define STALL_TICK_WINDOW_CRUISE_MS 1400
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
#define ROVER_BOOT_PAN_SWEEP 1
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
