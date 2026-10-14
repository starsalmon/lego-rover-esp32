#pragma once

// Unexpected Maker Pro S3 — sensor board (L8 + side ToF + sonar + IR + PCA9685).
// Drive / IMU / motors stay on the T-Display. Do not reuse T-Display motor pins here.

#ifndef ROVER_PROS3
#define ROVER_PROS3 1
#endif

// UM Pro S3 second LDO (GPIO17 / LDO2). Must be HIGH before external 3V3 peripherals.
#ifndef PIN_PROS3_LDO2
#define PIN_PROS3_LDO2 17
#endif

// ProS3[D] RF switch (GPIO37): LOW=onboard ceramic, HIGH=external u.FL.
#ifndef PIN_PROS3_ANT_SEL
#define PIN_PROS3_ANT_SEL 37
#endif
// 1 = force u.FL, 0 = force onboard. Default 0 unless build flag sets it.
#ifndef ROVER_PROS3_ANT_EXTERNAL
#define ROVER_PROS3_ANT_EXTERNAL 0
#endif

// ---------------------------------------------------------------------------
// Bus A — L8 alone (aim ~800 kHz after firmware load; 2.2 kΩ pull-ups, short wires)
// ---------------------------------------------------------------------------
#ifndef L8_SDA
#define L8_SDA 8
#endif
#ifndef L8_SCL
#define L8_SCL 9
#endif
#ifndef L8_LPN_PIN
#define L8_LPN_PIN 4
#endif
#ifndef L8_INT_PIN
#define L8_INT_PIN 5  // optional; -1 to ignore
#endif
#ifndef L8_I2C_HZ
#define L8_I2C_HZ 400000
#endif
#ifndef L8_I2C_INIT_HZ
#define L8_I2C_INIT_HZ 400000
#endif

// ---------------------------------------------------------------------------
// Bus B — side VL53L1X + PCA9685
// ---------------------------------------------------------------------------
#ifndef SIDE_SDA
#define SIDE_SDA 1
#endif
#ifndef SIDE_SCL
#define SIDE_SCL 2
#endif
#ifndef SIDE_I2C_HZ
#define SIDE_I2C_HZ 400000
#endif
#ifndef VL53_XSHUT_L
#define VL53_XSHUT_L 6
#endif
#ifndef VL53_XSHUT_R
#define VL53_XSHUT_R 7
#endif
#ifndef VL53L_ADDR_L
#define VL53L_ADDR_L 0x30
#endif
#ifndef VL53L_ADDR_R
#define VL53L_ADDR_R 0x31
#endif
#ifndef PCA9685_ADDR
#define PCA9685_ADDR 0x40
#endif
#ifndef SONAR_SERVO_CHANNEL
#define SONAR_SERVO_CHANNEL 0
#endif

// ---------------------------------------------------------------------------
// Sonar (HC-SR04) — ECHO must be level-shifted to 3.3 V
// ---------------------------------------------------------------------------
#ifndef SONAR_TRIG_PIN
#define SONAR_TRIG_PIN 12
#endif
#ifndef SONAR_ECHO_PIN
#define SONAR_ECHO_PIN 13
#endif

// ---------------------------------------------------------------------------
// IR
// ---------------------------------------------------------------------------
#ifndef IR_RX_PIN
#define IR_RX_PIN 14
#endif
#ifndef IR_TX_PIN
#define IR_TX_PIN 21
#endif
#ifndef IR_TX_HZ
#define IR_TX_HZ 38000
#endif
#ifndef IR_TX_LEDC_CHANNEL
#define IR_TX_LEDC_CHANNEL 4
#endif

// Spares for later: 15, 16, 34–37
// Avoid: 0, 3, 10, 17 (LDO2), 18 (RGB), 19/20 (USB), 33 (VBUS), 45, 46

#ifndef ROVER_HOSTNAME
#define ROVER_HOSTNAME "rover-tof"
#endif
