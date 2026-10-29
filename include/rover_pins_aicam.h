#pragma once
// DFRobot ESP32-S3 AI Camera V1.1 (DFR1154) - OV3660 wide-angle, 16 MB flash, 8 MB octal PSRAM.
// Pin map taken from the working Home Assistant (ESPHome) config for this exact board.

// Camera (DVP). No PWDN / RESET lines on this board.
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 5
#define CAM_XCLK_HZ 20000000
#define CAM_PIN_SIOD 8  // SCCB shares the board I2C bus (LTR-308 light sensor @0x53)
#define CAM_PIN_SIOC 9
#define CAM_PIN_D0 16
#define CAM_PIN_D1 18
#define CAM_PIN_D2 21
#define CAM_PIN_D3 17
#define CAM_PIN_D4 14
#define CAM_PIN_D5 7
#define CAM_PIN_D6 6
#define CAM_PIN_D7 4
#define CAM_PIN_VSYNC 1
#define CAM_PIN_HREF 2
#define CAM_PIN_PCLK 15

// LEDs
#define AICAM_IR_LED_PIN 47      // built-in IR illuminators (plain GPIO, active high)
#define AICAM_STATUS_LED_PIN 3   // on-board status LED
#define AICAM_EXT_IR_PIN 44      // "External IR LEDs" PWM in the HA config (also U0RXD; unused here)

// Light sensor
#define AICAM_LTR308_ADDR 0x53

// Audio - defined for later, NOT initialised by the camera firmware.
// UNVERIFIED: the HA config only notes "PDM mic CLK GPIO46/45, DATA GPIO39".
// Check the DFRobot DFR1154 schematic before using these.
#define AICAM_MIC_PDM_CLK 46
#define AICAM_MIC_PDM_DATA 39
