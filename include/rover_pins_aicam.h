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

// Audio, per DFRobot DFR1154 wiki/examples (Recording & Playback, Play Online Music):
//   PDM mic: CLK GPIO38, DATA GPIO39
//   MAX98357 I2S amp -> SPK (MX1.25-2P, 4R 1.5W / 8R 1W): BCLK GPIO45, LRC/WS GPIO46, DOUT GPIO42
// (Earlier HA note "mic CLK 46/45" was wrong: 45/46 are the amp's I2S clocks.)
#define AICAM_MIC_PDM_CLK 38
#define AICAM_MIC_PDM_DATA 39
#define AICAM_SPK_I2S_BCLK 45
#define AICAM_SPK_I2S_LRC 46
#define AICAM_SPK_I2S_DOUT 42
