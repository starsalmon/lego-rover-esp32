# Rover pin map

Three ESP32-S3 boards: T-Display (drive + TFT), Pro S3 (sensors + fleet IR), AI cam (MJPEG + I2S speaker). micro-ROS agent on `192.168.3.172:8888` (domain 0). Fleet IR id **0**.

## Boards

| Board | Role | Hostname / IP |
|-------|------|----------------|
| LilyGO T-Display-S3 | Motors, IMU, wheel ticks, TFT, WS2812 ring, piezo speaker, Go/E-stop | `rover-esp` / `192.168.3.71` |
| Unexpected Maker Pro S3[D] | VL53L8, side ToF, sonar servo, STL-19P lidar, fleet IR TX/RX | `rover-tof` / DHCP (`.local`) |
| DFRobot ESP32-S3 AI Camera V1.1 | OV3660 MJPEG, I2S speaker, greeting playback | `rover-cam` / DHCP (`.local`) |

## T-Display-S3 (drive)

| Board pin (silk) | GPIO | Function | Connects to | Notes |
|------------------|------|----------|-------------|--------|
| BOOT | 0 | E-stop input | Go/E-stop harness | Hold = motor cut on ESP |
| IO1 | 1 | Motor A DIR | TC78H660 DIR A (left) | DIR high + PWM = forward |
| IO2 | 2 | Motor A PWM | TC78H660 PWM A | 30 kHz |
| IO3 | 3 | Motor B DIR | TC78H660 DIR B (right) | |
| IO10 | 10 | Motor B PWM | TC78H660 PWM B | |
| IO11 | 11 | Wheel tick L | Reflective sensor (Schmitt) | Rising-edge ISR |
| IO12 | 12 | Wheel tick R | Reflective sensor (Schmitt) | Rising-edge ISR |
| KEY (top-right) | 14 | Go button | On-board KEY | Session / menus |
| — | 15 | Board power on | LilyGO power rail | HIGH at boot |
| IO16 | 16 | Battery ADC | 2S divider tap | R1 200 kΩ VM→GPIO, R2 100 kΩ GPIO→GND; `BAT_ADC_SCALE=2.64` |
| IO17 | 17 | Speaker PWM | Passive buzzer + | − to GND; boot beeps + tones |
| IO18 | 18 | WS2812 DIN | 8-LED ring | 330 Ω series optional; 5 V ring |
| IO21 | 21 | IR beacon TX | BC547 + IR LED | 38 kHz; `IR_TX_DEFAULT_ON=0` on drive build |
| TFT BL | 38 | LCD backlight | ST7789 BL | HIGH = on |
| IO43 | 43 | I2C SDA | MPU6050 SDA | 400 kHz |
| IO44 | 44 | I2C SCL | MPU6050 SCL | |
| TFT CS | 6 | TFT CS | ST7789 | Parallel chip select (LilyGO Setup206; see `platformio.ini` `[env:s3_tft_flags]`) |
| TFT DC | 7 | TFT DC | ST7789 | Parallel data/command |
| TFT RST | 5 | TFT RST | ST7789 | |
| TFT WR | 8 | TFT WR | ST7789 | |
| TFT RD | 9 | TFT RD | ST7789 | |
| TFT D0–D7 | 39–42, 45–48 | TFT data | ST7789 | 8-bit parallel |

Motor **VM**/GND: 2S pack (~7–8.4 V). Driver **5Vo** → LilyGO JST (+/−), common GND.

## Pro S3 (sensors)

| Board pin (silk) | GPIO | Function | Connects to | Notes |
|------------------|------|----------|-------------|--------|
| L8 SDA / SCL | 8 / 9 | I2C bus A | VL53L8CX SDA/SCL | Dedicated bus @ 400 kHz; 2.2 kΩ pull-ups |
| L8 LPn | 4 | VL53L8 reset | VL53L8CX LPn | |
| L8 INT | 5 | VL53L8 interrupt | VL53L8CX GPIO1 | Optional |
| SIDE SDA / SCL | 1 / 2 | I2C bus B | VL53L0X L/R, PCA9685 | 400 kHz |
| XSHUT L / R | 6 / 7 | Side ToF reset | VL53L0X XSHUT | Addr 0x30 / 0x31 |
| — | 12 | Sonar TRIG | HC-SR04 TRIG | |
| — | 13 | Sonar ECHO | HC-SR04 ECHO | Level-shift to 3.3 V |
| — | 14 | Fleet IR RX | TSOP38338 (or similar) | Active low; 38 kHz |
| — | 15 | Lidar UART RX | STL-19P ZH1.5 pin 1 Tx | 230400 8N1, one-way |
| — | 16 | Lidar PWM (reserve) | STL-19P pin 2 | Wired to **GND** for internal 10 Hz spin |
| LDO2 | 17 | 3V3 LDO enable | On-board LDO2 + RGB rail | Must be HIGH before peripherals and WS2812 |
| RGB | 18 | Status LED DIN | On-board WS2812 (×1) | Wi-Fi / agent colours; `pros3_status_led` (dim, non-blocking) |
| — | 21 | Fleet IR TX | BC547 + IR LED | RMT 38 kHz; `ROVER_IR_BEACON_ON=0` (TX task idle; RX active) |
| RF switch | 11 | Wi-Fi antenna | u.FL vs ceramic | HIGH = external u.FL (`ROVER_PROS3_ANT_EXTERNAL=1`) |

**STL-19P lidar mount:** The physical **0° mark on the sensor housing points to starboard** (chassis +Y). Firmware does **not** apply an extra yaw in the scan message: `sensor_msgs/LaserScan` uses `frame_id` **`lidar_link`**, with sensor 0° as +X in the message and angles increasing counter-clockwise (REP-103). The brain publishes **`base_link` → `lidar_link`** with a **+90° Z yaw** so scan forward matches chassis forward.

Lidar power: ZH1.5 pin 4 → solid **5 V** (~290 mA); pin 3 GND; pin 2 PWM → GND.

## ESP32-S3 AI Camera (rover-cam)

| Board pin (silk) | GPIO | Function | Connects to | Notes |
|------------------|------|----------|-------------|--------|
| XCLK | 5 | Camera XCLK | OV3660 | 20 MHz |
| SIOD / SIOC | 8 / 9 | SCCB I2C | OV3660 + LTR-308 @ 0x53 | |
| D0–D7 | 16, 18, 21, 17, 14, 7, 6, 4 | DVP data | OV3660 | See `rover_pins_aicam.h` |
| VSYNC / HREF / PCLK | 1 / 2 / 15 | DVP sync | OV3660 | |
| IR LED | 47 | IR illuminator | On-board IR LEDs | `/ir?on=0\|1` |
| Status LED | 3 | Status | On-board LED | |
| MIC CLK / DATA | 38 / 39 | PDM mic | On-board microphone | |
| I2S BCLK / LRC / DOUT | 45 / 46 / 42 | I2S out | MAX98357 → speaker | Greeting / stream playback |

HTTP: `:80` capture/control, `:81/stream` MJPEG. OTA hostname `rover-cam`.

## I2C devices

| Bus | Address | Device | Board |
|-----|---------|--------|-------|
| T-Display 43/44 | 0x68 | MPU6050 | Drive |
| T-Display 43/44 | 0x21 | MCP23008 (optional body IO) | Drive |
| Pro S3 8/9 | (VL53L8) | VL53L8CX front 8×8 | Pro S3 |
| Pro S3 1/2 | 0x30 | VL53L0X left | Pro S3 |
| Pro S3 1/2 | 0x31 | VL53L0X right | Pro S3 |
| Pro S3 1/2 | 0x40 | PCA9685 (sonar pan servo ch 0) | Pro S3 |
| AI cam 8/9 | 0x53 | LTR-308 ambient light | Camera |
| AI cam 8/9 | (OV3660) | Camera SCCB | Camera |

## Build steps

1. **Power:** Common GND across all three boards. Motor VM on the drive board only; 5 V rails for ESP boards, lidar P5V, and WS2812 where noted.
2. **T-Display motors:** Wire TC78H660 per table; left motor A−/A+ swapped at driver if needed. Enable board power (GPIO 15) and backlight (38) in firmware at boot.
3. **Wheel ticks:** Schmitt outputs → GPIO 11 / 12 with pull-ups; 95 mm wheels, 60 slots/rev.
4. **MPU6050:** SDA/SCL on 43/44; 3.3 V and GND.
5. **Speaker + ring:** Buzzer on GPIO 17; WS2812 DIN on 18 with optional 330 Ω; ring 5 V.
6. **Pro S3 L8:** Short I2C wires on 8/9, 2.2 kΩ pull-ups, LPn on GPIO 4.
7. **Pro S3 sides + PCA:** Bus 1/2; XSHUT 6/7; servo signal from PCA ch 0.
8. **Sonar:** TRIG 12, ECHO 13 through 3.3 V-safe divider.
9. **Fleet IR (Pro S3):** TSOP output → GPIO 14; TX GPIO 21 → 1 kΩ → NPN → IR LED(s) with series resistor from 5 V; fleet id **0** in firmware (`FLEET_IR_ID=0`).
10. **STL-19P:** Rx on GPIO 15; PWM pin to GND; 5 V and GND to lidar; mount with 0° mark to starboard; u.FL antenna on GPIO 11 HIGH.
11. **AI cam:** Stock DFR1154 harness; Wi-Fi credentials in `platformio_private.ini`; speaker on MX1.25 SPK header.

## Free pins

| Board | GPIOs (not assigned in firmware) |
|-------|----------------------------------|
| T-Display-S3 | **IO13** (GPIO 13) — only free pin on the breakout header; all other header GPIOs are assigned (motors, ticks, I2C, TFT parallel bus including CS on 6, etc.) |
| Pro S3 | 16 (reserved lidar PWM), 34, 35, 36, 37 — avoid 0, 3, 10, 17 (LDO2), 18 (RGB status), 19/20 (USB), 33 (VBUS), 45, 46 |
| AI cam | 44 (EXT IR, unused), USB/UART strapping per DFR1154 schematic |
