# LEGO Rover — wiring

Three ESP32-S3 boards; two join micro-ROS on the brain (`192.168.3.172:8888`). Full pin tables: **`docs/PINMAP.md`**.

| Board | Host | Role | Pin header |
|-------|------|------|------------|
| LilyGO **T-Display-S3** | `rover-esp` (`.71`) | Drive: motors, IMU, ticks, TFT, ring, **speaker**, Go/E-stop | `include/rover_pins_s3.h` |
| UM **Pro S3** | `rover-tof` | ToF, sonar, **fleet IR TX/RX**, STL-19P lidar | `include/rover_pins_pros3.h` |
| DFRobot **AI cam** | `rover-cam` | OV3660 MJPEG, **I2S speaker**, greeting playback | `include/rover_pins_aicam.h` |

Fleet IR id **0** lives on the Pro S3 (GPIO 14 RX, GPIO 21 TX). The drive board has an optional GPIO 21 beacon (`IR_TX_DEFAULT_ON=0`). Audio for speech and alerts is on the **T-Display piezo (GPIO 17)** and the **AI cam I2S amp**.

---

# T-Display-S3 drive board

## Pin map

| GPIO | Function |
|------|----------|
| **0** | E-stop (BOOT) |
| **1** / **2** | Motor A DIR / PWM (left) |
| **3** / **10** | Motor B DIR / PWM (right) |
| **11** / **12** | Wheel tick left / right |
| **14** | Go button |
| **15** | Board power on |
| **16** | Battery ADC |
| **17** | Passive buzzer (+) |
| **18** | WS2812 ring DIN |
| **21** | IR beacon TX (off by default on production drive build) |
| **38** | LCD backlight |
| **43** / **44** | I2C — MPU6050 @ **0x68** |

Parallel TFT uses GPIO 6 (CS), 5 (RST), 7–9, 39–42, 45–48 (see `docs/PINMAP.md`). Free header pin: **IO13** only.

Motors: **A = left**, **B = right**. Firmware `MOTOR_INVERT_L/R=1`, `PWM_FREQ_HZ=30000`, floor **0.55**.

## TC78H660 hookup

| T-Display | Driver |
|-----------|--------|
| 1 / 2 | DIR A / PWM A |
| 3 / 10 | DIR B / PWM B |
| GND | GND |

VM from 2S pack; **5Vo** → LilyGO JST. Left motor may need A−/A+ swap at the driver.

## Speaker + ring

| Device | GPIO | Notes |
|--------|------|--------|
| Buzzer + | **17** | − to GND |
| WS2812 DIN | **18** | 8 LEDs; 5 V |
| Ring GND/VCC | GND / 5 V | |

Ring colours follow Pro S3 topics (`l8_cols`, `tof/left|right`).

## Battery

R1 **200 kΩ** (VM → GPIO16), R2 **100 kΩ** (GPIO16 → GND). Cal `BAT_ADC_SCALE` **2.64**; critical **6.0 V**, low **6.4 V**.

---

# Pro S3 sensor board

| Bus / pin | Device |
|-----------|--------|
| I2C **8** / **9** | VL53L8CX (LPn **4**, INT **5**) |
| I2C **1** / **2** | VL53L0X **0x30** / **0x31** (XSHUT **6** / **7**), PCA9685 **0x40** |
| **12** / **13** | HC-SR04 |
| **14** / **21** | Fleet IR RX / TX (`ROVER_IR_BEACON_ON=0`, RX still counted) |
| **15** | STL-19P UART RX |
| **11** | RF: HIGH = u.FL |

Publishes `/rover/tof/*`, `/rover/sonar/range`, `/rover/scan`, `/rover/tof/lidar` diagnostics.

## STL-19P

| Lidar ZH1.5 | Pro S3 |
|-------------|--------|
| 1 Tx | GPIO **15** (UART RX, 230400 8N1) |
| 2 PWM | **GND** (internal 10 Hz) |
| 3 GND | GND |
| 4 P5V | 5 V (~290 mA) |

**Mount:** The sensor housing **0° mark points to starboard**. Firmware publishes `lidar_link` with REP-103 angles (no extra yaw in the message). Apply **`base_link` → `lidar_link` +90° yaw** on the brain.

Scan: `frame_id` **`lidar_link`**, 500 bins, NaN for misses. Custom XRCE MTU **1400** on this client.

`/rover/tof/status` is **16** floats (`[15]` = lidar recent). UART bring-up counters: `/rover/tof/lidar`.

---

# AI camera (`rover-cam`)

DFRobot DFR1154: OV3660 on the built-in DVP bus, LTR-308 on I2C, on-board IR LEDs (GPIO 47), **MAX98357 I2S speaker** (GPIO 45/46/42). MJPEG **:81/stream**, capture **:80/capture**. No micro-ROS on this node.

---

# Flash

```bash
# Drive (OTA example)
pio run -e s3_tdisplay_microros_rover_drive
pio run -e s3_tdisplay_microros_rover_drive_ota -t upload

# Pro S3 — USB first, then OTA
pio run -e um_pros3_tof_microros -t upload
pio run -e um_pros3_tof_microros_ota -t upload

# Camera
pio run -e aicam_rover_cam -t upload
pio run -e aicam_rover_cam_ota -t upload
```

Wi-Fi: `platformio_private.ini` (gitignored).

## Related

- [`docs/PINMAP.md`](docs/PINMAP.md)
- [`lego-rover-ros2/ROVER_BEHAVIOR.md`](../lego-rover-ros2/ROVER_BEHAVIOR.md)
