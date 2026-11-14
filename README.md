# LEGO Rover — ESP32-S3 firmware

Rover uses three ESP32-S3 nodes on WiFi; drive and sensors are split across two micro-ROS clients on the brain (Jazzy, agent `192.168.3.172:8888`).

| Board | Hostname | Firmware env | Source |
|-------|----------|--------------|--------|
| LilyGO **T-Display-S3** | `rover-esp` | `s3_tdisplay_microros_rover_drive` | `src/microros_main.cpp` |
| UM **Pro S3[D]** | `rover-tof` | `um_pros3_tof_microros` | `src/pros3_tof_microros_main.cpp` |
| DFRobot **AI Camera** | `rover-cam` | `aicam_rover_cam` | `src/aicam_main.cpp` |

**Drive board:** motors, MPU6050, wheel ticks, TFT, WS2812 ring, piezo speaker (GPIO 17), Go/E-stop, local safety.

**Sensor board:** VL53L8CX, side VL53L0X, rear sonar + PCA9685 pan, STL-19P `/rover/scan` (`lidar_link`), fleet IR TX/RX (id 0; beacon TX disabled in build to avoid cam bloom).

**Camera:** MJPEG stream and I2S speaker for greetings (NUC Piper → HTTP play).

## Docs

- [`docs/PINMAP.md`](docs/PINMAP.md) — pin map (three boards)
- [`WIRING.md`](WIRING.md) — wiring summary and flash commands
- [`docs/TODO.md`](docs/TODO.md) — Rover and fleet future plans

## Flash

See `WIRING.md` → Flash. Turn `allow_drive` OFF before OTA. Wi-Fi in gitignored `platformio_private.ini` (see `platformio_private.ini.example`).

## Use

1. Power all boards (Wi-Fi + micro-ROS agent on the two ROS nodes).
2. `allow_drive` ON via MQTT `lab/robots/allow_drive/set`.
3. Tap **Go** (or `/rover/button` true) → brain session → explore publishes `/cmd_vel`.
4. Drive board enforces E-stop, stall lockout, and cmd timeout.

Diagnostics: `/rover/drive_diag` (drive, 1 Hz).

## T-Display boot beeps

Passive buzzer on GPIO 17 plays a short non-blocking sequence at boot (LEDC speaker task).

Order: power chirp → Wi-Fi (optional) → agent (optional) → hardware OK triad → or error codes (low double + N short beeps).

| N | Device | Health check |
|---|--------|----------------|
| 1 | IMU (MPU6050) | `mpu.ok()` |
| 2 | Wheel ticks | GPIO 11/12 ISRs attached |
| 3 | VL53L8 (Pro S3) | `/rover/tof/status[12]` or fresh `l8_cols` |
| 4 | Side VL53 left | `[13]` or `/rover/tof/left` |
| 5 | Side VL53 right | `[14]` or `/rover/tof/right` |
| 6 | Rear sonar | `[11]` (no echo is OK) |
| 7 | PCA9685 pan | `[2] ≥ 0.5` |
| 8 | micro-ROS agent | Session + Pro S3 status path |

Spoken time-of-day greeting is streamed from the NUC to the AI cam (`GET /play?url=`), not the piezo.

## Related

- `../lego-rover-ros2/ROVER_BEHAVIOR.md` — explore behaviour
- `../c3-mini-bot/docs/SWARM.md` — fleet IR protocol
