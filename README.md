# LEGO Rover — ESP32-S3 firmware

Rover is dual-board, both thin WiFi micro-ROS clients of the brain:

- **T-Display-S3 drive board** (`rover-esp`, .71) — motors, MPU6050 IMU, wheel ticks, LED ring, speaker, Go/E-stop, display, local safety. Env `s3_tdisplay_microros_rover_drive` (`src/microros_main.cpp`).
- **Pro S3 ToF board** (`rover-tof`) — VL53L8CX front, VL53L0X sides, PCA9685. Env `um_pros3_tof_microros` (`src/pros3_tof_microros_main.cpp`).

**Brain:** NUC `brain` (192.168.3.172) runs `rover-brain` in Docker under `/data/swarm/robot-fleet` (ROS 2 Jazzy), micro-ROS agent on UDP 8888.

## Flash

See `WIRING.md` → Flash. Turn `allow_drive` OFF before an OTA.

## Use

1. Power the rover (both boards join WiFi and the agent).
2. `allow_drive` ON via MQTT `lab/robots/allow_drive/set`.
3. Tap **Go** (or publish `/rover/button` true) → brain session starts → explore publishes `/cmd_vel`.
4. The drive board enforces E-stop, stall lockout and cmd timeout; explore/escape logic is on the brain.

Diagnostics: `/rover/drive_diag` (drive board, 1 Hz).

## Docs

- `WIRING.md` — pins for both boards
- `../lego-rover-ros2/ROVER_BEHAVIOR.md` — explore behaviour
- `../c3-mini-bot/docs/SWARM.md` — fleet

## T-Display boot beeps

The passive buzzer (GPIO 17) plays a short non-blocking diagnostic sequence at boot (LEDC, speaker task ? it never stalls drive or micro-ROS).

Order:

1. **Power-on chirp** ? one short pip as early as possible in `setup()`, before the rest of the hardware init.
2. **Wi-Fi connected** (optional, short rising pair) once the station has an IP.
3. **Agent link** (optional, short pair) when the micro-ROS session comes up.
4. **Hardware OK** ? a cheerful three-note triad, as soon as every device below looks healthy.
5. **Hardware error** ? if anything is still missing after ~15 s: a low double beep, then **N** short beeps for each failed device (250 ms between codes).

### Error code table (N short beeps)

| N | Device | How it is judged |
|---|--------|------------------|
| 1 | IMU (MPU6050) on the T-Display | `mpu.ok()` |
| 2 | Encoders on the T-Display | MCP23017 body / MCP IR OK |
| 3 | VL53L8CX front (Pro S3) | Recent `/rover/tof/l8_cols` or L8 frames in `/rover/tof/status` |
| 4 | Side VL53 left (Pro S3) | Recent `/rover/tof/left` (or a valid range latched this boot) |
| 5 | Side VL53 right (Pro S3) | Recent `/rover/tof/right` (or a valid range latched this boot) |
| 6 | Rear sonar (Pro S3) | `/rover/tof/status[0]` > 0 with a fresh status |
| 7 | PCA9685 pan servo (Pro S3) | `/rover/tof/status[2]` ? 0.5 with a fresh status |
| 8 | micro-ROS agent | No agent session / no path to the Pro S3 status topics |

The sequence is also logged as `boot_beep ...` on serial and via `rover_diag_event` (see `/rover/drive_diag` / telem).

The spoken ?Hi, good morning/afternoon/evening?? greeting is **not** on the buzzer. The NUC `rover-boot-greet` service synthesises it with Piper (same Alan voice as rover-speech) and streams it to the AI cam with `GET /play?url=`.

