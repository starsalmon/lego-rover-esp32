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
