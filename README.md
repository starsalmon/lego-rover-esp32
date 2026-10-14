# LEGO Rover — ESP32-S3 (T-Display)

**ESP role:** thin real-time client — motors, IMU, wheel ticks, ring/speaker, safety cutouts, display, WiFi OTA. Front/side ToF live on the separate Pro S3 board (`src/pros3_tof_microros_main.cpp`).

**Brain:** NUC `brain` (192.168.3.172) runs `rover-brain` in Docker under `/data/swarm/robot-fleet` (ROS 2 Jazzy), via the WiFi micro-ROS agent on UDP 8888. The Pi is no longer used.

## Flash (OTA)

```bash
cd lego-rover-esp32
pio run -e s3_tdisplay_microros_rover_ota -t upload
```

First-ever flash or bricked recovery: USB with the same env (omit espota) or `s3_erase` then USB.

## Use

1. Power rover (ESP + Pi).
2. Pi agent: `systemctl status rover-agent` (or `go_auto.sh` starts the stack).
3. Tap **Go** on ESP display → Pi session active → explore publishes `/cmd_vel`.
4. ESP hard-brakes on imminent collision; wander/escape logic is on the Pi.

## Deprecated

`s3_standalone_bench` — old ESP-local wander sketch. Not production.

## Docs

- `WIRING.md` — pins
- `../lego-rover-ros2/ROVER_BEHAVIOR.md` — priorities / explore
- `../lego-rover-ros2/PI_PERIPHERAL.md` — Pi speaker + ring
- `../c3-mini-bot/docs/SWARM.md` — fleet (all bots on the NUC brain)
