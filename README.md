# LEGO Rover — ESP32-S3 (T-Display)

**ESP role:** thin real-time client — motors, IMU, pan sonar, safety cutouts, display, WiFi OTA.

**Brain:** Raspberry Pi on the rover runs `autonomous_explore.py` over micro-ROS (UART agent). Pi also drives speaker + LED ring.

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
- `../c3-mini-bot/docs/SWARM.md` — fleet (minis on dockerhost, rover on Pi)
