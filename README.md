# LEGO Rover — ESP32-S3 firmware

Rover uses three ESP32-S3 nodes on WiFi; drive and sensors are split across two micro-ROS clients on the brain (Jazzy, agent `192.168.3.172:8888`).

| Board | Hostname | Firmware env | Source |
|-------|----------|--------------|--------|
| LilyGO **T-Display-S3** | `rover-esp` | `s3_tdisplay_microros_rover_drive` | `src/microros_main.cpp` |
| UM **Pro S3[D]** | `rover-tof` | `um_pros3_tof_microros` | `src/pros3_tof_microros_main.cpp` |
| DFRobot **AI Camera** | `rover-cam` | `aicam_rover_cam` | `src/aicam_main.cpp` |

**Drive board:** motors, MPU6050, wheel ticks, TFT, WS2812 ring, piezo speaker (GPIO 17), Go/E-stop, local safety.

**Sensor board:** VL53L8CX, side VL53L0X, rear sonar + PCA9685 pan, STL-19P `/rover/scan_slice` (`lidar_link`, 4 best-effort slices per revolution; the NUC rebuilds `/rover/scan`), fleet IR TX/RX (id 0; beacon TX disabled in build to avoid cam bloom). Onboard **RGB status LED** (GPIO 18, rail via LDO2): amber = Wi-Fi joining, slow blue = no agent, dim green = linked, red blink = reconnecting.

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

### Fleet PS4 manual drive (ESP-NOW)

The **T-Display** drive board (`s3_tdisplay_microros_rover_drive`) can take manual drive from the fleet PS4 bridge (NodeMCU-32S, jazz `ps4_bridge_fleet` env). Pairing matches Jazz/Roby: bridge MAC `e0:8c:fe:f9:c2:34`, Share short-press cycles target (Rover = slot **2** once jazz bridge adds it).

| Control | Action |
|---------|--------|
| **Options** | Toggle manual ON/OFF (motors never run until manual + live link) |
| **Left stick X** | Steer (expo 0.6); optional **heading hold** when stick centered and MPU ready |
| **R2** | Forward; **L2** brake-then-reverse |
| **Circle** | Speed LOW (50%) / HIGH |
| **Square** | Steer LOW (50%) / HIGH |

Manual ON + link OK overrides `/cmd_vel`. Link loss (**300 ms** failsafe), manual OFF, or controller disconnect → motors stop and ROS session drive resumes. Acks are sent on **every** packet (even while the micro-ROS agent is down) so the bridge learns Rover’s STA MAC and Wi-Fi channel. Flip steering in `platformio.ini` with `-DROVER_PS4_STEER_SIGN=-1.0f` if needed.

`/rover/drive_diag` (1 Hz) also adds `ps4_link`, `ps4_man`, `ps4_age`, `ps4_ch`, `ps4_ack`; the TFT shows `PS4 MAN` when active. `bssid=` / `roams=` show the associated AP and app roam count. Pro S3 `/rover/tof/wifi` adds `bssid` last octet and roam count as elements 7–8.

### Wi-Fi roaming (same SSID, multiple APs)

Both ROS nodes use shared `wifi_roam` (`src/wifi_roam.cpp`): `wifi_sta_config_t` rm/btm (802.11k/v) where the SDK exposes those fields (Arduino-ESP32 3.2 / IDF 4.4 here), plus a non-blocking scan every 10 s when RSSI &lt; −70 dBm. If another BSSID is ≥ 8 dB stronger, reconnect directly (min 30 s between roams). Power save stays off; TX power unchanged. Serial logs each roam with old/new RSSI and BSSID.

## T-Display boot beeps

Passive buzzer on GPIO 17 plays a short non-blocking sequence at boot (LEDC speaker task).

Order: power chirp → Wi-Fi (optional) → agent (optional) → hardware OK triad → or error codes (low double + N short beeps). Error codes play only after grace: **20 s** for local I2C/IMU/encoders, **45 s** for micro-ROS agent and Pro S3 ToF topics (`ROVER_BOOT_BEEP_LOCAL_GRACE_MS` / `ROVER_BOOT_BEEP_NETWORK_GRACE_MS`). Pro S3 RGB LED uses the same 45 s window before “no agent” styling.

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
