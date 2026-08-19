# LEGO Rover — ESP32 wiring

**Source of truth:** `include/rover_pins_s3.h` (T-Display-S3) or C3 headers if you swap boards.

**Production rover:** LilyGO **T-Display-S3** — speaker + ring on ESP GPIO **17/18**; brain on dockerhost.

---

# LilyGO T-Display-S3 (current)

Board: [T-Display-S3](https://github.com/Xinyuan-LilyGO/T-Display-S3)

## Quick pin map

| GPIO | Function |
|------|----------|
| **0** | E-stop button (BOOT — hold = motor cut on ESP) |
| **1** | Motor A DIR (**left**) |
| **2** | Motor A PWM (**left**) |
| **3** | Motor B DIR (**right**) |
| **10** | Motor B PWM (**right**) |
| **11** | Sonar TRIG (HC-SR04) |
| **12** | Sonar ECHO (HC-SR04) — **level-shift to 3.3 V** |
| **14** | Go / start-stop button (KEY, top-right) |
| **15** | Board power on (`PIN_POWER_ON`) |
| **16** | Battery ADC (2S divider tap) |
| **17** | Passive buzzer / piezo (+) |
| **18** | WS2812 ring DIN (8 LEDs) |
| **21** | IR chase beacon TX (38 kHz, for mini-bot TSOP) |
| **38** | LCD backlight |
| **43** | I2C SDA (MPU6050 + MCP23008 + PCA9685) |
| **44** | I2C SCL |

Motor naming in firmware: **A = left**, **B = right**. `DIR high` + `PWM` = forward on that channel.

---

## T-Display-S3 → Makerverse TC78H660 driver

| T-Display | GPIO | Driver |
|-----------|------|--------|
| IO1 | 1 | DIR A (left) |
| IO2 | 2 | PWM A (left) |
| IO3 | 3 | DIR B (right) |
| IO10 | 10 | PWM B (right) |
| GND | — | GND |

| Driver | Connect |
|--------|---------|
| **VM** + **GND** | 2S battery (~7–8.4 V) |
| **5Vo** | ESP 5 V in (JST recommended), Pi 5 V |
| Motor A / B | LEGO motors |

**Confirmed:** Motor **A = left**, **B = right**. Firmware: `MOTOR_INVERT_L=1`, `MOTOR_INVERT_R=1` (see `platformio.ini`).

PWM **15 kHz** — better low-end torque on this driver/motors than 20 kHz (may be slightly more audible). Floors **0.54** both sides (fwd start; use **56%** raw in motor_test for paired start on the floor). If wheels need a push, check motor-holder screws before raising duty.

### Motor → driver (physical)

| Motor | Driver | Notes |
|-------|--------|-------|
| **Right** | B+ / B− | Normal polarity |
| **Left** | A− / A+ | Wires swapped at driver |

---

## T-Display-S3 ↔ Raspberry Pi (UART — legacy)

**Deprecated** for production — brain is on dockerhost; speaker + ring are on the ESP (GPIO **17** / **18**).

| Pi header | Pi BCM | T-Display (old) |
|-----------|--------|-----------------|
| Pin 8 TX | GPIO **14** | → **IO17** |
| Pin 10 RX | GPIO **15** | ← **IO18** |

Only needed if you still run `pi_peripheral_daemon.py` on a Pi.

---

## I2C bus (GPIO43 SDA, GPIO44 SCL)

One daisy-chained bus — **3.3 V only**, **4.7 kΩ** pull-ups on SDA/SCL once per bus.

| Device | Address | Role |
|--------|---------|------|
| MPU6050 | **0x68** | IMU — flat on chassis, Z up |
| MCP23008 “front” | **0x20** | Front bumper IR (ribbon PCB) |
| MCP23008 “body” | **0x21** | Aux + wheel IR |
| PCA9685 | **0x40** | Pan sonar servo **ch 15**, aux head servo **ch 0** |
| VL53L1X left | **0x30** | Side ToF — XSHUT → body MCP **GPA6** |
| VL53L1X right | **0x31** | Side ToF — XSHUT → body MCP **GPA7** |

### VL53L1X side ToF (hallway wall-follow)

Two **VL53L1X** breakouts on the same bus as MPU/MCP/PCA. Default address is **0x29** on both — firmware toggles **XSHUT** on body MCP **GPA6/GPA7** to assign **0x30** (left) and **0x31** (right) at boot.

| Breakout | Connect |
|----------|---------|
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO **43** |
| SCL | GPIO **44** |
| XSHUT left | Body MCP **GPA6** (pin 6 on @0x21) |
| XSHUT right | Body MCP **GPA7** (pin 7 on @0x21) |
| GPIO1 (interrupt) | leave unconnected |

**One sensor only:** wire XSHUT to **GPA6** (left) or **GPA7** (right); the other init fails harmlessly. Set `HALLWAY_WALL=left` or `right` on dockerhost.

Mount left/right pointing sideways (~90° from forward). Brain: `ROVER_MODE=hallway`, tap **Go** — holds **10 cm** (`HALLWAY_TARGET_M=0.10`).

### MPU6050

| MPU6050 | ESP |
|---------|-----|
| SDA | GPIO **43** |
| SCL | GPIO **44** |
| VCC | 3.3 V |
| GND | GND |

Heading hold uses **gyro Z**. Boot log `MPU at rest: …` — **|az| ≈ 1g** if Z points up.

### Front bumper MCP @ **0x20** (A2 A1 A0 = GND GND GND)

| GPA | Dir | Signal |
|-----|-----|--------|
| **0** | in | Front left detect ← LM358 |
| **1** | in | Front right detect ← LM358 |
| **2** | out | Front left emitter gate → BC547 (**steady ON**; 555 on bumper modulates) |
| **3** | out | Front right emitter gate → BC547 (**steady ON**) |

### Body MCP @ **0x21** (A2 A1 A0 = GND GND **3.3 V**)

| GPA | Dir | Signal |
|-----|-----|--------|
| **0** | in | Aux detect ← Schmitt |
| **1** | in | Wheel left detect |
| **2** | in | Wheel right detect |
| **3** | out | Aux emitter (GPIO pulse) |
| **4** | out | Wheel left emitter |
| **5** | out | Wheel right emitter |
| **6** | out | Left VL53L XSHUT (HIGH = on) |
| **7** | out | Right VL53L XSHUT (HIGH = on) |

Pi uses `ROVER_IR_SOURCE=esp` — no Pi GPIO for IR.

### PCA9685 @ **0x40** — servos

| Channel | Silkscreen | Role |
|---------|------------|------|
| **0** | OUT1 | Aux head servo (rear radar aim) — SIG / V+ / GND on header |
| **15** | OUT16 | **Pan sonar** bracket servo — sweeps HC-SR04 left/right |

OE: tie **LOW** on module or wire per your board (legacy Pi GPIO24 OE only if PCA was on Pi).

Servo power: **5 V** from PCA V+ rail (separate BEC OK). Signal is 3.3 V logic from ESP via PCA.

Firmware pan: **0°–180°** logical sweep, centre **90°** (`SONAR_PAN_CENTER_DEG`). Trim/invert: `SONAR_PAN_TRIM_DEG`, `SONAR_PAN_INVERT` in `rover_pins_s3.h`.

---

## Pan sonar (HC-SR04 + servo)

Ultrasonic on a pan bracket — ESP reads range and drives escape. **Standalone brain** (`standalone_main.cpp`); not Pi GPIO.

### HC-SR04 → T-Display-S3

| HC-SR04 | ESP GPIO | Notes |
|---------|----------|--------|
| **VCC** | **5 V** | Same 5 V rail as motors/Pi (not 3.3 V) |
| **GND** | GND | Common with ESP |
| **TRIG** | **GPIO 11** | 3.3 V OK |
| **ECHO** | **GPIO 12** | **Must be 3.3 V** — divider or level shifter (HC-SR04 ECHO is 5 V) |

Typical divider: ECHO → **20 kΩ** → GPIO12 → **10 kΩ** → GND (ratio ≈ 3.3/5).

### Pan servo → PCA9685

| Servo wire | PCA9685 |
|------------|---------|
| Signal (orange/yellow) | **Channel 15** (OUT16) |
| V+ (red) | V+ terminal (5 V) |
| GND (brown/black) | GND |

Mount: servo horn centres the bracket at **90°**; firmware sweeps full **0° / 180°** during cruise glance and 180° scan on escape.

### Behaviour (firmware)

| Constant | Default | Meaning |
|----------|---------|---------|
| `SONAR_TRIG_PIN` | 11 | Trigger |
| `SONAR_ECHO_PIN` | 12 | Echo (shifted) |
| `SONAR_SERVO_CHANNEL` | 15 | PCA channel |
| `SONAR_PAN_MIN_DEG` / `MAX` | 0 / 180 | Full sweep endpoints |
| `SONAR_PAN_CENTER_DEG` | 90 | Centre (forward) |
| `ROVER_BOOT_PAN_SWEEP` | 0 | Boot pan left/right sweep (1 = bench; 0 = quiet center only) |
| `SONAR_BOOT_PAN_MS` | 280 | Boot left/right/center dwell when sweep enabled |

Boot sequence (when `ROVER_BOOT_SPIN_TRICK=1`): **pan left (180°) → right (0°) → centre**, then IMU **left 360° + right 360°** at 10% (`ROVER_BOOT_SPIN_LIN`).

If the boot pan sweep looks reversed, set `SONAR_PAN_INVERT=1` in `platformio.ini` and reflash.

---

## Speaker + LED ring (GPIO 17 / 18)

| Device | ESP GPIO | Notes |
|--------|----------|--------|
| Passive buzzer **+** | **17** (P1 header) | PWM tones — **−** to GND |
| WS2812 ring **DIN** | **18** (P1 header) | 330 Ω series if you have one |
| Ring **VCC** | **5 V** | Same rail as Pi used (pin 2/4) |
| Ring **GND** | GND | Common with ESP |

Firmware: `ROVER_PERIPH=1` in `s3_tdisplay_microros*`. Cues: ready tune on agent connect, session start/stop, bump/stall flashes, sonar distance colour map during pan scans.

---

## IR chase beacon (38 kHz) — GPIO **21**

For c3-mini-bot / TSOP4138 swarm chase — **separate** from bumper 555 proximity IR.

| ESP | Circuit |
|-----|---------|
| **GPIO21** | **1 kΩ** → BC547 base |
| **5 V** | → resistor → IR LED anode → collector |
| **GND** | emitter + LED cathode |

Firmware: `IR_TX_DEFAULT_ON=1`, burst 560 µs on/off (`IR_TX_MODULATE=1`). Point LEDs **forward** on the rover.

---

## Battery monitor (2S on VM)

| Part | Value |
|------|--------|
| R1 (VM → GPIO16) | **200 kΩ** |
| R2 (GPIO16 → GND) | **100 kΩ** |

**Important:** divider is on **motor VM** (after your main pack switch). Switch **OFF** + ESP on USB → tap/pack readings are meaningless.

Nominal **200k + 100k** → tap ≈ **pack ÷ 3** (7.28 V pack → ~2.4–2.6 V tap). Cal: `BAT_ADC_SCALE = pack_V / tap_V` (currently **2.76**).

Critical **6.0 V**, low **6.4 V**.

---

## Onboard buttons

| Button | GPIO | Action |
|--------|------|--------|
| **Go** | **14** | Start/stop, menus → `/rover/button` |
| **E-stop** | **0** | Hold = instant motor stop on ESP |

Display shows link, battery, cmd_vel, mode.

---

## Power (T-Display-S3)

**Remove internal JST LiPo** if using external pack.

**Recommended:** motor driver **5Vo** → LilyGO **JST battery socket** (+/−). Allows 3–6 V on JST path; 5Vo is in range.

| Do | Don't |
|----|--------|
| 5Vo → **JST** + common GND | 5V header + **USB** at the same time |
| Flash via USB, unplug for driving | Pi 3.3 V → LilyGO power |

Firmware enables **GPIO15** + **GPIO38** at boot.

**On-robot builds** (no USB wait): `s3_tdisplay_microros_rover` / `_rover_ota`.

---

## Motor calibration (motor_test)

```bash
pio run -e s3_tdisplay_motor_test -t upload
pio device monitor -b 115200
```

| Command | Use |
|---------|-----|
| `freq 20000` | Try PWM Hz live (production default is **20000** in `platformio.ini`) |
| `scan B 5` | Ramp both wheels — find **start** % on the floor |
| `scan L 5` / `scan R 5` | Per-wheel if one side is lazy |
| `B 60` | Spot-check raw duty (bypasses ROS remap) |
| `stop` | Off |

**Start vs run:** note the lowest % that **breaks static friction** (start) vs % that **keeps rolling** after a nudge (run). Autonomous uses compile-time **`MIN_PWM_FLOOR_L` / `MIN_PWM_FLOOR_R`** in `[env:s3_common]` — set from the **start** value (e.g. 58% → `0.58f`).

`freq` in serial **does not persist** — change `PWM_FREQ_HZ` in `platformio.ini` and reflash `s3_tdisplay_microros_rover` for autonomous.

---

## Flash (T-Display-S3)

```bash
# Motor smoke test
pio run -e s3_tdisplay_motor_test -t upload

# Dev micro-ROS + display (USB)
pio run -e s3_tdisplay_microros -t upload

# Rover on robot + OTA
cp platformio_private.ini.example platformio_private.ini   # WiFi once
pio run -e s3_tdisplay_microros_rover -t upload
pio run -e s3_tdisplay_microros_rover_ota -t upload      # wireless updates
```

OTA hostname: `rover-esp.local` (see serial log).

---

# ESP32-C3 Zero (alternate board)

If you swap back to [ESP32-C3 Zero](https://www.espboards.dev/esp32/esp32-c3-zero/):

**GPIO4–7 are JTAG/flash — not usable for motors.**

| C3 | Driver |
|----|--------|
| IO0 | DIR A (left) |
| IO1 | PWM A (left) |
| IO3 | DIR B (right) |
| IO2 | PWM B (right) |

| C3 | MPU6050 |
|----|---------|
| IO8 (SDA) | SDA |
| IO9 (SCL) | SCL |

| Pi | C3 |
|----|-----|
| TX GPIO14 | → **IO20** (RX) |
| RX GPIO15 | ← **IO21** (TX) |

Status: onboard WS2812 **GPIO10** (no TFT).

```bash
pio run -e c3_motor_test -t upload
pio run -e c3_microros -t upload
```

IR via MCP23008 and PCA9685 wiring is the same logical layout on the shared I2C bus (use C3 GPIO8/9 for SDA/SCL instead of 43/44).

---

## Deprecated / not used

- **74HC595 / 74HC165** shift-register IR — replaced by MCP23008.
- **Single-breadboard MCP layout** (all inputs @0x20, all outputs @0x21 at matching GPA) — superseded by **front @0x20 + body @0x21** split above.
- **Pi GPIO IR** (6, 22, 26, 27) — use when `ROVER_IR_SOURCE=pi` only.

---

## Related

- [`lego-rover-ros2/WIRING.md`](../lego-rover-ros2/WIRING.md) — Pi UART, buzzer, ring
- [`lego-rover-ros2/BUTTON.md`](../lego-rover-ros2/BUTTON.md) — Go / E-stop behaviour
- `include/rover_pins_s3.h` — pin `#define`s
