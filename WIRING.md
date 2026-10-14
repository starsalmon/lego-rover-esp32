# LEGO Rover — ESP32 wiring

Rover has two ESP32-S3 boards on WiFi micro-ROS to the brain (`192.168.3.172:8888`):

| Board | Host | Role | Pin header |
|-------|------|------|------------|
| LilyGO **T-Display-S3** | `rover-esp` (.71) | Drive: motors, IMU, wheel ticks, ring, speaker, buttons, display | `include/rover_pins_s3.h` |
| UM **Pro S3** | `rover-tof` | ToF: VL53L8CX front, VL53L0X left/right, PCA9685 | `include/rover_pins_pros3.h` |

---

# T-Display-S3 drive board

## Pin map

| GPIO | Function |
|------|----------|
| **0** | E-stop button (BOOT — hold = motor cut on ESP) |
| **1** | Motor A DIR (**left**) |
| **2** | Motor A PWM (**left**) |
| **3** | Motor B DIR (**right**) |
| **10** | Motor B PWM (**right**) |
| **11** | Wheel tick left (direct GPIO ISR) |
| **12** | Wheel tick right (direct GPIO ISR) |
| **14** | Go button (KEY, top-right) |
| **15** | Board power on (`PIN_POWER_ON`) |
| **16** | Battery ADC (2S divider tap) |
| **17** | Passive buzzer / piezo (+) |
| **18** | WS2812 ring DIN (8 LEDs) |
| **21** | IR beacon TX (38 kHz, off by default: `IR_TX_DEFAULT_ON=0`) |
| **38** | LCD backlight |
| **43** | I2C SDA — MPU6050 |
| **44** | I2C SCL |

Motor naming in firmware: **A = left**, **B = right**. `DIR high` + `PWM` = forward on that channel.

Wheels: **95 mm** diameter, **60** slits/rev, **130 mm** track.

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
| **5Vo** | ESP 5 V in (JST recommended) |
| Motor A / B | LEGO motors |

Firmware: `MOTOR_INVERT_L=1`, `MOTOR_INVERT_R=1`, `PWM_FREQ_HZ=30000`, PWM floor `MIN_PWM_FLOOR_L/R=0.55f` (see `[env:s3_common]`). A cmd of 0..1 maps onto the usable range above that floor, so speeds are quoted as **usable %** (cmd × 100).

| Motor | Driver | Notes |
|-------|--------|-------|
| **Right** | B+ / B− | Normal polarity |
| **Left** | A− / A+ | Wires swapped at driver |

## I2C (GPIO43 SDA, GPIO44 SCL)

| Device | Address |
|--------|---------|
| MPU6050 | **0x68** |

Nothing else is on this bus. The firmware retries IMU detection every 3 s if it is missing at boot. `/rover/drive_diag` reports `mpu=` and the bus scan.

## Speaker + LED ring (GPIO 17 / 18)

| Device | ESP GPIO | Notes |
|--------|----------|--------|
| Passive buzzer **+** | **17** | PWM tones — **−** to GND |
| WS2812 ring **DIN** | **18** | 330 Ω series if you have one |
| Ring **VCC** | **5 V** | |
| Ring **GND** | GND | Common with ESP |

Firmware: `ROVER_PERIPH=1`. Ring shows distance colours (near = orange → far = blue) from the Pro S3 topics: front LEDs (45°/0°/315°) from `/rover/tof/l8_cols`, side LEDs (90°/270°) from `/rover/tof/left|right`. Cues for ready, session start/stop, bump/stall.

## IR beacon (38 kHz) — GPIO **21**

| ESP | Circuit |
|-----|---------|
| **GPIO21** | **1 kΩ** → BC547 base |
| **5 V** | → resistor → IR LED anode → collector |
| **GND** | emitter + LED cathode |

## Battery monitor (2S on VM)

| Part | Value |
|------|--------|
| R1 (VM → GPIO16) | **200 kΩ** |
| R2 (GPIO16 → GND) | **100 kΩ** |

Divider is on **motor VM** (after the main pack switch); readings are meaningless with the pack off. Cal `BAT_ADC_SCALE` (currently **2.76**). Critical **6.0 V**, low **6.4 V**.

## Onboard buttons

| Button | GPIO | Action |
|--------|------|--------|
| **Go** | **14** | Start/stop session, menus → `/rover/button`, `/rover/button_event` |
| **E-stop** | **0** | Hold = instant motor stop on ESP |

## Power

**Recommended:** motor driver **5Vo** → LilyGO **JST battery socket** (+/−), common GND. Don't power from the 5 V header and USB at the same time. Firmware enables **GPIO15** + **GPIO38** at boot.

---

# Pro S3 ToF board

| Bus / pin | Device |
|-----------|--------|
| I2C A: SDA **8**, SCL **9** @ **400 kHz** | VL53L8CX front 8×8 (LPn **4**, INT **5**) |
| I2C B: SDA **1**, SCL **2** @ 400 kHz | VL53L0X left/right (XSHUT **6** / **7**, addr **0x30** / **0x31**), PCA9685 **0x40** |
| **12** / **13** | HC-SR04 TRIG/ECHO input (firmware publishes `/rover/sonar/range` when fitted) |
| **14** / **21** | IR RX / IR TX |

Onboard ceramic antenna (`ROVER_PROS3_ANT_EXTERNAL=0`). Publishes `/rover/tof/front`, `front_left`, `front_right`, `l8_cols`, `left`, `right`.

---

# Flash

```bash
# Drive board: build, then OTA directly (allow_drive OFF first)
pio run -e s3_tdisplay_microros_rover_drive
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i 192.168.3.71 -p 3232 -f .pio/build/s3_tdisplay_microros_rover_drive/firmware.bin

# ToF board
pio run -e um_pros3_tof_microros -t upload        # USB
pio run -e um_pros3_tof_microros_ota -t upload    # OTA (rover-tof.local)
```

WiFi credentials: `platformio_private.ini` (gitignored).

## Related

- [`lego-rover-ros2/ROVER_BEHAVIOR.md`](../lego-rover-ros2/ROVER_BEHAVIOR.md) — explore behaviour
- [`lego-rover-ros2/BUTTON.md`](../lego-rover-ros2/BUTTON.md) — Go / E-stop behaviour
