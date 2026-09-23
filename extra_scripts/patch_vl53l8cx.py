# STM32duino VL53L8CX IO_Read retries endTransmission forever on NACK.
# That holds the rover I2C mutex and kills every device on the bus (sides + L8).
Import("env")
from pathlib import Path

MARKER = "CURSOR_L8_I2C_RETRY"
NEEDLE = """          // End of fix
        } while (status != 0);"""
PATCH = """          // End of fix
          if (status != 0) {
            // CURSOR_L8_I2C_RETRY
            _l8_i2c_tries++;
            if (_l8_i2c_tries > 4) {
              return 1;
            }
          }
        } while (status != 0);"""


def _patch(path: Path) -> None:
    text = path.read_text()
    if MARKER in text:
        return
    if NEEDLE not in text:
        print(f"vl53l8cx: retry needle missing in {path}")
        return
    text = text.replace(
        "        int status = 0;\n        uint8_t buffer[2];\n        // Loop until the port is transmitted correctly\n        do {",
        "        int status = 0;\n        int _l8_i2c_tries = 0;\n        uint8_t buffer[2];\n        // Loop until the port is transmitted correctly\n        do {",
        1,
    )
    text = text.replace(NEEDLE, PATCH, 1)
    path.write_text(text)
    print(f"vl53l8cx: patched infinite I2C retry in {path}")


libdeps = Path(env["PROJECT_LIBDEPS_DIR"])
for header in libdeps.rglob("vl53l8cx.h"):
    _patch(header)
