# Copy net-locked WiFi micro-ROS transport over libdeps copy (DIAG/uros share rover_net_*).
Import("env")

import os
import shutil

MARKER = "rover_net_lock"
TEMPLATE = os.path.join(env.subst("$PROJECT_DIR"), "patches", "micro_ros_transport_wifi_netlock.cpp")


def _patch():
    pioenv = env.get("PIOENV", "")
    if not pioenv:
        return
    # Only wifi-transport rover envs need this.
    try:
        transport = str(env.BoardConfig().get("microros_transport", ""))
    except Exception:
        transport = ""
    # board_microros_transport is a board option set in platformio.ini
    # Also gate on ROVER_MICROROS_WIFI in CPPDEFINES
    defs = [str(x) for x in env.Flatten(env.get("CPPDEFINES", []))]
    want = any("ROVER_MICROROS_WIFI" in d for d in defs) or transport == "wifi"
    # platformio stores board_microros_transport differently — check lib path exists
    dest = os.path.join(
        env.subst("$PROJECT_DIR"),
        ".pio",
        "libdeps",
        pioenv,
        "micro_ros_platformio",
        "platform_code",
        "arduino",
        "wifi",
        "micro_ros_transport.cpp",
    )
    if not os.path.isfile(TEMPLATE):
        print("micro-ROS netlock: template missing", TEMPLATE)
        return
    if not os.path.isfile(dest):
        # lib not installed yet — hook again after deps? pre script may run before lib extract
        print("micro-ROS netlock: transport not present yet:", dest)
        return
    try:
        with open(dest, encoding="utf-8") as f:
            cur = f.read()
    except OSError:
        return
    if MARKER in cur:
        return
    shutil.copyfile(TEMPLATE, dest)
    print("micro-ROS netlock: patched", dest)


_patch()
