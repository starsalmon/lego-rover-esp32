"""Fail closed when Pro S3 micro-ROS ships without compile-time Wi-Fi credentials."""
Import("env")

env_name = env.get("PIOENV", "")
if env_name not in ("um_pros3_tof_microros", "um_pros3_tof_microros_ota"):
    pass
else:
    flags = " ".join(env.get("BUILD_FLAGS", []))
    if "ROVER_WIFI_SSID" not in flags:
        raise SystemExit(
            f"{env_name}: missing -DROVER_WIFI_SSID in build flags. "
            "Add platformio_private.ini ([rover_wifi]) before building ToF firmware."
        )
