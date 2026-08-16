# Sanitize CFLAGS/CCFLAGS before micro_ros_platformio builds the MCU libs.
# Flatten nested tuples/lists (PlatformIO -include flags) so cmake join works.
# Invalidate cached libmicroros when colcon user meta changes (PIO caches forever).

Import("env")

import os
import re
import shutil


def _flatten_env_flags(name):
    try:
        flat = [str(x) for x in env.Flatten(env.get(name, []))]
    except Exception:
        flat = []
        for item in env.get(name, []):
            if isinstance(item, (list, tuple)):
                flat.extend(str(x) for x in item)
            elif item is not None:
                flat.append(str(item))
    clean = []
    for part in flat:
        part = part.strip()
        if not part or "CHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER" in part:
            continue
        for chunk in part.replace(";", " ").split():
            if chunk and "CHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER" not in chunk:
                clean.append(chunk)
    env.Replace(**{name: clean})


def _meta_limit(meta_path, key):
    try:
        with open(meta_path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return None
    match = re.search(rf"-D{key}=(\d+)", text)
    return int(match.group(1)) if match else None


def _built_limit(config_h, key):
    try:
        with open(config_h, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return None
    match = re.search(rf"#define {key} (\d+)", text)
    return int(match.group(1)) if match else None


def _invalidate_stale_microros():
    project_dir = env.subst("$PROJECT_DIR")
    pioenv = env.get("PIOENV", "")
    if not pioenv:
        return

    try:
        user_meta_rel = env.BoardConfig().get("microros_user_meta", "")
    except Exception:
        return
    if not user_meta_rel:
        return

    user_meta = os.path.join(project_dir, user_meta_rel)
    if not os.path.isfile(user_meta):
        return

    lib_root = os.path.join(
        project_dir, ".pio", "libdeps", pioenv, "micro_ros_platformio"
    )
    lib_a = os.path.join(lib_root, "libmicroros", "libmicroros.a")
    config_h = os.path.join(
        lib_root,
        "build",
        "mcu",
        "install",
        "include",
        "rmw_microxrcedds_c",
        "config.h",
    )
    if not os.path.isfile(lib_a):
        return

    stale = os.path.getmtime(user_meta) > os.path.getmtime(lib_a)
    want_pub = _meta_limit(user_meta, "RMW_UXRCE_MAX_PUBLISHERS")
    want_sub = _meta_limit(user_meta, "RMW_UXRCE_MAX_SUBSCRIPTIONS")
    if os.path.isfile(config_h):
        have_pub = _built_limit(config_h, "RMW_UXRCE_MAX_PUBLISHERS")
        have_sub = _built_limit(config_h, "RMW_UXRCE_MAX_SUBSCRIPTIONS")
        if want_pub is not None and have_pub is not None and want_pub != have_pub:
            stale = True
        if want_sub is not None and have_sub is not None and want_sub != have_sub:
            stale = True

    if not stale:
        return

    print(
        "micro-ROS: colcon user meta changed — deleting cached libmicroros for rebuild"
    )
    shutil.rmtree(os.path.join(lib_root, "libmicroros"), ignore_errors=True)
    shutil.rmtree(os.path.join(lib_root, "build"), ignore_errors=True)


_flatten_env_flags("CFLAGS")
_flatten_env_flags("CCFLAGS")
_flatten_env_flags("CXXFLAGS")
_invalidate_stale_microros()
