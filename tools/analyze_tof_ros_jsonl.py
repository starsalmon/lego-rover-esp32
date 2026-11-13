#!/usr/bin/env python3
"""Analyze raw ROS JSONL captures for Pro S3 ToF stale-republish signatures."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path


def analyze(path: Path) -> int:
    topics: Counter[str] = Counter()
    fl: set[float] = set()
    fr: set[float] = set()
    fc: set[float] = set()
    cols_sigs: set[tuple] = set()
    grid_sigs: set[tuple] = set()
    fl_stamps: list[tuple[int, int]] = []
    status_lens: Counter[int] = Counter()
    right_vals: list[float] = []
    t0 = t1 = None

    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        o = json.loads(line)
        topic = o.get("topic")
        if not topic:
            continue
        topics[topic] += 1
        msg = o.get("message", {})
        ep = o.get("epoch")
        if ep is not None:
            t0 = ep if t0 is None else t0
            t1 = ep

        if topic == "/rover/tof/front_left":
            fl.add(float(msg["range"]))
            h = msg["header"]["stamp"]
            fl_stamps.append((int(h["sec"]), int(h["nanosec"])))
        elif topic == "/rover/tof/front_right":
            fr.add(float(msg["range"]))
        elif topic == "/rover/tof/front":
            fc.add(float(msg["range"]))
        elif topic == "/rover/tof/right":
            r = msg["range"]
            if isinstance(r, (int, float)) and math.isfinite(r):
                right_vals.append(float(r))
        elif topic == "/rover/tof/l8_cols":
            cols_sigs.add(tuple(msg.get("data", [])))
        elif topic == "/rover/tof/l8_grid":
            grid_sigs.add(tuple(msg.get("data", [])))
        elif topic == "/rover/tof/status":
            status_lens[len(msg.get("data", []))] += 1

    dur = (t1 - t0) if t0 is not None and t1 is not None else 0.0
    print(f"file: {path}")
    print(f"duration_s: {dur:.2f}")
    tof = {k: v for k, v in sorted(topics.items()) if "/tof/" in k}
    print("tof_msg_counts:", tof)
    print(f"front_left: msgs={topics['/rover/tof/front_left']} unique={len(fl)}")
    print(f"front_right: msgs={topics['/rover/tof/front_right']} unique={len(fr)}")
    print(f"l8_cols: msgs={topics['/rover/tof/l8_cols']} unique_arrays={len(cols_sigs)}")
    print(f"l8_grid: msgs={topics['/rover/tof/l8_grid']} unique_arrays={len(grid_sigs)}")
    print(f"right: unique_finite={len(set(right_vals))} (side bus sanity check)")
    print(f"/rover/tof/status lens: {dict(status_lens)}")
    if fl_stamps:
        span = max(s[0] for s in fl_stamps) - min(s[0] for s in fl_stamps)
        print(f"front_left header stamp span_sec={span} unique_headers={len(set(fl_stamps))}")

    stale_signature = (
        topics["/rover/tof/front_left"] >= 20
        and len(fl) == 1
        and len(cols_sigs) == 1
        and len(set(fl_stamps)) > 20
    )
    if stale_signature:
        print("** stale-republish signature: frozen L8 payload + advancing Range headers")
    if 11 not in status_lens and 6 in status_lens:
        print("** deployed status has 6 floats only (no L8 diag [6..10]); likely pre-b6e41ca or truncated capacity")
    return 1 if stale_signature else 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", type=Path)
    return analyze(ap.parse_args(argv).jsonl)


if __name__ == "__main__":
    sys.exit(main())
