#!/usr/bin/env python3
"""Detect frozen ToF diagonal ranges in brain-captured JSONL (e.g. G6 baseline)."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path


def analyze(path: Path) -> int:
    kinds = Counter()
    uniq: dict[str, set[float]] = defaultdict(set)
    stamps: dict[str, list[float]] = defaultdict(list)

    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        o = json.loads(line)
        kind = o["kind"]
        kinds[kind] += 1
        if kind not in ("front_left", "front_right", "left", "right"):
            continue
        data = o["data"]
        r = data.get("range")
        if r is not None and isinstance(r, float) and math.isfinite(r):
            uniq[kind].add(r)
        stamps[kind].append(float(data.get("stamp", 0.0)))

    print(f"file: {path}")
    print("counts:", dict(kinds))
    for k in ("front_left", "front_right", "left", "right"):
        n_msg = kinds[k]
        n_uniq = len(uniq[k])
        deltas = [stamps[k][i] - stamps[k][i - 1] for i in range(1, len(stamps[k]))]
        med_delta = sorted(deltas)[len(deltas) // 2] if deltas else 0.0
        print(f"{k}: msgs={n_msg} unique_ranges={n_uniq} median_stamp_delta_s={med_delta:.4f}")
        if n_msg >= 20 and n_uniq == 1:
            print(f"  ** frozen diagonal: single range {next(iter(uniq[k])):.6f} m with fresh stamps")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", type=Path)
    args = ap.parse_args(argv)
    return analyze(args.jsonl)


if __name__ == "__main__":
    sys.exit(main())
