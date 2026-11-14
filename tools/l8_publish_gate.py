#!/usr/bin/env python3
"""Policy for when Pro S3 firmware may publish VL53L8-derived ROS ranges.

Stale symptom (G6 baseline): front_left/front_right stamps advance ~10 Hz while
range floats are bit-identical — publish loop stamped millis() on cached g_l8_*.
"""


def should_publish_l8(*, frames: int, last_pub_frame: int) -> bool:
    """Publish only when at least one new L8 frame was consumed since last publish."""
    if frames <= 0:
        return False
    return frames != last_pub_frame


def stamp_ms_for_l8(*, sample_ms: int, publish_now_ms: int) -> int:
    """Measurement stamp must be the sample time, not the republish tick."""
    return sample_ms if sample_ms > 0 else publish_now_ms


def test_gate_skips_republish():
    assert not should_publish_l8(frames=5, last_pub_frame=5)
    assert should_publish_l8(frames=6, last_pub_frame=5)


def test_gate_waits_for_first_frame():
    assert not should_publish_l8(frames=0, last_pub_frame=0)
    assert should_publish_l8(frames=1, last_pub_frame=0)


def test_stamp_uses_sample_time():
    assert stamp_ms_for_l8(sample_ms=3502, publish_now_ms=9999) == 3502


if __name__ == "__main__":
    test_gate_skips_republish()
    test_gate_waits_for_first_frame()
    test_stamp_uses_sample_time()
    print("ok")
