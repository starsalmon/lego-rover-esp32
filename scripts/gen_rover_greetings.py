#!/usr/bin/env python3
"""Regenerate Rover AI-cam greeting MP3s with Piper en_GB-alan-medium (rover-speech)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEST = ROOT / "data"
ONES = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen",
    "seventeen", "eighteen", "nineteen",
]
TENS = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]


def num_words(n):
    if n < 20:
        return ONES[n]
    if n == 100:
        return "one hundred"
    t, u = divmod(n, 10)
    return TENS[t] if u == 0 else f"{TENS[t]}-{ONES[u]}"


PHRASES = {
    "hi_morning": "Hi. Good morning, I'm Rover.",
    "hi_afternoon": "Hi. Good afternoon, I'm Rover.",
    "hi_evening": "Hi. Good evening, I'm Rover.",
    "hello_rover": "Hello, I'm Rover.",
}
for n in range(0, 101):
    PHRASES[f"batt_{n}"] = f"My battery is {num_words(n)} percent."

INNER = r"""
import os, subprocess, sys
VOICE = "/voices/en_GB-alan-medium.onnx"
AF = "acompressor=threshold=0.05:ratio=6:attack=3:release=60,volume=20dB,alimiter=limit=0.84:attack=2:release=60:level=false,apad=pad_dur=0.08"
OUT = "/tmp/rover-greet-alan"
os.makedirs(OUT, exist_ok=True)
failed = 0
for line in sys.stdin:
    line = line.rstrip("\n")
    if not line or "|" not in line:
        continue
    name, text = line.split("|", 1)
    wav = f"{OUT}/{name}.wav"
    mp3 = f"{OUT}/{name}.mp3"
    r = subprocess.run(["piper", "-m", VOICE, "-f", wav], input=text.encode(), capture_output=True, timeout=30)
    if r.returncode != 0:
        sys.stderr.write(f"piper fail {name}: {r.stderr.decode(errors='replace')}\n")
        failed += 1
        continue
    r = subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                        "-i", wav, "-af", AF, "-ac", "1", "-ar", "22050", "-b:a", "48k", mp3],
                       capture_output=True, timeout=30)
    if r.returncode != 0:
        sys.stderr.write(f"ffmpeg fail {name}: {r.stderr.decode(errors='replace')}\n")
        failed += 1
        continue
    os.remove(wav)
    print(f"ok {name} {os.path.getsize(mp3)}", flush=True)
sys.exit(1 if failed else 0)
"""


def main():
    subprocess.check_call(["docker", "exec", "rover-speech", "test", "-f", "/voices/en_GB-alan-medium.onnx"])
    Path("/tmp/gen_alan_inner.py").write_text(INNER)
    subprocess.check_call(["docker", "cp", "/tmp/gen_alan_inner.py", "rover-speech:/tmp/gen_alan_inner.py"])
    lines = "\n".join(f"{k}|{v}" for k, v in PHRASES.items()) + "\n"
    p = subprocess.run(
        ["docker", "exec", "-i", "rover-speech", "python3", "/tmp/gen_alan_inner.py"],
        input=lines.encode(),
    )
    if p.returncode != 0:
        sys.exit(p.returncode)
    subprocess.check_call(["rm", "-rf", "/tmp/rover-greet-alan-host"])
    subprocess.check_call(["docker", "cp", "rover-speech:/tmp/rover-greet-alan", "/tmp/rover-greet-alan-host"])
    DEST.mkdir(parents=True, exist_ok=True)
    for pth in DEST.glob("*.mp3"):
        if pth.name != "chime.mp3":
            pth.unlink()
    n = 0
    for pth in Path("/tmp/rover-greet-alan-host").glob("*.mp3"):
        if pth.name == "alan_test.mp3":
            continue
        (DEST / pth.name).write_bytes(pth.read_bytes())
        n += 1
    print(f"Wrote {n} clips in {DEST}")


if __name__ == "__main__":
    main()
