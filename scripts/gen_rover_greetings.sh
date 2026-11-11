#!/usr/bin/env bash
# Regenerate Rover AI-cam greeting MP3s (local espeak-ng + ffmpeg).
set -euo pipefail
export LD_LIBRARY_PATH="${HOME}/.local/piper${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ESPEAK_DATA_PATH="${HOME}/.local/piper/espeak-ng-data"
export PATH="${HOME}/.local/bin:${PATH}"
ESP="${HOME}/.local/piper/espeak-ng"
DEST="$(cd "$(dirname "$0")/.." && pwd)/data"
mkdir -p "$DEST" /tmp/rover-greet-wav
say() {
  local name="$1" text="$2"
  "$ESP" -v en-gb -s 145 -p 45 -g 4 -w "/tmp/rover-greet-wav/${name}.wav" "$text"
  ffmpeg -y -hide_banner -loglevel error -i "/tmp/rover-greet-wav/${name}.wav" \
    -af "silenceremove=start_periods=1:start_silence=0.05:start_threshold=-38dB:detection=peak,areverse,silenceremove=start_periods=1:start_silence=0.08:start_threshold=-38dB:detection=peak,areverse,apad=pad_dur=0.08" \
    -ac 1 -ar 44100 -b:a 128k "$DEST/${name}.mp3"
}
say hi "Hi."
say morning "Good morning,"
say afternoon "Good afternoon,"
say evening "Good evening,"
say hello "Hello,"
say im_rover "I'm Rover."
say my_battery_is "My battery is"
say percent "percent."
for pair in \
  "n0:zero" "n1:one" "n2:two" "n3:three" "n4:four" "n5:five" "n6:six" "n7:seven" \
  "n8:eight" "n9:nine" "n10:ten" "n11:eleven" "n12:twelve" "n13:thirteen" "n14:fourteen" \
  "n15:fifteen" "n16:sixteen" "n17:seventeen" "n18:eighteen" "n19:nineteen" \
  "n20:twenty" "n30:thirty" "n40:forty" "n50:fifty" "n60:sixty" "n70:seventy" \
  "n80:eighty" "n90:ninety" "n100:one hundred"
do
  say "${pair%%:*}" "${pair#*:}"
done
echo "Wrote clips in $DEST"
