#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(cd "$(dirname "$0")" && pwd)/gen_rover_greetings.py" "$@"
