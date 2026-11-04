#!/usr/bin/env bash
set -u
cd "$(dirname "$0")/.."
for program in python3 ros2 colcon freecad nvidia-smi; do command -v "$program" || true; done
python3 --version
if command -v nvidia-smi >/dev/null; then nvidia-smi -L || true; fi
for path in "$HOME/isaacsim" /opt/isaac-sim; do
  if test -d "$path"; then
    echo "Isaac candidate: $path"
    if test -f "$path/VERSION"; then echo "Isaac version: $(cat "$path/VERSION")"; fi
  fi
done
