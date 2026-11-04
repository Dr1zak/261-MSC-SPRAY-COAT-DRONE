#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
airframe=x650
arguments=("$@")
for ((i=0;i<${#arguments[@]};i++)); do
  if [[ "${arguments[i]}" == --airframe ]]; then airframe="${arguments[i+1]}"; fi
done
if [[ "$airframe" == x650 && ! -s assets/generated/x650_asset_manifest.json ]]; then
  ./scripts/convert_x650_model.sh
fi
./scripts/run_isaac_scene.sh --airframe "$airframe" --scenario construction_site --headless --frames 0 --no-ros-clock
"$isaac_root/python.sh" scripts/animate_construction_flight.py --workspace "$PWD" "$@"
