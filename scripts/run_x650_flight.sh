#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
if [[ ! -s assets/generated/x650_asset_manifest.json ]]; then
  ./scripts/convert_x650_model.sh
fi
mode=hover
arguments=("$@")
for ((i=0;i<${#arguments[@]};i++)); do
  if [[ "${arguments[i]}" == --mode ]]; then mode="${arguments[i+1]}"; fi
done
stamp="$(mktemp)"
trap 'rm -f "$stamp"' EXIT
./scripts/run_isaac_scene.sh --airframe x650 --scenario construction_site --headless --frames 0 --no-ros-clock
"$isaac_root/python.sh" scripts/run_thrust_flight.py --workspace "$PWD" --airframe x650 "$@"
summary="reports/thrust_x650_${mode}_summary.json"
if [[ ! -s "$summary" || ! "$summary" -nt "$stamp" ]]; then
  echo "Isaac did not write a fresh X650 thrust-flight summary: $summary" >&2
  exit 2
fi
python3 - "$summary" <<'PY'
import json, sys
report = json.load(open(sys.argv[1]))
if not report.get('success'):
    raise SystemExit('X650 thrust-flight check did not meet its stability or route criteria')
PY
