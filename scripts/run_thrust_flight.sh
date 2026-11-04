#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
mode=hover
arguments=("$@")
for ((i=0;i<${#arguments[@]};i++)); do
  if [[ "${arguments[i]}" == --mode ]]; then mode="${arguments[i+1]}"; fi
done
stamp="$(mktemp)"
trap 'rm -f "$stamp"' EXIT
./scripts/run_isaac_scene.sh --airframe x500 --scenario construction_site --headless --frames 0 --no-ros-clock
"$isaac_root/python.sh" scripts/run_thrust_flight.py --workspace "$PWD" --airframe x500 "$@"
summary="reports/thrust_${mode}_summary.json"
if [[ ! -s "$summary" || ! "$summary" -nt "$stamp" ]]; then
  echo "Isaac did not write a fresh thrust-flight summary: $summary" >&2
  exit 2
fi
python3 - "$summary" <<'PY'
import json, sys
report = json.load(open(sys.argv[1]))
if not report.get('success'):
    raise SystemExit('Thrust-flight check did not meet its stability or route criteria')
PY
