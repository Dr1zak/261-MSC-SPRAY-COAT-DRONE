#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
scenario=baseline
airframe=x650
arguments=("$@")
for ((i=0;i<${#arguments[@]};i++)); do
  if [[ "${arguments[i]}" == --scenario ]]; then scenario="${arguments[i+1]}"; fi
  if [[ "${arguments[i]}" == --airframe ]]; then airframe="${arguments[i+1]}"; fi
done
stamp="$(mktemp)"
trap 'rm -f "$stamp"' EXIT
"$isaac_root/python.sh" scripts/run_isaac_site.py --workspace "$PWD" "$@"
scene="assets/generated/${airframe}_site_${scenario}.usd"
if [[ ! -s "$scene" || ! "$scene" -nt "$stamp" ]]; then
  echo "Isaac did not produce a fresh site scene: $scene" >&2
  exit 2
fi
