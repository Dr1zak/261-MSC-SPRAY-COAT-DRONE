#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
scenario=baseline
backend=isaac
while (($#)); do
  case "$1" in
    --scenario) scenario="$2"; shift 2;;
    --backend) backend="$2"; shift 2;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
if [[ "$backend" == test ]]; then
  PYTHONPATH=src/construction_autonomy python3 -m construction_autonomy.cli --scenario "$scenario"
else
  echo 'Isaac demo unavailable: model conversion, physical controller, ROS bridge, and mission adapter are not implemented. Use --backend test for algorithm-only output.' >&2
  exit 2
fi
