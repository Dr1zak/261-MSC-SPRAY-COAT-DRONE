#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH=src/construction_autonomy python3 scripts/run_experiments.py "$@"
