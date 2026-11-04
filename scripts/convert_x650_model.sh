#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
python3 scripts/build_x650_model.py
"$isaac_root/python.sh" scripts/import_x650_isaac.py --workspace "$PWD" --isaac-root "$isaac_root"
python3 - <<'PY'
import json
from pathlib import Path
record=json.loads(Path('assets/generated/x650_asset_manifest.json').read_text())
usd=Path(record['usd_path'])
if not usd.is_file() or usd.stat().st_size == 0:
    raise SystemExit(f'X650 USD missing: {usd}')
print(f'Verified X650 USD: {usd}')
PY
