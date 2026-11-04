#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
airframe=x650
arguments=("$@")
for ((i=0;i<${#arguments[@]};i++)); do
  if [[ "${arguments[i]}" == --airframe ]]; then airframe="${arguments[i+1]}"; fi
done
if [[ "$airframe" == x650 ]]; then
  exec ./scripts/convert_x650_model.sh
fi
if [[ "$airframe" != x500 ]]; then
  echo "Unknown airframe: $airframe" >&2
  exit 2
fi
isaac_root="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
if [[ ! -x "$isaac_root/python.sh" ]]; then
  echo "Isaac Python missing: $isaac_root/python.sh" >&2
  exit 2
fi
./scripts/prepare_assets.sh
stamp="$(mktemp)"
trap 'rm -f "$stamp"' EXIT
"$isaac_root/python.sh" scripts/import_x500_isaac.py --workspace "$PWD" --isaac-root "$isaac_root"
python3 - "$stamp" <<'PY'
import json
import sys
from pathlib import Path

stamp = Path(sys.argv[1])
manifest_path = Path('assets/generated/asset_manifest.json')
manifest = json.loads(manifest_path.read_text())
record = manifest.get('isaac_import')
if not record or manifest_path.stat().st_mtime_ns <= stamp.stat().st_mtime_ns:
    raise SystemExit('Isaac importer did not record a fresh conversion')
usd = Path(record['usd_path'])
if not usd.is_file() or not usd.stat().st_size:
    raise SystemExit(f'Isaac importer did not produce a readable USD: {usd}')
print(f'Verified Isaac USD: {usd}')
PY
