#!/usr/bin/env bash
# Preliminary experiment: one subset per VeReMi Extension position/speed malfunction,
# no sensor errors, replayed through the engine, then scored by scripts/detect_eval.py and
# scripts/association_eval.py.
# Fault parameters: libs/Artery-Dataset-Generator/config/faults.toml, `types` replaced per subset.
# Usage: scripts/run_preliminary.sh [groundtruth dir] [name]

set -euo pipefail
cd "$(dirname "$0")/.."

GENERATOR=$(realpath "${GENERATOR:-libs/Artery-Dataset-Generator}")
GT=$(realpath "${1:-$GENERATOR/data/groundtruth/lust-full-run0}")
OUT=$(realpath -m "results/${2:-$(basename "$GT")}")
PYTHON=.venv/bin/python   # detect_eval.py needs matplotlib (requirements.txt)
[ -x "$PYTHON" ] || { echo "missing $PYTHON: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2; exit 1; }
TYPES=(ConstPos ConstPosOffset RandomPos RandomPosOffset ConstSpeed ConstSpeedOffset RandomSpeed RandomSpeedOffset)

for type in "${TYPES[@]}"; do
    dir="$OUT/$type"
    subset="$GENERATOR/data/subsets/$(basename "$GT")-$type"
    mkdir -p "$dir"
    rm -rf "$subset"
    python3 - "$GENERATOR/config/faults.toml" "$dir/faults.toml" "$GT" "$subset" "$type" <<'EOF'
import re, sys
src, dst, gt, subset, kind = sys.argv[1:]
text = open(src).read()
text = re.sub(r'^input = .*$', f'input = "{gt}"', text, flags=re.M)
text = re.sub(r'^output = .*$', f'output = "{subset}"', text, flags=re.M)
text = re.sub(r'^types = \[.*?\]', f'types = ["{kind}"]', text, flags=re.M | re.S)
text = re.sub(r'\[sensor_errors\].*?(?=\n\[)', '', text, flags=re.S)
open(dst, "w").write(text)
EOF
    python3 "$GENERATOR/scripts/inject_faults.py" "$dir/faults.toml" >/dev/null
    python3 "$GENERATOR/scripts/receiver_view.py" "$subset" >/dev/null
    python3 scripts/flatten_dataset.py "$subset" "$dir/receptions.csv"
    build/replay "$dir/receptions.csv" "$dir/records.csv" >/dev/null
    [ "${KEEP_RECEPTIONS:-0}" = "1" ] || rm -f "$dir/receptions.csv"
    echo "$type done"
done

"$PYTHON" scripts/detect_eval.py "$OUT"
"$PYTHON" scripts/association_eval.py "$OUT"
