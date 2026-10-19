#!/usr/bin/env bash
# Preliminary experiment: one subset per VeReMi Extension position/speed malfunction,
# no sensor errors, replayed through the engine.
# Fault parameters: libs/Artery-Dataset-Generator/config/faults.toml, `types` replaced per subset.
# Usage: scripts/run_preliminary.sh [groundtruth dir] [output dir]
# GENERATOR overrides the generator checkout (default: the libs/ submodule)
set -euo pipefail
cd "$(dirname "$0")/.."

GENERATOR=$(realpath "${GENERATOR:-libs/Artery-Dataset-Generator}")
GT=$(realpath "${1:-$GENERATOR/data/groundtruth/lust-run0}")
OUT=$(realpath -m "${2:-results/preliminary}")
TYPES=(ConstPos ConstPosOffset RandomPos RandomPosOffset ConstSpeed ConstSpeedOffset RandomSpeed RandomSpeedOffset)

for type in "${TYPES[@]}"; do
    dir="$OUT/$type"
    mkdir -p "$dir"
    python3 - "$GENERATOR/config/faults.toml" "$dir/faults.toml" "$GT" "$dir/subset" "$type" <<'EOF'
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
    python3 scripts/flatten_dataset.py "$dir/subset" "$dir/receptions.csv"
    build/replay "$dir/receptions.csv" "$dir/records.csv" >/dev/null
    # keep only the records
    rm -rf "$dir/subset" "$dir/receptions.csv"
    echo "$type done"
done
