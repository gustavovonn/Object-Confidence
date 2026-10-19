#!/usr/bin/env bash
# Preliminary experiment on one ground-truth run, one subset per fault type as VeReMi Extension
# (Sec. IV-D) and paper 1:
#   faults         VeReMi Extension CAM malfunction only
#   faults_sensor  malfunction + VeReMi Extension sensor errors
# Fault parameters: Artery-Dataset-Generator/config/faults.toml, `types` replaced per subset.
# Usage: scripts/run_preliminary.sh [groundtruth dir] [output dir]
set -euo pipefail
cd "$(dirname "$0")/.."

GENERATOR=../Artery-Dataset-Generator
GT=$(realpath "${1:-$GENERATOR/data/groundtruth/lust-run0}")
OUT=$(realpath -m "${2:-results/preliminary}")
TYPES=(ConstPos ConstPosOffset RandomPos RandomPosOffset ConstSpeed ConstSpeedOffset RandomSpeed RandomSpeedOffset)

for variant in faults faults_sensor; do
    for type in "${TYPES[@]}"; do
        dir="$OUT/$variant/$type"
        mkdir -p "$dir"
        python3 - "$GENERATOR/config/faults.toml" "$dir/faults.toml" "$GT" "$dir/subset" "$variant" "$type" <<'EOF'
import re, sys
src, dst, gt, subset, variant, kind = sys.argv[1:]
text = open(src).read()
text = re.sub(r'^input = .*$', f'input = "{gt}"', text, flags=re.M)
text = re.sub(r'^output = .*$', f'output = "{subset}"', text, flags=re.M)
text = re.sub(r'^types = \[.*?\]', f'types = ["{kind}"]', text, flags=re.M | re.S)
if variant == "faults":
    text = re.sub(r'\[sensor_errors\].*?(?=\n\[)', '', text, flags=re.S)
open(dst, "w").write(text)
EOF
        python3 "$GENERATOR/scripts/inject_faults.py" "$dir/faults.toml" >/dev/null
        python3 scripts/flatten_dataset.py "$dir/subset" "$dir/receptions.csv"
        build/replay "$dir/receptions.csv" "$dir/records.csv" >/dev/null
        python3 scripts/evaluate.py "$dir/records.csv" > "$dir/summary.txt"
        # keep only the records and summary
        rm -rf "$dir/subset" "$dir/receptions.csv"
        echo "$variant/$type done"
    done
done
