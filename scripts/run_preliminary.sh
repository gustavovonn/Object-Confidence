#!/usr/bin/env bash
# Preliminary experiment on one ground-truth run:
#   faults         VeReMi Extension CAM malfunctions only
#   faults_sensor  malfunctions + VeReMi Extension sensor errors
# Fault parameters: Artery-Dataset-Generator/config/faults.toml.
# Usage: scripts/run_preliminary.sh [groundtruth dir] [output dir]
set -euo pipefail
cd "$(dirname "$0")/.."

GENERATOR=../Artery-Dataset-Generator
GT=$(realpath "${1:-$GENERATOR/data/groundtruth/lust-run0}")
OUT=$(realpath -m "${2:-results/preliminary}")
mkdir -p "$OUT"

for variant in faults faults_sensor; do
    dir="$OUT/$variant"
    mkdir -p "$dir"
    # same faults as the generator config; sensor errors kept only in faults_sensor
    python3 - "$GENERATOR/config/faults.toml" "$dir/faults.toml" "$GT" "$dir/subset" "$variant" <<'EOF'
import re, sys
src, dst, gt, subset, variant = sys.argv[1:]
text = open(src).read()
text = re.sub(r'^input = .*$', f'input = "{gt}"', text, flags=re.M)
text = re.sub(r'^output = .*$', f'output = "{subset}"', text, flags=re.M)
if variant == "faults":
    text = re.sub(r'\[sensor_errors\].*?(?=\n\[)', '', text, flags=re.S)
open(dst, "w").write(text)
EOF
    python3 "$GENERATOR/scripts/inject_faults.py" "$dir/faults.toml"
    python3 scripts/flatten_dataset.py "$dir/subset" "$dir/receptions.csv"
    build/replay "$dir/receptions.csv" "$dir/records.csv"
    python3 scripts/evaluate.py "$dir/records.csv" | tee "$dir/summary.txt"
done
