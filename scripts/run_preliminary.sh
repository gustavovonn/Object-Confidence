#!/usr/bin/env bash
# Preliminary experiment: one subset per VeReMi Extension position/speed malfunction,
# no sensor errors, replayed through the engine, then scored by scripts/detect_eval.py and
# scripts/association_eval.py.
# Fault parameters: libs/Artery-Dataset-Generator/config/faults.toml, `types` replaced per subset.
# Honest, with no faulty vehicle (empty `types`), is the fault-free baseline.
# Default ground truth: the generator's CONFIG run RUN, data/groundtruth/<CONFIG>-run<RUN>; when missing
# it is simulated first with the generator's scripts (docker setup, build, run).
# Usage: [CONFIG=lust-full] [RUN=0] scripts/run_preliminary.sh [groundtruth dir] [name]

set -euo pipefail
cd "$(dirname "$0")/.."

GENERATOR=$(realpath "${GENERATOR:-libs/Artery-Dataset-Generator}")
CONFIG=${CONFIG:-lust-full}  # generator omnetpp.ini config
RUN=${RUN:-0}                # its run number (seed)
GT=$(realpath -m "${1:-$GENERATOR/data/groundtruth/$CONFIG-run$RUN}")
OUT=$(realpath -m "results/${2:-$(basename "$GT")}")

if [ ! -d "$GT" ]; then
    [ -z "${1:-}" ] || { echo "missing $GT" >&2; exit 1; }
    echo "missing $GT: simulating config $CONFIG, run $RUN"
    "$GENERATOR/scripts/setup_docker.sh"
    "$GENERATOR/scripts/build.sh"
    "$GENERATOR/scripts/run.sh" -c "$CONFIG" -r "$RUN"
fi
PYTHON=.venv/bin/python   # detect_eval.py needs matplotlib (requirements.txt)
[ -x "$PYTHON" ] || { echo "missing $PYTHON: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2; exit 1; }
TYPES=(Honest ConstPos ConstPosOffset RandomPos RandomPosOffset ConstSpeed ConstSpeedOffset RandomSpeed RandomSpeedOffset)

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
types = "[]" if kind == "Honest" else f'["{kind}"]'
text = re.sub(r'^types = \[.*?\]', f'types = {types}', text, flags=re.M | re.S)
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
