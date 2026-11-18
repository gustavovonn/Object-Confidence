#!/usr/bin/env bash
# Opens sumo-gui (no Artery) LEAD s before an episode's onset, tracking the vehicle, so it can be watched.
# The generator's dataset time t is OMNeT time; the state recorded at t is the latest SUMO step at or
# before it, t0 + floor(t / 0.1) * 0.1, t0 the run's SUMO time of t = 0 (scripts/common.py sumo_clock: the
# run's --begin plus its warm-up; checked against TraCI positions: identical). The scenario is the run's.
# Station -> SUMO id from the ground truth's vehicles.jsonl. Runs in the generator's artery-dev image
# (scripts/setup_docker.sh), which has sumo-gui and TraCI; X access for containers once per session:
#   xhost +local:
# Usage: [GT=<ground truth dir>] [SEED=23423] scripts/view_in_sumo.sh <station> <t [s]> [lead [s], default 3]
#   e.g. the station and t columns of results/<run>/Honest/error_diagnosis/episodes.csv
set -euo pipefail
cd "$(dirname "$0")/.."
[ $# -ge 2 ] || { sed -n '2,9p' "$0"; exit 1; }
STATION=$1
T=$2
LEAD=${3:-3}
GENERATOR=$(realpath libs/Artery-Dataset-Generator)
GT=$(realpath "${GT:-$GENERATOR/data/groundtruth/lust-full-run0}")
SEED=${SEED:-23423}  # SUMO seed of the generator's runs

SUMO_ID=$(python3 - "$GT/vehicles.jsonl" "$STATION" <<'EOF'
import json, sys
for line in open(sys.argv[1]):
    v = json.loads(line)
    if v["station"] == int(sys.argv[2]):
        print(v["sumoId"]); break
else:
    sys.exit(f"station {sys.argv[2]} not in {sys.argv[1]}")
EOF
)
read -r BEGIN T0 SUMOCFG < <(python3 scripts/common.py sumo-clock "$GT")
ONSET=$(python3 -c "import math; print(round($T0 + math.floor($T / 0.1 + 1e-6) * 0.1, 1))")
echo "station $STATION = SUMO $SUMO_ID, onset at SUMO time $ONSET"

docker run --rm -it --network none -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$GENERATOR":/workspace:ro -v "$PWD/scripts/sumo_follow.py":/sumo_follow.py:ro \
    artery-dev bash -c '
        # the scenario writes its detector outputs next to its files: a writable copy
        cp -r "/workspace/$(dirname "$1")" /tmp/scenario && cd /tmp/scenario && shift &&
        python3 /sumo_follow.py "$@"' _ "$SUMOCFG" "$SUMO_ID" "$ONSET" "$LEAD" "$SEED" "$BEGIN" "$(basename "$SUMOCFG")"
