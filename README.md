# Object Confidence

Per-object confidence for CAM and CPM motion vectors. Each report is associated to a fused track (Hungarian
algorithm, gated by the vehicles' boxes), checked against the track's projection (UKF-CTRA; OLS-CTRA without the
filter), and scored on the track and its author (Beta reputation).

## Setup

```sh
git submodule update --init
cmake -S . -B build && cmake --build build -j2
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

The ground truth comes from `libs/Artery-Dataset-Generator` (VeReMi Extension network and demand): `veremi-val`
(15:30–15:45, calibration) and `veremi-test` (08:00–08:15, evaluation).

## Reproducing the paper

```sh
.venv/bin/python reproduce_paper.py   # simulates what is missing, calibrates on val, evaluates on test
```

| Step | Script | Output |
|---|---|---|
| V1 | `flatten_truth.py`, `calibrate_ukf` | UKF Q and P0: `results/calibration/<val>/` |
| V2 | `calib_maxposdev.py` | predictive bound quantiles: `results/calibration/<val>/` |
| V3 | `sensitivity.py` | frozen, retire and maxPosDev sweeps: `results/sensitivity-<val>/` |
| 1 | `kalman_eval.py` | Kalman vs OLS-CTRA per position noise: `results/kalman-eval-<test>/` |
| 2 | `error_diagnosis.py` | confidence drops, physically possible or not |
| 3 | `association_eval.py` | association quality |
| 4 | `score_paradigm_ablation.py` | detection and ROC on Honest and the 8 faults, per paradigm: `results/veremi-eval-<test>/` |

The calibration only writes reports; the values are fixed in `include/MechanismConstants.h`. Calibration
scripts refuse the test window. `SKIP_VALIDATION=1` skips V1–V3; `JOBS=N` sets the parallel workers (default 3).
Each script's docstring has its details.

Inputs live in `data/` (git-ignored: ground truth, fault subsets, receptions); `results/` keeps only conclusions.

## Cost on the VisionFive 2

```sh
build/replay Honest.csv /dev/null --latency-out latency.csv   # neighbourhood per receiver
scripts/select_receivers.py Honest.csv latency.csv bench.csv --receivers 20
./vf2-setup/vf2_setup.sh --data bench.csv   # cross-compiles, builds the initramfs, netboots
# on the board:
cd /bench && ./run_benchmarks_riscv.sh --core 1 --repeat 3
```

`--bands` prints the latency per band of live tracks, per stage:

| Stage | Work |
|---|---|
| retire | drop stale tracks |
| plausibility | finite-value check |
| associate | linked sources keep their track; the rest by Hungarian, gated |
| checks | projection; speed, frozen, predictive |
| kalmanUpdate | UKF predict and update |
| score | Beta update of track and author |
