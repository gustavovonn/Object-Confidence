# Object Confidence

Per-object confidence for CAM and CPM motion vectors. Each report is associated to a fused track (Hungarian
algorithm, elliptical gate), checked against the track's projection (UKF with CTRA; OLS-CTRA without the filter),
and scored on the track and on its author (Beta reputation).

## Setup

```sh
git submodule update --init
cmake -S . -B build && cmake --build build -j2
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

Ground truth comes from `libs/Artery-Dataset-Generator`, which has its own README: configs `veremi-val` (validation,
15:30–15:45) and `veremi-test` (test, 08:00–08:15), on the VeReMi Extension network and demand.

## Protocol

Calibrate on the validation window. Evaluate on the test window. The calibration scripts refuse the test window.

| Calibration (validation) | Sets |
|---|---|
| `scripts/flatten_truth.py <gt> <dir>` + `build/calibrate_ukf <dir> <dir>/ukf_calibration.md` | UKF Q and P0 (`ConstantsUkf`) |
| `scripts/calib_maxposdev.py` | residual quantiles behind `DEFAULT_MAX_POS_DEV` |
| `scripts/sensitivity.py` | frozen bound, retirement time and `maxPosDev` sweeps |

```sh
.venv/bin/python reproduce_paper.py   # evaluation on veremi-test, lowest CPU/I/O priority, one process at a time
```

| Step | Script | Output |
|---|---|---|
| 1 | `kalman_eval.py` | Kalman vs OLS-CTRA on Honest per white position noise: `results/kalman-eval-<gt>/summary.md` |
| 2 | `error_diagnosis.py` | confidence drops of two of those cells, physically possible or not: `<cell>/error_diagnosis/` |
| 3 | `association_eval.py` | association quality of the same cells: `association.md` |
| 4 | `score_paradigm_ablation.py` | detection, fate, association and ROC on Honest and the 8 VeReMi fault subsets, per confidence paradigm: `results/veremi-eval-<gt>/` |

Standard condition: the VeReMi Extension sensor errors, with the UKF position R from their jitter
(`STANDARD` in `scripts/veremi_eval.py`). Records are kept gzipped (`records.csv.gz`).

## Latency on the VisionFive 2

`replay --profile` reports the latency per engine stage (mean, p50, p99, max), and `--csv-out` appends it as
CSV. `vf2-setup/vf2_setup.sh` cross-compiles a static `replay`, stages receptions and netboots the board:

```sh
KEEP_RECEPTIONS=1 python3 scripts/veremi_eval.py   # keeps results/veremi-eval-<gt>/receptions/<subset>.csv
./vf2-setup/vf2_setup.sh [--skip-tftp] [--results <receptions dir>]
```

On the board: `cd /bench && ./run_benchmarks_riscv.sh [--profile <subset>] [--repeat N] [--core <id>] [--max-msgs N]`.

| Stage | Per | Work |
|---|---|---|
| `0 retire` | message | drop stale tracks and their scores |
| `1 plausibility` | message | finite-value check |
| `2 associate` | message | linked sources keep their track; the rest by the Hungarian algorithm, gated |
| `3 checks` | vector with a track | one projection to the vector time; speed, frozen and predictive checks |
| `4 kalmanUpdate` | fused vector | UKF predict and update; out-of-sequence vectors are not fused |
| `5 score` | vector | hit or miss on the track and, by `--confidence`, on the author |

`onMotionVectors` times the whole call, including `initiateTrack`.
