# Object Confidence

Per-object confidence for CAM and CPM motion vectors: reports are associated to fused tracks
(Hungarian algorithm, `libs/hungarian-algorithm-cpp`; UKF, `libs/ukf`) and each track is scored by how consistently they agree with it.

## How to run

```sh
git submodule update --init
cmake -S . -B build && cmake --build build
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

scripts/run_preliminary.sh [groundtruth dir] [name]   # 8 VeReMi Extension fault subsets -> results/<name>/
```

`results/<name>/` (default name: the ground-truth directory, `lust-full-run0`) holds one directory per
fault subset with its `records.csv`, the detection table `table.md` (all vectors, CAMs, CPM objects) and `confidence_over_time.png`
(`scripts/detect_eval.py`) and the association table `association.md` (`scripts/association_eval.py`).
The fault subsets themselves stay with the generator, in its format:
`libs/Artery-Dataset-Generator/data/subsets/<ground truth>-<fault type>/`.

The default ground truth is `libs/Artery-Dataset-Generator/data/groundtruth/lust-full-run0`
(complete LuST, 40 s from 07:00: `scripts/run.sh -c lust-full` there); generating it needs that
submodule's own submodules and its README.

## Latency benchmark on the VisionFive 2

`replay --profile` reports the latency of each `ObjectConfidenceEngine` stage (mean, p50, p99,
max); `--csv-out` appends it as CSV rows. `vf2-setup/vf2_setup.sh` cross-compiles a static `replay`
(`-DSTATIC_BUILD=ON`, needs `riscv64-linux-gnu-g++`), builds a BusyBox userland, stages the first
`--data-rows` receptions of each fault subset and packs an initramfs netbooted over TFTP:

```sh
KEEP_RECEPTIONS=1 scripts/run_preliminary.sh   # keeps results/<name>/*/receptions.csv
./vf2-setup/vf2_setup.sh                       # prompts before touching the network/TFTP
./vf2-setup/vf2_setup.sh --skip-tftp           # build and package only
./vf2-setup/vf2_setup.sh --results results/lust-full-run0   # default: the newest results/<name> with receptions
```

Stages timed by `--profile` (`src/ObjectConfidenceEngine.cc`):

| Stage | Per | Includes |
|---|---|---|
| `onMotionVectors` | message | the whole call, stages below included |
| `0 retire` | message | drop tracks without a fused report for `retireAfter`, and their scores |
| `1 plausibility` | message | finite-value check of every vector |
| `2 associate` | message | bound sources keep their track; the others jointly by the Hungarian algorithm, gated |
| `3 checks` | vector with a track | track projection to the vector time (UKF `Prediction` on a copy) for the residual, consistency against the source's last report, position gate |
| `4 kalmanUpdate` | fused vector | UKF predict and update (`ProcessMeasurement`) |
| `5 score` | vector | hit or miss on the track's confidence |

Not in any stage, so only in `onMotionVectors`: `initiateTrack` for vectors that start a track.

On the board, run `cd /bench && ./run_benchmarks_riscv.sh` (options: `--profile <subset>`,
`--repeat N`, `--core <id>`, `--max-msgs N`). The same script runs on the host against
`build/replay`.
