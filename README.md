# Attributing Confidence to C-ITS Entities based on Trajectory Consistency

Receiver-local confidence for **fused object trajectories**. Motion vectors about the same object may
come from its own CAM or from the perceived-object container of other stations' CPMs; the engine
associates them to one track, keeps a fused reference state per object with an Unscented Kalman
Filter, and scores each track by how consistently the reports agree with it.

It extends the per-sender mechanism of paper 1 (CP-Artery) to collective perception. Input traces
come from `libs/Artery-Dataset-Generator` (submodule).

## Pipeline (MVP 0)

```
onMotionVectors(vectors, receiver, now)        one received message
  0 retireStaleTracks     tracks without a fused report for 3.1 s, before association
  1 plausibilityCheck     finite values
  2 associate             per message: bound (author, objectId) keeps its track; unbound sources
                          jointly, one per track, inside the gate (Hungarian)
  3 checks                per vector: track projected to its time (UKF predict, not committed);
                          consistency against the source's last report (paper 1); residual <= gate
  4 kalmanUpdate          only if 3 passed
  5 score                 passed: hit, failed: miss
```

| Piece | Choice | Reference |
|---|---|---|
| Stage order | align, check, associate, fuse | Allig et al., VNC 2019, Sec. II-A |
| Association | bound source keeps its track (unique object ID); new sources of a message assigned jointly, one per track, at most one source per author and one CAM per track ; Hungarian algorithm via `libs/hungarian-algorithm-cpp` |
| Gate | Euclidean position residual <= 4.35 m | paper 1, static `d_max_0` (Sec. V-A) |
| Consistency | reported vs implied speed, frozen position | paper 1, Sec. IV-B-1 |
| Filter | UKF, augmented process noise, additive measurement noise | Wan & van der Merwe 2000; Julier & Uhlmann 2004; Schubert et al. 2008 |
| Motion model | CTRA `[x, y, heading, v, a, w]` | Schubert et al. 2008, Eq. 6-9 |
| Measurement noise | position and heading std of the VeReMi Extension sensor errors | Kamel et al., ICC 2020, Sec. IV-B |
| Confidence | Beta reputation with forgetting, one per track; hit or miss per report | Jøsang & Ismail 2002; Hallyburton & Pajic, CDC 2024 |

Every default lives in `include/MechanismConstants.h` with its source.

## Layout

```
include/   MotionVector, EngineParams, MechanismConstants, ObjectTrack, Tracker, Scorer,
           VectorRecord, ObjectConfidenceEngine, StageTimer
src/       Tracker, ObjectConfidenceEngine
tools/     replay (dataset -> engine -> records)
scripts/   flatten_dataset, run_preliminary, evaluate
libs/      v2x-misc (UKF, CTRA, consistency check, geodesy), eigen 3.4.1, hungarian-algorithm-cpp,
           Artery-Dataset-Generator (ground truth and fault injection)
```

`ObjectTrack` holds only the fused state (`x`, `P`, `t`). Source bindings live in `Tracker`,
evidence in `Scorer`, evaluation data in `VectorRecord`.

## Build

```sh
git submodule update --init                     # engine: eigen, v2x-misc; generator scripts
cmake -S . -B build && cmake --build build      # engine library and tools/replay
```

Generating a dataset needs the generator's own submodules (Artery, LuST); see its README:

```sh
git -C libs/Artery-Dataset-Generator submodule update --init --recursive
```

## Preliminary experiment

```sh
scripts/run_preliminary.sh                  # 8 fault subsets, replay -> results/preliminary/<type>/records.csv
.venv/bin/python scripts/evaluate.py        # table.md and confidence_over_time.png
```

One subset per VeReMi Extension position/speed malfunction (8 types, as VeReMi Extension and
paper 1), without sensor errors. A track is faulty when most of its vectors are; its score is
the mean confidence over its vectors; the AUC compares honest and faulty tracks of the same
subset. The figure follows three tracks of one receiver: an honest one, one fed by a RandomPos
source and one fed by a ConstPosOffset source.

The venv: `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`.

## Known limitations of MVP 0

- The gate ignores the track uncertainty and checks position only; next step is
  `d^2 = r^T S^-1 r` against a chi-square quantile (Ambrosin et al. 2019).
- The 4.35 m gate was calibrated on consecutive reports of one sender, where a constant GNSS
  bias cancels. Across sources it does not, so with injected sensor errors honest reports of the
  same object may land outside the gate.
- CPM objects are already tracks of their sender; fusing them as independent measurements makes
  the filter optimistic (track-to-track fusion, Allig et al. 2019).
- A source that only ever feeds its own track scores itself.
