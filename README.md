# Attributing Confidence to C-ITS Entities based on Trajectory Consistency

Receiver-local confidence for **fused object trajectories**. Motion vectors about the same object may
come from its own CAM or from the perceived-object container of other stations' CPMs; the engine
associates them to one track, keeps a fused reference state per object with an Unscented Kalman
Filter, and scores each track by how consistently the reports agree with it.

It extends the per-sender mechanism of paper 1 (CP-Artery) to collective perception. Input traces
come from `Artery-Dataset-Generator`.

## Pipeline (MVP 0)

```
onMotionVectors(vectors, receiver, now)        per vector:
  1 plausibilityCheck     finite values
  2 project + associate   tracks moved to the vector time (UKF predict, not committed);
                          bound (author, objectId) keeps its track, else nearest track in the gate
  3 checks                consistency against the source's last report (paper 1);
                          position residual <= gate
  4 kalmanUpdate          only if 3 passed
  5 score                 passed: hit, failed: miss
  6 retireStaleTracks     once per call
```

| Piece | Choice | Reference |
|---|---|---|
| Stage order | align, check, associate, fuse | Allig et al., VNC 2019, Sec. II-A |
| Association | bound source keeps its track (unique object ID); new source: nearest in the gate | Allig et al. 2019; Ambrosin et al., ITSC 2019 |
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
tests/     test_engine (synthetic pipeline checks)
libs/      v2x-misc (UKF, CTRA, consistency check), eigen 3.4.1
```

`ObjectTrack` holds only the fused state (`x`, `P`, `t`). Source bindings live in `Tracker`,
evidence in `Scorer`, evaluation data in `VectorRecord`.

## Build

```sh
git submodule update --init
cmake -S . -B build && cmake --build build
./build/test_engine
```

## Preliminary experiment

```sh
scripts/run_preliminary.sh [groundtruth dir]     # inject faults, replay, evaluate -> results/preliminary/
python3 scripts/plot_preliminary.py               # figures/*.png and tables.md (needs requirements.txt)
```

One subset per VeReMi Extension position/speed malfunction (8 types, as VeReMi Extension and
paper 1), each without and with VeReMi Extension sensor errors: `results/preliminary/<variant>/<type>/`.
`scripts/evaluate.py` defines the metrics: per received vector, the outcome by source
(rejected = flagged); per track, the mean confidence over its vectors and the AUC of honest
against faulty tracks of the same subset (a track is faulty when most of its vectors are).

## Known limitations of MVP 0

- The gate ignores the track uncertainty and checks position only; next step is
  `d^2 = r^T S^-1 r` against a chi-square quantile (Ambrosin et al. 2019).
- The 4.35 m gate was calibrated on consecutive reports of one sender, where a constant GNSS
  bias cancels. Across sources it does not, so with injected sensor errors honest reports of the
  same object may land outside the gate.
- CPM objects are already tracks of their sender; fusing them as independent measurements makes
  the filter optimistic (track-to-track fusion, Allig et al. 2019).
- A source that only ever feeds its own track scores itself.
