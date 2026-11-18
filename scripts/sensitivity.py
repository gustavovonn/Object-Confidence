#!/usr/bin/env python3
"""Sensitivity of the engine's fixed parameters on the fault-free run: the frozen-position bound, the track
retirement time and the predictive check bound, each swept around its default with everything else at the defaults.

Usage: [BUILD=build] sensitivity.py [ground truth dir]
Default ground truth: the validation window (common.validation_gt, veremi-val); the test window is refused: the
parameters are chosen here, then fixed for the test window.
Prints each row as it is scored and writes results/sensitivity-<ground truth name>/sensitivity.md.

The Honest subset is made once, in the standard condition of scripts/veremi_eval.py (VeReMi sensor errors,
--meas-noise VEREMI_JITTER), and replayed per value, Kalman, object confidence:
  frozen   replay --frozen-scale K: a moving object reported closer to the track's position at its latest
           update than K * v_avg * dt is frozen (default DEFAULT_FROZEN_SCALE). Every vector is honest, so each
           flagged vector is a false positive: FPR (common.metrics) for all vectors, CAMs and CPM objects, and
           the honest vectors the consistency check rejected.
  retire   replay --retire-after S (default DEFAULT_RETIRE_AFTER): association quality (association_eval.py,
           with the same RETIRE_AFTER, so its k replay retires tracks like the engine did).
  maxPosDev  replay --max-pos-dev D (default DEFAULT_MAX_POS_DEV, calibrated by scripts/calib_maxposdev.py): FPR
           as for frozen, and the honest vectors the predictive check rejected. Honest only: the false-positive
           side of the bound; what a tighter bound gains in detection is in the fault subsets.
One process at a time, at the lowest CPU and I/O priority; each value's records are deleted once scored.
"""
import subprocess
import sys

import association_eval
import veremi_eval
from common import ROOT, THRESHOLD, calibration_dir, counts, load, metrics, receptions, receptions_dir, run, setup

FROZEN = [0.1, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.75, 1.0]
RETIRE = [1.0 + 0.5 * i for i in range(9)]
MAX_POS_DEV = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.35, 5.0, 6.0]


def honest_row(label, path, check):
    """FPR for all vectors, CAMs and CPM objects, flagged vectors, and the honest vectors rejected by `check`."""
    rows = load(path)
    fprs = [metrics(*counts(rows, origin))[1] for origin in (None, "cam", "cpm")]
    flagged = sum(1 for r in rows if r["outcome"] != "dropped" and float(r["confidence"]) < THRESHOLD)
    rejected = sum(1 for r in rows if r["outcome"] == "rejected" and r[check] == "1")
    return f"| {label} | {fprs[0]:.3f} | {fprs[1]:.3f} | {fprs[2]:.3f} | {flagged} | {rejected} |"


def frozen_row(value, path):
    return honest_row(f"{value:.2f} · v · dt", path, "consistencyFailed")


def max_pos_dev_row(value, path):
    return honest_row(f"{value:.2f}", path, "predictiveFailed")


def retire_row(value, path):
    association_eval.RETIRE_AFTER = value  # its k replay retires tracks like the engine did
    return association_eval.row(f"{value:.1f}", association_eval.stats(association_eval.load(path)), vectors=False)


# name, replay flag, values, row scorer, table header
SWEEPS = [
    ("Frozen-position bound (default DEFAULT_FROZEN_SCALE = 0.25)", "--frozen-scale", FROZEN, frozen_row,
     ["| Frozen bound | FPR (%) | CAM FPR (%) | CPM FPR (%) | Flagged vectors | Consistency rejections |",
      "|---|---|---|---|---|---|"]),
    ("Track retirement (default DEFAULT_RETIRE_AFTER = 1.5 s)", "--retire-after", RETIRE, retire_row,
     ["| Retire after (s) | Tracks | Purity (%) | Mixed | Mean k | Objects with k >= 2 (%) "
      "| Honest CAM rejected (%) | Honest CPM rejected (%) |",
      "|---|---|---|---|---|---|---|---|"]),
    ("Predictive check bound (default DEFAULT_MAX_POS_DEV)", "--max-pos-dev", MAX_POS_DEV, max_pos_dev_row,
     ["| maxPosDev (m) | FPR (%) | CAM FPR (%) | CPM FPR (%) | Flagged vectors | Predictive rejections |",
      "|---|---|---|---|---|---|"]),
]


def main():
    if len(sys.argv) > 2:
        sys.exit(__doc__)
    gt = calibration_dir(sys.argv[1:])
    out = ROOT / "results" / f"sensitivity-{gt.name}"
    out.mkdir(parents=True, exist_ok=True)
    replay = setup()

    rec = receptions_dir("sensitivity", gt)
    rec.mkdir(parents=True, exist_ok=True)
    receptions(gt, "Honest", rec, sensor_errors=True)
    flat = rec / "Honest.csv"
    lines = [f"# Parameter sensitivity: {gt.name}, Honest, VeReMi sensor errors, Kalman, object confidence", "",
             f"FPR: honest vectors flagged (confidence < {THRESHOLD}).", ""]
    for title, flag, values, row, header in SWEEPS:
        lines += [f"## {title}", ""] + header
        print("\n".join(lines[-len(header) - 2:]), flush=True)
        for value in values:
            records = out / "records.csv"
            run([replay, flat, records, flag, value, *veremi_eval.STANDARD], stdout=subprocess.DEVNULL)
            lines.append(row(value, records))
            print(lines[-1], flush=True)
            records.unlink()
        lines.append("")
    flat.unlink()
    (out / "sensitivity.md").write_text("\n".join(lines))
    print(f"wrote {out / 'sensitivity.md'}")


if __name__ == "__main__":
    main()
