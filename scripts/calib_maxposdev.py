#!/usr/bin/env python3
"""Calibration of the predictive check bound (EngineParams::maxPosDev) on the honest residuals of a calibration run,
as paper 1 calibrated d_max: a high quantile of the residual of honest reports to the projected track.

Usage: [BUILD=build] calib_maxposdev.py [calibration ground truth dir]
Default: the validation window (common.validation_gt, veremi-val); the test window is refused (common.calibration_dir). Prints and writes results/calibration/<ground truth name>/maxposdev.md.

The Honest subset is made in the standard condition of scripts/veremi_eval.py (VeReMi sensor errors,
--meas-noise VEREMI_JITTER) and replayed with Kalman, object confidence and the predictive check off
(--no-predictive), so the bound being calibrated does not shape the tracks its residuals are measured on.
Residual: distance of each honest report compared with a track (fused or rejected) to that track projected to its
time. Written, no value chosen here (the quantile is fixed in include/MechanismConstants.h with its reason):
  every quantile from p99 up (p99.0 ... p99.9, p99.95, p99.99, max), with the vectors above it per origin;
  p99 and p99.9 per band of v * |dt| (track speed times the time since the track's latest update), paper 1's
  dynamic term: flat bands support a static bound.
One process at a time, at the lowest CPU and I/O priority; the records are deleted once read.
"""
import math
import subprocess
import sys
from pathlib import Path

import veremi_eval
from common import ROOT, calibration_dir, engine_constant, receptions, run, setup, stream

QUANTILES = [99.0 + 0.1 * i for i in range(10)] + [99.95, 99.99, 100.0]
BANDS = [(0.0, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 10.0), (10.0, math.inf)]  # v * |dt| [m]


def quantile(ordered, q):
    """Nearest-rank q-th percentile of an ascending list."""
    return ordered[min(len(ordered) - 1, max(0, math.ceil(q / 100 * len(ordered)) - 1))]


def residuals(path):
    """[(residual, origin, v * |dt|)] of the honest reports compared with a track."""
    out, last = [], {}  # last: track -> time of its state after the previous vector on it
    for r in stream(path):
        track = (r["receiver"], r["trackId"])
        if r["outcome"] in ("fused", "rejected") and r["attackType"] == "0" and track in last:
            t_state = last[track]
            v = float(r["vhat"]) if r["vhat"] not in ("", "nan") else math.nan
            out.append((float(r["residual"]), r["origin"], v * abs(float(r["t"]) - t_state)))
        if r["outcome"] in ("started", "fused", "rejected") and r["stateT"] not in ("", "nan"):
            last[track] = float(r["stateT"])
    return out


def main():
    args = sys.argv[1:]
    if len(args) > 1:
        sys.exit(__doc__)
    gt = calibration_dir(args)
    if not gt.is_dir():
        sys.exit(f"missing calibration ground truth {gt}: simulate it with the generator (scripts/run.sh -c veremi-val)")
    out = ROOT / "results" / "calibration" / gt.name
    out.mkdir(parents=True, exist_ok=True)
    replay = setup()

    receptions(gt, "Honest", out, sensor_errors=True)
    flat, records = out / "Honest.csv", out / "records.csv"
    run([replay, flat, records, "--no-predictive", "--state", *veremi_eval.STANDARD], stdout=subprocess.DEVNULL)
    flat.unlink()
    rows = residuals(records)
    records.unlink()

    current = engine_constant("DEFAULT_MAX_POS_DEV")
    ordered = sorted(r for r, _, _ in rows)
    lines = [f"# Predictive bound calibration: {gt.name}, Honest, VeReMi sensor errors, Kalman, predictive check off",
             "", f"{len(ordered)} honest reports compared with a track. Current DEFAULT_MAX_POS_DEV: {current} m "
             f"({100 * sum(r > current for r in ordered) / len(ordered):.3f} % of them above it).", "",
             "## Quantiles from p99", "",
             "| Quantile | Residual [m] | Above it: all | CAM | CPM |", "|---|---|---|---|---|"]
    for q in QUANTILES:
        value = quantile(ordered, q)
        above = [o for r, o, _ in rows if r > value]
        lines.append(f"| p{q:g} | {value:.3f} | {len(above)} | {above.count('cam')} | {above.count('cpm')} |")
    lines += ["", "## Dependence on v * |dt| (paper 1's dynamic term)", "",
              "| v * \\|dt\\| [m] | n | p99 [m] | p99.9 [m] |", "|---|---|---|---|"]
    for low, high in BANDS:
        band = sorted(r for r, _, d in rows if math.isfinite(d) and low <= d < high)
        if band:
            lines.append(f"| {low:g}-{high:g} | {len(band)} | {quantile(band, 99):.3f} | {quantile(band, 99.9):.3f} |")
    text = "\n".join(lines) + "\n"
    (out / "maxposdev.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
