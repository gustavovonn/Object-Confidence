#!/usr/bin/env python3
"""Kalman against no Kalman on the fault-free run, at several levels of white position noise.

Usage: [BUILD=build] [KEEP_RECEPTIONS=1] kalman_eval.py [ground truth dir] [--sigmas 0,0.1,0.3]
Default ground truth: the test window (common.default_gt, veremi-test).

The Honest subset is made and flattened once (common.receptions), noised per sigma by
scripts/add_noise.py, and replayed by both variants with --meas-noise sigma (the UKF's position R follows the
injected noise; the variant without the filter ignores it) and --state (track state per vector).
Runs one process at a time, at the lowest CPU and I/O priority.

Writes results/kalman-eval-<ground truth name>/:
  s<sigma>/<variant>/Honest/records.csv   build/replay; each s<sigma>/<variant>/ laid out as results/<run>/
  receptions/Honest-s<sigma>.csv   the noised receptions, kept with KEEP_RECEPTIONS=1
  summary.md   per cell: vectors, % rejected, % flagged (common.counts)
"""
import os
import subprocess
import sys

from common import ROOT, compress, counts, ground_truth_dir, option, receptions, receptions_dir, run, setup, stream

SIGMAS = [0.0, 0.1, 0.3]  # white position noise per axis [m]
VARIANTS = [("kalman", []), ("no-kalman", ["--no-kalman"])]


def tag(sigma):
    return f"s{sigma:g}"


def out_dir(gt):
    return ROOT / "results" / f"kalman-eval-{gt.name}"


def cell_dir(gt, variant, sigma):
    return out_dir(gt) / tag(sigma) / variant / "Honest"


def noised(gt, sigma):
    return receptions_dir("kalman-eval", gt) / f"Honest-{tag(sigma)}.csv"


def honest_rates(path):
    """(vectors, rejected, flagged) of a records.csv, dropped vectors left out; streaming."""
    tp, fp, tn, fn = counts(stream(path))
    rejected = sum(r["outcome"] == "rejected" for r in stream(path))
    return tp + fp + tn + fn, rejected, tp + fp


def main():
    args = sys.argv[1:]
    sigmas = [float(s) for s in option(args, "--sigmas", ",".join(map(str, SIGMAS))).split(",")]
    gt = ground_truth_dir(args)
    out = out_dir(gt)
    rec = receptions_dir("kalman-eval", gt)
    rec.mkdir(parents=True, exist_ok=True)
    replay = setup()

    receptions(gt, "Honest", rec)
    clean = rec / "Honest.csv"
    rows = ["| sigma [m] | variant | vectors | rejected % | flagged % |", "|---|---|---|---|---|"]
    for sigma in sigmas:
        noisy = noised(gt, sigma)
        run(["python3", ROOT / "scripts" / "add_noise.py", clean, noisy, sigma])
        for name, flags in VARIANTS:
            cell = cell_dir(gt, name, sigma)
            cell.mkdir(parents=True, exist_ok=True)
            records = cell / "records.csv"
            run([replay, noisy, records, *flags, "--meas-noise", sigma, "--state"], stdout=subprocess.DEVNULL)
            n, rejected, flagged = honest_rates(records)
            compress(records)
            rows.append(f"| {sigma:g} | {name} | {n} | {100 * rejected / n:.3f} | {100 * flagged / n:.3f} |")
            print(rows[-1], flush=True)
        if os.environ.get("KEEP_RECEPTIONS") != "1":
            noisy.unlink()

    commit = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
    head = [f"# Kalman x no Kalman under white position noise: {gt.name}, Honest", "",
            f"Commit {commit.stdout.strip()}; dropped vectors left out.", ""]
    (out / "summary.md").write_text("\n".join(head + rows) + "\n")
    if os.environ.get("KEEP_RECEPTIONS") != "1":
        clean.unlink()


if __name__ == "__main__":
    main()
