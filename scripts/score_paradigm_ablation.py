#!/usr/bin/env python3
"""Ablation of the confidence paradigm: the vector's track (object), min(object, author), min(object, author x kind).

Usage: [BUILD=build] score_paradigm_ablation.py [ground truth dir] [--variant kalman|no-kalman]
Default ground truth: the test window (common.default_gt, veremi-test); default variant: kalman.

Runs veremi_eval.evaluate() with the three modes (each subset flattened once, replayed per mode), then writes
results/veremi-eval-<ground truth name>/score-paradigms-<variant>.md: per mode, the Honest FPR, the macro F1 (mean
of the 8 subsets' F1, all vectors) and the mean over the subsets of the % flagged of the faulty vectors per fate.
"""
import statistics
import sys

import veremi_eval
from common import SUBSETS, ground_truth_dir, metrics, option

FATES = ["1 right, rejected", "2 right, accepted", "3 wrong track", "4 ghost track"]


def summary(gt, variant, results):
    """score-paradigms-<variant>.md from veremi_eval.evaluate()'s results. Each value is rounded as table.md and
    fate.md print it (F1 to 4 decimals, % flagged per fate to an integer) before the mean over the subsets."""
    lines = [f"# Confidence paradigms: {gt.name}, {variant}, VeReMi sensor errors", "",
             "Macro F1: mean of the 8 subsets' F1 (all vectors). Fates: mean over the subsets of % flagged.", "",
             "Threshold-free (roc.md, all vectors): macro AUC and macro mean recall over FPR <= 1 %.", "",
             "| Mode | Honest FPR (%) | Macro F1 | Macro AUC | Macro low-FPR recall (%) | "
             + " | ".join(f[2:] for f in FATES) + " |", "|---|---|---|---|---|---|---|---|---|"]
    for mode in veremi_eval.CONFIDENCE:
        detection, fates, roc = results[mode]
        aucs = [roc[s]["All vectors"] for s in SUBSETS[1:] if "All vectors" in roc.get(s, {})]
        f1 = [metrics(*detection[s][None])[0] for s in SUBSETS[1:] if s in detection]
        f1 = [float(f"{v:.4f}") for v in f1 if v is not None]
        fpr = metrics(*detection[SUBSETS[0]][None])[1]
        means = []
        for fate in FATES:
            values = [float(f"{100 * flagged[fate] / count[fate]:.0f}") for count, flagged in fates.values()
                      if count[fate]]
            means.append(f"{statistics.mean(values):.1f}" if values else "-")
        lines.append(f"| {mode} | {fpr:.2f} | {statistics.mean(f1):.4f} | {statistics.mean(a for a, _ in aucs):.4f} | "
                     f"{100 * statistics.mean(low for _, low in aucs):.2f} | " + " | ".join(means) + " |")
    path = veremi_eval.out_dir(gt, variant, "object").parent / f"score-paradigms-{variant}.md"
    path.write_text("\n".join(lines) + "\n")
    print("\n".join(lines), flush=True)


def main():
    args = sys.argv[1:]
    variant = option(args, "--variant", "kalman")
    if len(args) > 1:
        sys.exit(__doc__)
    gt = ground_truth_dir(args)
    summary(gt, variant, veremi_eval.evaluate(gt, variant, veremi_eval.CONFIDENCE))


if __name__ == "__main__":
    main()
