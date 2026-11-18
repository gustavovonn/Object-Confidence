#!/usr/bin/env python3
"""Detection on Honest and the 8 VeReMi Extension subsets, with the VeReMi Extension sensor errors: one variant,
per confidence mode.

Usage: [BUILD=build] [KEEP_RECEPTIONS=1] veremi_eval.py [ground truth dir] [--variant kalman|no-kalman]
                                                         [--confidence <modes>]
  <modes>  comma-separated, of object, author, author-kind (replay --confidence); default: object
Default ground truth: the test window (common.default_gt, veremi-test); default variant: kalman.

Each subset is made and flattened (common.receptions) with the generator's sensor errors kept
(VeReMi Extension Sec. IV-B, libs/Artery-Dataset-Generator/scripts/inject_faults.py), replayed once
with the variant's flags and --meas-noise VEREMI_JITTER, once for every mode (the replay records the track's, the
author's and the author and kind's confidence; each mode combines them, common.with_paradigm), and its receptions deleted (kept with
KEEP_RECEPTIONS=1, in data/receptions/veremi-eval-<ground truth name>/, for vf2-setup/); then each
mode's run is scored.
One process at a time, at the lowest CPU and I/O priority.

Writes results/veremi-eval-<ground truth name>/<variant>-<confidence>/ (laid out as results/<run>/):
  <subset>/records.csv.gz   link to <variant>-records/<subset>/records.csv.gz, the subset's one build/replay
  table.md               detection_table(): F1, FPR, accuracy, precision, recall per subset (common.metrics), over
                         all vectors, CAMs and CPM objects; Honest first, out of the total over the fault subsets
  fate.md                fate_table(): where the faulty vectors go, and the share of each fate flagged
  association.md         association_eval.table(): purity, mixed tracks, tracks per object, honest rejections
  roc.md, roc.png        roc_table(): ROC per fault subset over every threshold (all vectors; CAMs, the scope of
                         the speed faults), its AUC, the mean recall at FPR <= 1 % and the operating point

Fate of every faulty vector (attackType > 0); a track (receiver, trackId) belongs to the station holding most
of its honest vectors (fused or started):
  1 right, rejected   on the track of its true station, a check failed
  2 right, accepted   on the track of its true station, it passed: invisible to the checks
  3 wrong track       on a track that belongs to another station
  4 ghost track       on a track with no honest vector of its true station
  dropped             failed plausibility before association
flagged = confidence < THRESHOLD (common.py), per fate. Dilution: faulty vectors rejected on the right track,
by the number of other sources (author, objectId) fused into that track within RETIRE_AFTER, and the share of them
flagged: honest observers of the same object keep a track's confidence up when the confidence is per track.
"""
import csv
import math
import multiprocessing
import os
import subprocess
import sys
from pathlib import Path
from collections import Counter, defaultdict

import association_eval
import kalman_eval
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

from common import (JOBS, RETIRE_AFTER, ROOT, SUBSETS, THRESHOLD, compress, counts, ground_truth_dir, load, metrics,
                    open_records, option, receptions, receptions_dir, records_path, run, setup, stream,
                    wait_for_memory)

FLAGS = dict(kalman_eval.VARIANTS)
CONFIDENCE = ["object", "author", "author-kind"]  # replay --confidence

# White part of the VeReMi position error, the UKF's measurement noise [m]. Per vehicle and axis, a bias
# e0 ~ U(-5, 5) m and a jitter around it, e_t - e0 = 0.5 (e_{t-1} - e0) + w, w ~ N(0, (0.03 e0)^2): stationary
# sigma 0.03 |e0| / sqrt(1 - 0.5^2), RMS over the fleet (E[e0^2] = 25/3) 0.10 m. The bias is constant per vehicle
# and stays out of R.
VEREMI_JITTER = 0.03 / math.sqrt(1 - 0.5 ** 2) * math.sqrt(25 / 3)
# replay flags of the standard condition (VeReMi sensor errors), shared by the sensitivity analyses
STANDARD = ["--meas-noise", f"{VEREMI_JITTER:.4f}"]


def out_dir(gt, variant, confidence):
    return ROOT / "results" / f"veremi-eval-{gt.name}" / f"{variant}-{confidence}"


FATES = ["1 right, rejected", "2 right, accepted", "3 wrong track", "4 ghost track", "dropped"]
BANDS = ["0", "1-2", "3+"]


def owners(path):
    """Track -> station holding most of its honest vectors, and that station's honest count on it."""
    honest = defaultdict(Counter)
    with open_records(path) as f:
        for r in csv.DictReader(f):
            if r["attackType"] == "0" and r["outcome"] in ("fused", "started"):
                honest[(r["receiver"], r["trackId"])][r["trueStation"]] += 1
    return honest


def fates(path, paradigm=None):
    honest = owners(path)
    count, flagged = Counter(), Counter()
    diluted = defaultdict(lambda: [0, 0])  # band -> [flagged, n]
    last = defaultdict(dict)               # track -> source -> latest fused time
    for r in stream(path, paradigm):
        track, source, t = (r["receiver"], r["trackId"]), (r["author"], r["objectId"]), float(r["t"])
        if r["attackType"] != "0":
            if r["outcome"] == "dropped":
                fate = "dropped"
            else:
                h = honest.get(track, Counter())
                if h[r["trueStation"]] == 0:
                    fate = "4 ghost track"
                elif h.most_common(1)[0][0] != r["trueStation"]:
                    fate = "3 wrong track"
                else:
                    fate = "1 right, rejected" if r["outcome"] == "rejected" else "2 right, accepted"
            flag = float(r["confidence"]) < THRESHOLD
            count[fate] += 1
            flagged[fate] += flag
            if fate == "1 right, rejected":
                n = sum(1 for s, lt in last[track].items() if s != source and t - lt <= RETIRE_AFTER)
                band = "0" if n == 0 else "1-2" if n <= 2 else "3+"
                diluted[band][0] += flag
                diluted[band][1] += 1
        if r["outcome"] in ("fused", "started"):
            last[track][source] = t
    return count, flagged, diluted


ORIGINS = [("All vectors", None), ("CAM", "cam"), ("CPM objects", "cpm")]


def fmt(value, digits):
    return "-" if value is None else f"{value:.{digits}f}"


def table_row(name, c):
    tp, fp, tn, fn = c
    f1, fpr, accuracy, precision, recall = metrics(*c)
    return "| {} | {} | {} | {} | {} | {} | {} | {} |".format(
        name, tp + fp + tn + fn, tp + fn, fmt(f1, 4), fmt(fpr, 2), fmt(accuracy, 2), fmt(precision, 2), fmt(recall, 2))


def detection_table(run, paradigm=None):
    """table.md of a run dir (<subset>/records.csv): one table per origin; `-` when undefined (no positives: speed
    faults never reach a CPM); counts summed over the subset. Subsets without records.csv are left out."""
    per = {}  # {subset: {origin: counts}}, one subset in memory at a time
    for subset in SUBSETS:
        if records_path(run / subset / "records.csv"):
            rows = load(run / subset / "records.csv", paradigm)
            per[subset] = {origin: counts(rows, origin) for _, origin in ORIGINS}
    lines = []
    for title, origin in ORIGINS:
        lines += [f"## {title}", "",
                  "| Fault | Vectors | Positives | F1 | FPR (%) | Accuracy (%) | Precision (%) | Recall (%) |",
                  "|---|---|---|---|---|---|---|---|"]
        if SUBSETS[0] in per:
            lines.append(table_row(SUBSETS[0], per[SUBSETS[0]][origin]))
        total = [0, 0, 0, 0]
        for subset in [s for s in SUBSETS[1:] if s in per]:
            c = per[subset][origin]
            total = [a + b for a, b in zip(total, c)]
            lines.append(table_row(subset, c))
        lines += [table_row("**all**", total), ""]
    (run / "table.md").write_text("\n".join(lines))
    return per


def fate_table(run, paradigm=None):
    """fate.md of a run dir (<subset>/records.csv), see the module docstring; streaming, each records.csv read
    twice."""
    per = {}  # {subset: (count, flagged)} per fate
    lines = [f"# Fate of the faulty vectors: {run}", "",
             f"% of each subset's faulty vectors (and % of them flagged, confidence < {THRESHOLD}).", "",
             "| Subset | n | " + " | ".join(FATES) + " | flagged |", "|---|---|" + "---|" * (len(FATES) + 1)]
    dil = ["", "## Dilution: rejected on the right track, by other sources on it (% flagged, % of them)", "",
           "| Subset | " + " | ".join(f"{b} other sources" for b in BANDS) + " |", "|---|" + "---|" * len(BANDS)]
    for subset in SUBSETS[1:]:  # the fault subsets
        path = run / subset / "records.csv"
        if not records_path(path):
            continue
        count, flagged, diluted = fates(path, paradigm)
        per[subset] = (count, flagged)
        n = sum(count.values())
        cells = [f"{100 * count[f] / n:.1f} ({100 * flagged[f] / count[f]:.0f})" if count[f] else "0" for f in FATES]
        lines.append(f"| {subset} | {n} | " + " | ".join(cells) + f" | {100 * sum(flagged.values()) / n:.1f} |")
        total = sum(v[1] for v in diluted.values())
        dil.append(f"| {subset} | " + " | ".join(
            f"{100 * diluted[b][0] / diluted[b][1]:.0f} ({100 * diluted[b][1] / total:.0f})" if diluted[b][1] else "-"
            for b in BANDS) + " |")
    (run / "fate.md").write_text("\n".join(lines + dil) + "\n")
    return per


# ROC: every confidence value as the threshold (flagged below it), so the comparison does not hang on THRESHOLD
ROC_SCOPES = [("All vectors", None), ("CAM", "cam")]  # speed faults reach CAMs only: CAM is their fair scope
LOW_FPR = 0.01        # mean recall over FPR in [0, LOW_FPR]: the operating region (partial AUC / LOW_FPR)
FPR_FLOOR = 1e-4      # left edge of the log FPR axis; FPR 0 is drawn there
# categorical slots 1-8 of the dataviz reference palette, in fixed order; position faults solid, speed dashed
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]


def roc_points(tally):
    """[(FPR, recall)] from {confidence: [honest, faulty]} counts: (0, 0), then one point per distinct confidence,
    flagging every vector at or below it (ties enter together), up to (1, 1). Counts, not one pair per vector:
    a test-window subset has ~10^6 vectors and few distinct confidences."""
    negatives = sum(n for n, _ in tally.values())
    positives = sum(p for _, p in tally.values())
    points, tp, fp = [(0.0, 0.0)], 0, 0
    for value in sorted(tally):
        fp += tally[value][0]
        tp += tally[value][1]
        points.append((fp / negatives if negatives else 0.0, tp / positives if positives else 0.0))
    return points


def area(points, max_fpr=1.0):
    """Area under the ROC up to max_fpr (trapezoids, interpolated at max_fpr), divided by max_fpr: the AUC for
    max_fpr = 1, the mean recall over FPR in [0, max_fpr] below it."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x0 >= max_fpr:
            break
        if x1 > max_fpr:
            y1 = y0 + (y1 - y0) * (max_fpr - x0) / (x1 - x0)
            x1 = max_fpr
        total += (x1 - x0) * (y0 + y1) / 2
    return total / max_fpr


def roc_table(run, title, paradigm=None):
    """roc.md and roc.png of a run dir: per fault subset and scope, the AUC, the mean recall over FPR <= LOW_FPR
    and the operating point at THRESHOLD; returns {subset: {scope: (AUC, mean recall at low FPR)}}."""
    per, curves = {}, {}
    lines = [f"# ROC per fault subset: {run}", "",
             f"Flagged below a threshold swept over every confidence value. AUC: area under the ROC (0.5 chance, "
             f"1 perfect). Low-FPR recall: mean recall over FPR in [0, {100 * LOW_FPR:g} %]. At {THRESHOLD}: the "
             "operating point of the tables.", ""]
    for scope, origin in ROC_SCOPES:
        lines += [f"## {scope}", "",
                  f"| Subset | AUC | Low-FPR recall (%) | FPR at {THRESHOLD} (%) | Recall at {THRESHOLD} (%) |",
                  "|---|---|---|---|---|"]
        for subset in SUBSETS[1:]:
            path = run / subset / "records.csv"
            if not records_path(path):
                continue
            tally = defaultdict(lambda: [0, 0])  # confidence -> [honest, faulty]
            for r in stream(path, paradigm):
                if r["outcome"] != "dropped" and origin in (None, r["origin"]):
                    tally[float(r["confidence"])][r["attackType"] != "0"] += 1
            negatives = sum(n for n, _ in tally.values())
            positives = sum(p for _, p in tally.values())
            if not positives:
                continue  # no faulty vector in this scope
            op_fpr = sum(n for c, (n, _) in tally.items() if c < THRESHOLD) / negatives
            op_recall = sum(p for c, (_, p) in tally.items() if c < THRESHOLD) / positives
            points = roc_points(tally)
            auc, low = area(points), area(points, LOW_FPR)
            per.setdefault(subset, {})[scope] = (auc, low)
            curves[(scope, subset)] = (points, (op_fpr, op_recall))
            lines.append(f"| {subset} | {auc:.4f} | {100 * low:.2f} | {100 * op_fpr:.2f} | {100 * op_recall:.2f} |")
        lines.append("")
    (run / "roc.md").write_text("\n".join(lines))
    roc_figure(curves, run / "roc.png", title)
    return per


def roc_figure(curves, path, title):
    """One panel per scope, one line per fault subset, log FPR axis; the operating point at THRESHOLD marked."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"no matplotlib: {path} not drawn (roc.md written)", file=sys.stderr)
        return
    ink, muted, grid = "#1f1f1e", "#6b6a64", "#e4e3df"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": muted, "axes.labelcolor": ink, "text.color": ink,
                         "xtick.color": muted, "ytick.color": muted, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(1, len(ROC_SCOPES), figsize=(10, 4.2), sharey=True)
    chance = [FPR_FLOOR * 10 ** (k / 20) for k in range(0, 81)]
    for ax, (scope, _) in zip(axes, ROC_SCOPES):
        ax.plot(chance, chance, color=muted, linewidth=1, linestyle=":", label="chance")
        for i, subset in enumerate(SUBSETS[1:]):
            if (scope, subset) not in curves:
                continue
            points, (op_fpr, op_recall) = curves[(scope, subset)]
            style = "-" if "Pos" in subset else "--"
            ax.plot([max(x, FPR_FLOOR) for x, _ in points], [y for _, y in points], color=COLORS[i],
                    linewidth=2, linestyle=style, label=subset, drawstyle="steps-post")
            ax.plot(max(op_fpr, FPR_FLOOR), op_recall, marker="o", markersize=6, color=COLORS[i],
                    markeredgecolor="white", markeredgewidth=1.5, linestyle="none")
        ax.axvline(LOW_FPR, color=muted, linewidth=1, linestyle="-", alpha=0.5)
        ax.set_xscale("log")
        ax.set_xlim(FPR_FLOOR, 1)
        ax.set_xticks([1e-4, 1e-3, 1e-2, 1e-1, 1], ["0.01 %", "0.1 %", "1 %", "10 %", "100 %"])
        ax.minorticks_off()
        ax.set_ylim(0, 1.01)
        ax.grid(True, color=grid, linewidth=0.8)
        ax.set_title(scope, loc="left", fontsize=10)
        ax.set_xlabel("False-positive rate (log scale)")
    axes[0].set_ylabel("Recall")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, frameon=False, fontsize=8)
    fig.suptitle(f"{title}  (dots: threshold {THRESHOLD}; line: FPR {100 * LOW_FPR:g} %)", x=0.01, ha="left",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def evaluate(gt, variant, modes):
    """Replays the 9 subsets per mode and scores them; returns {mode: (detection counts, fates, ROC areas)}, as
    detection_table(), fate_table() and roc_table() return them."""
    rec = receptions_dir("veremi-eval", gt)
    rec.mkdir(parents=True, exist_ok=True)
    replay = setup()

    def prepare(subset):
        """Made, flattened, replayed once and linked into every mode's dir; the subsets are independent, JOBS at
        a time (each worker's heavy work is in its own processes: injection, flatten, replay)."""
        wait_for_memory()
        print(f"{subset}: start", flush=True)
        receptions(gt, subset, rec, sensor_errors=True)
        flat = rec / f"{subset}.csv"
        # one replay serves every mode: the modes differ only in how the recorded confidences combine
        shared = out_dir(gt, variant, "records") / subset
        shared.mkdir(parents=True, exist_ok=True)
        (shared / "records.csv.gz").unlink(missing_ok=True)
        run([replay, flat, shared / "records.csv", *FLAGS[variant], *STANDARD], stdout=subprocess.DEVNULL)
        compress(shared / "records.csv")
        for mode in modes:
            cell = out_dir(gt, variant, mode) / subset
            cell.mkdir(parents=True, exist_ok=True)
            for name in ("records.csv", "records.csv.gz"):
                (cell / name).unlink(missing_ok=True)
            (cell / "records.csv.gz").symlink_to(Path("..") / ".." / shared.parent.name / subset / "records.csv.gz")
        if os.environ.get("KEEP_RECEPTIONS") != "1":
            flat.unlink()
        print(f"{subset}: replayed", flush=True)

    with ThreadPoolExecutor(JOBS) as pool:
        list(pool.map(prepare, SUBSETS))  # list(): re-raises a worker's failure

    # the modes are scored in parallel processes (pure Python, CPU-bound), each streaming its records
    with ProcessPoolExecutor(min(JOBS, len(modes)), mp_context=multiprocessing.get_context("fork")) as pool:
        results = pool.map(score_mode, [(gt, variant, mode) for mode in modes])
        return dict(zip(modes, results))


def score_mode(job):
    gt, variant, mode = job
    wait_for_memory()
    return score(out_dir(gt, variant, mode), f"{variant}, {mode} confidence", mode)


def score(run, title, paradigm=None):
    """table.md, fate.md, association.md, roc.md and roc.png of a run dir, with paradigm's confidence
    (common.with_paradigm); (detection counts, fates, ROC areas)."""
    (run / "association.md").write_text(association_eval.table(run))
    return detection_table(run, paradigm), fate_table(run, paradigm), roc_table(run, title, paradigm)


def main():
    args = sys.argv[1:]
    variant = option(args, "--variant", "kalman")
    modes = option(args, "--confidence", "object").split(",")
    if variant not in FLAGS or not set(modes) <= set(CONFIDENCE) or len(args) > 1:
        sys.exit(__doc__)
    evaluate(ground_truth_dir(args), variant, modes)


if __name__ == "__main__":
    main()
