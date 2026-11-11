#!/usr/bin/env python3
"""Ablation of the frozen-position bound on a fault-free subset: detector accuracy per value.

Usage: frozen_ablation.py [subset dir] [results dir]
  subset dir   generator subset; default libs/Artery-Dataset-Generator/data/subsets/lust-full-run0-Honest
  results dir  default results/frozen-ablation-<subset name>
Writes <results dir>/frozen<value>.csv per value and <results dir>/frozen_ablation.md.

The consistency check calls a moving source frozen when its displacement between two reports is below a
bound. "fixed" is the engine default, FROZEN_DISPLACEMENT_MIN; every other value K is replayed with
`build/replay --frozen-scale K`, frozen below K * v_avg * dt. Every vector of the subset is honest, so
each flagged vector is a false positive: the table gives accuracy and false-positive rate (detect_eval.py)
for all vectors, CAMs and CPM objects, and the honest vectors the consistency check rejected.
JOBS (environment, default 3) replays run at once.
"""
import os
import subprocess
import sys
from pathlib import Path

import detect_eval

VALUES = ["fixed", 0.1, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.75, 1.0]
ROOT = Path(__file__).resolve().parent.parent


def flatten(subset, receptions):
    if not (subset / "receivers").is_dir():
        generator = subset.parent.parent.parent
        subprocess.run([sys.executable, generator / "scripts/receiver_view.py", subset],
                       check=True, stdout=subprocess.DEVNULL)
    subprocess.run([sys.executable, ROOT / "scripts/flatten_dataset.py", subset, receptions], check=True)


def name(value):
    return value if value == "fixed" else f"{value:.2f}"


def replay(receptions, out, jobs):
    """Replays every value, at most `jobs` at once; returns {value: records path}."""
    records = {v: out / f"frozen{name(v)}.csv" for v in VALUES}
    running = []
    for v, path in records.items():
        if len(running) == jobs:
            running.pop(0).wait()
        extra = [] if v == "fixed" else ["--frozen-scale", str(v)]
        running.append(subprocess.Popen([ROOT / "build/replay", receptions, path, *extra], stdout=subprocess.DEVNULL))
    for p in running:
        p.wait()
    return records


def table(records):
    lines = ["| Frozen bound | Accuracy (%) | FPR (%) | CAM FPR (%) | CPM FPR (%) | Flagged vectors | Consistency rejections |",
             "|---|---|---|---|---|---|---|"]
    for v, path in records.items():
        rows = detect_eval.load(path)
        cells = []
        for origin in (None, "cam", "cpm"):
            _, fpr, accuracy, _, _ = detect_eval.metrics(*detect_eval.counts(rows, origin))
            cells.append((accuracy, fpr))
        flagged = sum(1 for r in rows if r["outcome"] != "dropped" and float(r["confidence"]) < detect_eval.THRESHOLD)
        rejected = sum(1 for r in rows if r["outcome"] == "rejected" and r["consistencyFailed"] == "1")
        label = "fixed (FROZEN_DISPLACEMENT_MIN)" if v == "fixed" else f"{v:.2f} · v · dt"
        lines.append(f"| {label} | {cells[0][0]:.3f} | {cells[0][1]:.3f} | {cells[1][1]:.3f} | {cells[2][1]:.3f} "
                     f"| {flagged} | {rejected} |")
    return "\n".join(lines) + "\n"


def main():
    subset = Path(sys.argv[1] if len(sys.argv) > 1
                  else ROOT / "libs/Artery-Dataset-Generator/data/subsets/lust-full-run0-Honest").resolve()
    out = Path(sys.argv[2] if len(sys.argv) > 2 else ROOT / f"results/frozen-ablation-{subset.name}").resolve()
    out.mkdir(parents=True, exist_ok=True)

    receptions = out / "receptions.csv"
    flatten(subset, receptions)
    records = replay(receptions, out, int(os.environ.get("JOBS", "3")))
    receptions.unlink()

    text = f"Subset: `{subset.name}`\n\n" + table(records)
    (out / "frozen_ablation.md").write_text(text)
    print(text)
    print(f"wrote {out / 'frozen_ablation.md'}")


if __name__ == "__main__":
    main()
