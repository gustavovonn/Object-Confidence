#!/usr/bin/env python3
"""Ablation of the track retirement time on a fault-free subset: association quality per value.

Usage: retire_ablation.py [subset dir] [results dir]
  subset dir   generator subset; default libs/Artery-Dataset-Generator/data/subsets/lust-full-run0-Honest
  results dir  default results/retire-ablation-<subset name>
Writes <results dir>/retire<value>.csv per value and <results dir>/retire_ablation.md.

Each value is replayed with `build/replay --retire-after <value>` and scored by association_eval.py
with the same RETIRE_AFTER, so its k replay retires tracks like the engine did. Values: 1.0 to 5.0 s
in 0.5 s steps (the engine default is 1.5 s). JOBS (environment, default 3) replays run at once.
"""
import os
import subprocess
import sys
from pathlib import Path

import association_eval

VALUES = [1.0 + 0.5 * i for i in range(9)]
ROOT = Path(__file__).resolve().parent.parent


def flatten(subset, receptions):
    if not (subset / "receivers").is_dir():
        generator = subset.parent.parent.parent
        subprocess.run([sys.executable, generator / "scripts/receiver_view.py", subset],
                       check=True, stdout=subprocess.DEVNULL)
    subprocess.run([sys.executable, ROOT / "scripts/flatten_dataset.py", subset, receptions], check=True)


def replay(receptions, out, jobs):
    """Replays every value, at most `jobs` at once; returns {value: records path}."""
    records = {v: out / f"retire{v:.1f}.csv" for v in VALUES}
    running = []
    for v, path in records.items():
        if len(running) == jobs:
            running.pop(0).wait()
        running.append(subprocess.Popen([ROOT / "build/replay", receptions, path, "--retire-after", str(v)],
                                        stdout=subprocess.DEVNULL))
    for p in running:
        p.wait()
    return records


def table(records):
    lines = ["| Retire after (s) | Tracks | Purity (%) | Mixed | Mean k | Objects with k >= 2 (%) "
             "| Honest CAM rejected (%) | Honest CPM rejected (%) |",
             "|---|---|---|---|---|---|---|---|"]
    for v, path in records.items():
        association_eval.RETIRE_AFTER = v
        s = association_eval.stats(association_eval.load(path))
        # association_eval.row, with the value in place of the subset name and without the vector count
        cells = association_eval.row(f"{v:.1f}", s).split(" | ")
        lines.append(" | ".join(cells[:1] + cells[2:]))
    return "\n".join(lines) + "\n"


def main():
    subset = Path(sys.argv[1] if len(sys.argv) > 1
                  else ROOT / "libs/Artery-Dataset-Generator/data/subsets/lust-full-run0-Honest").resolve()
    out = Path(sys.argv[2] if len(sys.argv) > 2 else ROOT / f"results/retire-ablation-{subset.name}").resolve()
    out.mkdir(parents=True, exist_ok=True)

    receptions = out / "receptions.csv"
    flatten(subset, receptions)
    records = replay(receptions, out, int(os.environ.get("JOBS", "3")))
    receptions.unlink()

    text = f"Subset: `{subset.name}`\n\n" + table(records)
    (out / "retire_ablation.md").write_text(text)
    print(text)
    print(f"wrote {out / 'retire_ablation.md'}")


if __name__ == "__main__":
    main()
