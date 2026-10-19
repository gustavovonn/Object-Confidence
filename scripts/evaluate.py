#!/usr/bin/env python3
"""Minimal preliminary result: per-fault detection table and one confidence-over-time figure.

Usage: evaluate.py [results/preliminary]
Reads <dir>/<fault type>/records.csv (scripts/run_preliminary.sh) and writes
<dir>/table.md and <dir>/confidence_over_time.png (needs matplotlib, requirements.txt).

Metrics as paper 1 (Beacon-Confidence, detection-eval/scripts/run_scorer_comparison.py):
one decision per received vector, flagged when the confidence of its track after the vector
is below 0.5; positive when the vector's content was falsified (attackType > 0).
Precision, recall, FPR and accuracy in %, F1 in [0, 1]; counts summed over the subset.
"""
import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path

TYPES = ["ConstPos", "ConstPosOffset", "RandomPos", "RandomPosOffset",
         "ConstSpeed", "ConstSpeedOffset", "RandomSpeed", "RandomSpeedOffset"]


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def tracks_of(rows):
    """{(receiver, trackId): [rows in order]}"""
    tracks = defaultdict(list)
    for r in rows:
        if r["trackId"] != "0":
            tracks[(r["receiver"], r["trackId"])].append(r)
    return tracks


def is_faulty(vectors):
    return sum(1 for v in vectors if v["attackType"] != "0") > len(vectors) / 2


THRESHOLD = 0.5


def counts(rows):
    tp = fp = tn = fn = 0
    for r in rows:
        if r["outcome"] == "dropped":
            continue
        flagged = float(r["confidence"]) < THRESHOLD
        faulty = r["attackType"] != "0"
        tp += flagged and faulty
        fp += flagged and not faulty
        tn += not flagged and not faulty
        fn += not flagged and faulty
    return tp, fp, tn, fn


def metrics(tp, fp, tn, fn):
    precision = 100.0 * tp / (tp + fp) if tp + fp else 0.0
    recall = 100.0 * tp / (tp + fn) if tp + fn else 0.0
    fpr = 100.0 * fp / (fp + tn) if fp + tn else 0.0
    accuracy = 100.0 * (tp + tn) / (tp + fp + tn + fn)
    f1 = 2 * precision * recall / (precision + recall) / 100.0 if precision + recall else 0.0
    return f1, fpr, accuracy, precision, recall


def table(root):
    lines = ["| Fault | F1 | FPR (%) | Accuracy (%) | Precision (%) | Recall (%) |",
             "|---|---|---|---|---|---|"]
    total = [0, 0, 0, 0]
    for kind in TYPES:
        c = counts(load(root / kind / "records.csv"))
        total = [a + b for a, b in zip(total, c)]
        lines.append("| {} | {:.4f} | {:.2f} | {:.2f} | {:.2f} | {:.2f} |".format(kind, *metrics(*c)))
    lines.append("| **all** | {:.4f} | {:.2f} | {:.2f} | {:.2f} | {:.2f} |".format(*metrics(*total)))
    return "\n".join(lines) + "\n"


def longest(tracks, faulty):
    """Longest honest or faulty track per receiver: {receiver: rows}."""
    best = {}
    for (receiver, _), vectors in tracks.items():
        if is_faulty(vectors) == faulty and len(vectors) > len(best.get(receiver, [])):
            best[receiver] = vectors
    return best


def figure(root, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    random_pos = tracks_of(load(root / "RandomPos" / "records.csv"))
    const_offset = tracks_of(load(root / "ConstPosOffset" / "records.csv"))
    honest = longest(random_pos, faulty=False)
    attacked = longest(random_pos, faulty=True)
    ghost = longest(const_offset, faulty=True)

    # one receiver that holds all three, chosen by the shortest of its three tracks
    receiver = max(set(honest) & set(attacked) & set(ghost),
                   key=lambda r: min(len(honest[r]), len(attacked[r]), len(ghost[r])))

    series = [
        ("honest track", honest[receiver], "#2a78d6"),
        ("RandomPos source", attacked[receiver], "#eb6834"),
        ("ConstPosOffset source (other subset)", ghost[receiver], "#1baf7a"),
    ]
    plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
                         "axes.grid": True, "grid.color": "#e4e3df", "legend.frameon": False})
    fig, ax = plt.subplots(figsize=(6.0, 3.4))
    for label, vectors, color in series:
        t0 = float(vectors[0]["t"])
        ax.plot([float(v["t"]) - t0 for v in vectors], [float(v["confidence"]) for v in vectors],
                color=color, linewidth=2, label=label)
    ax.axhline(0.5, color="#a3a29c", linewidth=1, linestyle="--")
    ax.set(ylim=(0, 1), xlabel="time since the track started [s]", ylabel="track confidence",
           title=f"Track confidence at receiver {receiver}")
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(out, dpi=200)


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "results/preliminary")
    text = table(root)
    (root / "table.md").write_text(text)
    print(text)
    figure(root, root / "confidence_over_time.png")
    print(f"wrote {root / 'table.md'} and {root / 'confidence_over_time.png'}")


if __name__ == "__main__":
    main()
