#!/usr/bin/env python3
"""Minimal preliminary result: per-fault detection table and one confidence-over-time figure.

Usage: detect_eval.py <results dir>
Reads <dir>/<fault type>/records.csv (scripts/run_preliminary.sh) and writes
<dir>/table.md and <dir>/confidence_over_time.png (needs matplotlib, requirements.txt).

Metrics as paper 1 (Beacon-Confidence, detection-eval/scripts/run_scorer_comparison.py):
one decision per received vector, flagged when the confidence of its track after the vector
is below 0.5; positive when the vector's content was falsified (attackType > 0; a CPM object
takes its message's). Precision, recall, FPR and accuracy in %, F1 in [0, 1], `-` when undefined
(no positives: speed faults never reach a CPM); counts summed over the subset.
One table over all vectors, one over CAMs, one over CPM objects.
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


ORIGINS = [("All vectors", None), ("CAM", "cam"), ("CPM objects", "cpm")]


def counts(rows, origin=None):
    tp = fp = tn = fn = 0
    for r in rows:
        if r["outcome"] == "dropped" or origin not in (None, r["origin"]):
            continue
        flagged = float(r["confidence"]) < THRESHOLD
        faulty = r["attackType"] != "0"
        tp += flagged and faulty
        fp += flagged and not faulty
        tn += not flagged and not faulty
        fn += not flagged and faulty
    return tp, fp, tn, fn


def metrics(tp, fp, tn, fn):
    """(F1, FPR, accuracy, precision, recall); None when undefined."""
    precision = 100.0 * tp / (tp + fp) if tp + fp else None
    recall = 100.0 * tp / (tp + fn) if tp + fn else None
    fpr = 100.0 * fp / (fp + tn) if fp + tn else None
    accuracy = 100.0 * (tp + tn) / (tp + fp + tn + fn) if tp + fp + tn + fn else None
    f1 = 2 * precision * recall / (precision + recall) / 100.0 if precision and recall else None
    if f1 is None and recall is not None:
        f1 = 0.0
    return f1, fpr, accuracy, precision, recall


def fmt(value, digits):
    return "-" if value is None else f"{value:.{digits}f}"


def row(name, c):
    tp, fp, tn, fn = c
    f1, fpr, accuracy, precision, recall = metrics(*c)
    return "| {} | {} | {} | {} | {} | {} | {} | {} |".format(
        name, tp + fp + tn + fn, tp + fn, fmt(f1, 4), fmt(fpr, 2), fmt(accuracy, 2),
        fmt(precision, 2), fmt(recall, 2))


def table(root):
    # one subset in memory at a time: {kind: {origin: counts}}
    per = {}
    for kind in TYPES:
        rows = load(root / kind / "records.csv")
        per[kind] = {origin: counts(rows, origin) for _, origin in ORIGINS}
    lines = []
    for title, origin in ORIGINS:
        lines += [f"## {title}", "",
                  "| Fault | Vectors | Positives | F1 | FPR (%) | Accuracy (%) | Precision (%) | Recall (%) |",
                  "|---|---|---|---|---|---|---|---|"]
        total = [0, 0, 0, 0]
        for kind in TYPES:
            c = per[kind][origin]
            total = [a + b for a, b in zip(total, c)]
            lines.append(row(kind, c))
        lines += [row("**all**", total), ""]
    return "\n".join(lines)


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
    root = Path(sys.argv[1])
    text = table(root)
    (root / "table.md").write_text(text)
    print(text)
    figure(root, root / "confidence_over_time.png")
    print(f"wrote {root / 'table.md'} and {root / 'confidence_over_time.png'}")


if __name__ == "__main__":
    main()
