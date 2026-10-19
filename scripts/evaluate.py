#!/usr/bin/env python3
"""Minimal preliminary result: per-fault AUC table and one confidence-over-time figure.

Usage: evaluate.py [results/preliminary]
Reads <dir>/<fault type>/records.csv (scripts/run_preliminary.sh) and writes
<dir>/table.md and <dir>/confidence_over_time.png (needs matplotlib, requirements.txt).

A track (one receiver, one trackId) is faulty when most of its vectors carry an attackType.
Its score is the mean confidence over its vectors. AUC = P(score honest > score faulty),
honest and faulty tracks taken from the same subset.
"""
import bisect
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


def score(vectors):
    return sum(float(v["confidence"]) for v in vectors) / len(vectors)


def auc(positive, negative):
    """Mann-Whitney: probability that a positive scores above a negative, ties count half."""
    ranked = sorted(negative)
    wins = 0.0
    for p in positive:
        below = bisect.bisect_left(ranked, p)
        wins += below + 0.5 * (bisect.bisect_right(ranked, p) - below)
    return wins / (len(positive) * len(negative))


def table(root):
    lines = ["| Fault | honest tracks | faulty tracks | AUC |", "|---|---|---|---|"]
    all_honest, all_faulty = [], []
    for kind in TYPES:
        tracks = tracks_of(load(root / kind / "records.csv")).values()
        honest = [score(t) for t in tracks if not is_faulty(t)]
        faulty = [score(t) for t in tracks if is_faulty(t)]
        all_honest += honest
        all_faulty += faulty
        lines.append(f"| {kind} | {len(honest)} | {len(faulty)} | {auc(honest, faulty):.3f} |")
    lines.append(f"| **all, pooled** | {len(all_honest)} | {len(all_faulty)} | {auc(all_honest, all_faulty):.3f} |")
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
