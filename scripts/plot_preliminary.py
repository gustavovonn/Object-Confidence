#!/usr/bin/env python3
"""Figures and tables of the preliminary experiment (scripts/run_preliminary.sh output).

Usage: plot_preliminary.py [results/preliminary]
Reads <dir>/<variant>/<fault type>/records.csv, one subset per fault type.
Genuine vectors and honest tracks are pooled over the subsets; a fault type's AUC compares its
faulty tracks with the honest tracks of its own subset.
Writes <dir>/figures/*.png and <dir>/tables.md. Needs matplotlib (requirements.txt).
"""
from collections import Counter, defaultdict
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate import OUTCOMES, auc, load, mean, track_scores, vector_outcomes  # noqa: E402

VARIANTS = {"faults": "faults only", "faults_sensor": "faults + sensor errors"}

# reference categorical palette (dataviz skill, light mode), fixed order
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
NEUTRAL = "#a3a29c"


def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": TEXT_2, "xtick.color": TEXT_2, "ytick.color": TEXT_2,
        "text.color": TEXT, "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
        "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "legend.frameon": False,
    })


def roc(honest, faulty):
    """Points (FPR, TPR) flagging a track when its score is <= threshold."""
    thresholds = sorted(set(honest) | set(faulty))
    pts = [(0.0, 0.0)]
    for th in thresholds:
        pts.append((sum(s <= th for s in honest) / len(honest), sum(s <= th for s in faulty) / len(faulty)))
    return pts


def fig_roc(data, out):
    fig, ax = plt.subplots(figsize=(4.2, 4.0))
    ax.plot([0, 1], [0, 1], color=NEUTRAL, linewidth=1, linestyle="--")
    for i, (variant, t) in enumerate(data.items()):
        x, y = zip(*roc(t["honest"], t["faulty_all"]))
        a = auc(t["honest"], t["faulty_all"])
        ax.plot(x, y, color=SERIES[i], linewidth=2, label=f"{VARIANTS[variant]} (AUC {a:.3f})")
    ax.set(xlim=(0, 1), ylim=(0, 1.01), xlabel="honest tracks flagged (FPR)", ylabel="faulty tracks flagged (TPR)",
           title="Track confidence: honest vs faulty")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=200)
    plt.close(fig)


def fig_scores(t, variant, out):
    kinds = sorted(t["faulty"], key=lambda k: mean(t["faulty"][k]))
    labels = ["honest"] + kinds
    values = [t["honest"]] + [t["faulty"][k] for k in kinds]
    fig, ax = plt.subplots(figsize=(6.0, 0.42 * len(labels) + 1.0))
    box = ax.boxplot(values, orientation="horizontal", widths=0.55, patch_artist=True, showfliers=False,
                     medianprops={"color": SURFACE, "linewidth": 2},
                     whiskerprops={"color": TEXT_2, "linewidth": 1}, capprops={"color": TEXT_2, "linewidth": 1})
    for i, patch in enumerate(box["boxes"]):
        patch.set(facecolor=NEUTRAL if i == 0 else SERIES[0], edgecolor="none")
    ax.set_yticks(range(1, len(labels) + 1), labels)
    ax.invert_yaxis()
    for i, k in enumerate(kinds, start=2):
        ax.text(1.02, i, f"AUC {auc(t['honest_by'][k], t['faulty'][k]):.2f}", va="center", fontsize=8, color=TEXT_2)
    ax.set(xlim=(0, 1), xlabel="track score (mean confidence)",
           title=f"Track score by dominating source ({VARIANTS[variant]})")
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(out, dpi=200)
    plt.close(fig)


def fig_outcomes(vec, variant, out):
    labels = list(vec)
    fig, ax = plt.subplots(figsize=(6.0, 0.38 * len(labels) + 1.2))
    left = [0.0] * len(labels)
    for j, outcome in enumerate(OUTCOMES):
        share = [100.0 * vec[k][outcome] / sum(vec[k].values()) for k in labels]
        ax.barh(range(len(labels)), share, left=left, color=SERIES[j], height=0.6,
                edgecolor=SURFACE, linewidth=1, label=outcome)
        left = [a + b for a, b in zip(left, share)]
    ax.set_yticks(range(len(labels)), labels)
    ax.invert_yaxis()
    ax.set(xlim=(0, 100), xlabel="% of received motion vectors", title=f"Vector outcome by source ({VARIANTS[variant]})")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18 if len(labels) < 6 else -0.12), ncol=3)
    fig.tight_layout()
    fig.savefig(out, dpi=200)
    plt.close(fig)


def tables(vectors, tracks):
    lines = ["## Vector level (% of received motion vectors; rejected = flagged)", ""]
    names = list(next(iter(vectors.values())))
    lines.append("| Source | " + " | ".join(f"{VARIANTS[v]}: fused / rejected / started" for v in vectors) + " |")
    lines.append("|---|" + "---|" * len(vectors))
    for k in names:
        cells = []
        for vec in vectors.values():
            c = vec.get(k)
            n = sum(c.values()) if c else 0
            cells.append(" / ".join(f"{100 * c[o] / n:.1f}" for o in OUTCOMES) if n else "-")
        lines.append(f"| {k} | " + " | ".join(cells) + " |")

    lines += ["", "## Track level (score = mean confidence; AUC = P(honest > faulty))", ""]
    lines.append("| Dominating source | " + " | ".join(f"{VARIANTS[v]}: tracks / score / AUC" for v in tracks) + " |")
    lines.append("|---|" + "---|" * len(tracks))
    first = next(iter(tracks.values()))
    rows = [("honest", "honest"), ("honest, clean", "clean"), ("honest, touched by faulty", "touched")]
    for label, key in rows:
        lines.append(f"| {label} | " + " | ".join(f"{len(t[key])} / {mean(t[key]):.3f} / -" for t in tracks.values()) + " |")
    for kind in sorted(first["faulty"]):
        cells = [f"{len(t['faulty'][kind])} / {mean(t['faulty'][kind]):.3f} / {auc(t['honest_by'][kind], t['faulty'][kind]):.3f}"
                 for t in tracks.values()]
        lines.append(f"| {kind} | " + " | ".join(cells) + " |")
    lines.append("| **all faulty** | " + " | ".join(
        f"{len(t['faulty_all'])} / {mean(t['faulty_all']):.3f} / {auc(t['honest'], t['faulty_all']):.3f}"
        for t in tracks.values()) + " |")
    return "\n".join(lines) + "\n"


def load_variant(path):
    """Merges the per-type subsets of one variant."""
    vec = defaultdict(Counter)
    t = {"clean": [], "touched": [], "faulty": defaultdict(list), "honest_by": {}}
    for subset in sorted(p for p in path.iterdir() if (p / "records.csv").exists()):
        rows = load(subset / "records.csv")
        for key, c in vector_outcomes(rows).items():
            vec[key].update(c)
        s = track_scores(rows)
        t["clean"] += s["clean"]
        t["touched"] += s["touched"]
        for kind, scores in s["faulty"].items():
            t["faulty"][kind] += scores
            t["honest_by"][kind] = s["honest"]
    t["honest"] = t["clean"] + t["touched"]
    t["faulty_all"] = [x for v in t["faulty"].values() for x in v]
    order = sorted(vec, key=lambda k: (not k.startswith("Genuine"), k))
    return {k: vec[k] for k in order}, t


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "results/preliminary")
    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    style()

    vectors, tracks = {}, {}
    for variant in VARIANTS:
        vectors[variant], tracks[variant] = load_variant(root / variant)
        fig_scores(tracks[variant], variant, figures / f"track_scores_{variant}.png")
        fig_outcomes(vectors[variant], variant, figures / f"vector_outcomes_{variant}.png")
    fig_roc(tracks, figures / "track_roc.png")
    (root / "tables.md").write_text(tables(vectors, tracks))
    print(f"wrote {figures} and {root / 'tables.md'}")


if __name__ == "__main__":
    main()
