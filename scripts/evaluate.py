#!/usr/bin/env python3
"""Preliminary evaluation of a replay on an injected subset (labels: attackType, VeReMi Extension ids).

Usage: evaluate.py <records.csv>
  vector level: outcome per attack type; rejected = flagged
  track level:  a track is faulty when most of its vectors are faulty; its score is the mean
                confidence over its vectors; AUC = P(score honest > score faulty)
"""
import csv
import sys
from collections import Counter, defaultdict

ATTACK_NAMES = {
    0: "Genuine", 1: "ConstPos", 2: "ConstPosOffset", 3: "RandomPos", 4: "RandomPosOffset",
    5: "ConstSpeed", 6: "ConstSpeedOffset", 7: "RandomSpeed", 8: "RandomSpeedOffset",
}
OUTCOMES = ("fused", "rejected", "started")


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def auc(positive, negative):
    """Mann-Whitney: probability that a positive scores above a negative, ties count half."""
    if not positive or not negative:
        return float("nan")
    ranked = sorted([(s, 1) for s in positive] + [(s, 0) for s in negative])
    rank_sum, i = 0.0, 0
    while i < len(ranked):
        j = i
        while j < len(ranked) and ranked[j][0] == ranked[i][0]:
            j += 1
        mean_rank = (i + j + 1) / 2.0
        rank_sum += mean_rank * sum(1 for k in range(i, j) if ranked[k][1] == 1)
        i = j
    n_pos, n_neg = len(positive), len(negative)
    return (rank_sum - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg)


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def vector_outcomes(rows):
    """{source: Counter(outcome)}; source = attack name, or 'Genuine CAM' / 'Genuine CPM'."""
    groups = defaultdict(Counter)
    for r in rows:
        label = int(r["attackType"])
        key = ATTACK_NAMES.get(label, str(label)) if label else f"Genuine {r['origin'].upper()}"
        groups[key][r["outcome"]] += 1
    order = sorted(groups, key=lambda k: (not k.startswith("Genuine"), k))
    return {k: groups[k] for k in order}


def track_scores(rows):
    """Honest scores (split clean/touched) and faulty scores per attack type."""
    tracks = defaultdict(list)
    for r in rows:
        if r["trackId"] != "0":
            tracks[(r["receiver"], r["trackId"])].append(r)
    out = {"clean": [], "touched": [], "faulty": defaultdict(list)}
    for vectors in tracks.values():
        labels = [int(v["attackType"]) for v in vectors]
        score = mean([float(v["confidence"]) for v in vectors])
        faulty = [l for l in labels if l]
        if len(faulty) > len(labels) / 2:
            kind = Counter(faulty).most_common(1)[0][0]
            out["faulty"][ATTACK_NAMES.get(kind, str(kind))].append(score)
        else:
            out["touched" if faulty else "clean"].append(score)
    out["honest"] = out["clean"] + out["touched"]
    out["faulty_all"] = [s for v in out["faulty"].values() for s in v]
    return out


def main():
    rows = load(sys.argv[1])

    print("vector level (rejected = flagged)")
    print(f"  {'source':<20}{'n':>8}{'fused':>9}{'rejected':>10}{'started':>9}")
    for key, c in vector_outcomes(rows).items():
        n = sum(c.values())
        print(f"  {key:<20}{n:>8}{100 * c['fused'] / n:>8.1f}%{100 * c['rejected'] / n:>9.1f}%{100 * c['started'] / n:>8.1f}%")

    t = track_scores(rows)
    print("\ntrack level (score = mean confidence over the track's vectors)")
    print(f"  honest tracks {len(t['honest'])}: mean score {mean(t['honest']):.3f}")
    print(f"    fed only by genuine vectors {len(t['clean'])}: {mean(t['clean']):.3f}; "
          f"also by faulty ones {len(t['touched'])}: {mean(t['touched']):.3f}")
    print(f"  faulty tracks {len(t['faulty_all'])}: mean score {mean(t['faulty_all']):.3f}")
    print(f"  AUC honest > faulty: {auc(t['honest'], t['faulty_all']):.3f}")
    for kind in sorted(t["faulty"]):
        s = t["faulty"][kind]
        print(f"    {kind:<20} tracks {len(s):>4}  mean {mean(s):.3f}  AUC {auc(t['honest'], s):.3f}")


if __name__ == "__main__":
    main()
