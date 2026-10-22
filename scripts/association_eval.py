#!/usr/bin/env python3
"""Association quality per fault subset, against the station each vector really describes.

Usage: association_eval.py <results dir>
Reads <dir>/<fault type>/records.csv (scripts/run_preliminary.sh) and writes <dir>/association.md.

Ground truth: `trueStation`, the station that sent the CAM or that the CPM object is (oracle);
vectors without one (-1) and dropped vectors are left out. Tracks are receiver-local.
  Purity        share of vectors whose track's majority station is their own (all vectors)
  Mixed         tracks holding vectors of more than one station
  Tracks/object tracks per (receiver, station), over honest vectors (attackType 0)
  Completeness  share of a (receiver, station)'s honest vectors in its largest track
  ID switches   track changes between consecutive honest vectors of one (receiver, station),
                in reception order, per 100 vectors
  Honest rejected  honest vectors associated to a track and rejected by its checks, per origin
"""
import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path

TYPES = ["ConstPos", "ConstPosOffset", "RandomPos", "RandomPosOffset",
         "ConstSpeed", "ConstSpeedOffset", "RandomSpeed", "RandomSpeedOffset"]


def load(path):
    with open(path) as f:
        return [r for r in csv.DictReader(f)
                if r["outcome"] != "dropped" and r["trackId"] != "0" and r["trueStation"] != "-1"]


def stats(rows):
    """Counts, summable across subsets."""
    stations = defaultdict(Counter)  # (receiver, trackId) -> stations of its vectors
    honest = defaultdict(list)       # (receiver, station) -> trackIds of its honest vectors, in order
    rejected, seen = Counter(), Counter()
    for r in sorted(rows, key=lambda r: (r["receiver"], float(r["rcvTime"]))):
        stations[(r["receiver"], r["trackId"])][r["trueStation"]] += 1
        if r["attackType"] == "0":
            honest[(r["receiver"], r["trueStation"])].append(r["trackId"])
            seen[r["origin"]] += 1
            rejected[r["origin"]] += r["outcome"] == "rejected"
    return {
        "vectors": len(rows),
        "tracks": len(stations),
        "majority": sum(c.most_common(1)[0][1] for c in stations.values()),
        "mixed": sum(1 for c in stations.values() if len(c) > 1),
        "objects": len(honest),
        "objectTracks": sum(len(set(ids)) for ids in honest.values()),
        "honest": sum(len(ids) for ids in honest.values()),
        "inLargest": sum(Counter(ids).most_common(1)[0][1] for ids in honest.values()),
        "switches": sum(sum(a != b for a, b in zip(ids, ids[1:])) for ids in honest.values()),
        "camSeen": seen["cam"], "camRejected": rejected["cam"],
        "cpmSeen": seen["cpm"], "cpmRejected": rejected["cpm"],
    }


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def row(name, s):
    return "| {} | {} | {} | {:.2f} | {} | {:.2f} | {:.2f} | {:.2f} | {:.2f} | {:.2f} |".format(
        name, s["vectors"], s["tracks"], pct(s["majority"], s["vectors"]), s["mixed"],
        s["objectTracks"] / s["objects"] if s["objects"] else 0.0, pct(s["inLargest"], s["honest"]),
        pct(s["switches"], s["honest"]), pct(s["camRejected"], s["camSeen"]),
        pct(s["cpmRejected"], s["cpmSeen"]))


def table(root):
    lines = ["| Fault | Vectors | Tracks | Purity (%) | Mixed | Tracks/object | Completeness (%) "
             "| ID switches (/100) | Honest CAM rejected (%) | Honest CPM rejected (%) |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    total = Counter()
    for kind in TYPES:
        s = stats(load(root / kind / "records.csv"))
        total.update(s)
        lines.append(row(kind, s))
    lines.append(row("**all**", total))
    return "\n".join(lines) + "\n"


def main():
    root = Path(sys.argv[1])
    text = table(root)
    (root / "association.md").write_text(text)
    print(text)
    print(f"wrote {root / 'association.md'}")


if __name__ == "__main__":
    main()
