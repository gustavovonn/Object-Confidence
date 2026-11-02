#!/usr/bin/env python3
"""Association quality per fault subset, against the station each vector really describes.

Usage: association_eval.py <results dir>
Reads <dir>/<fault type>/records.csv (scripts/run_preliminary.sh) and writes <dir>/association.md.

Ground truth: `trueStation`, the station that sent the CAM or that the CPM object is (oracle);
vectors without one (-1) and dropped vectors are left out of the metrics. Tracks are receiver-local.
  Purity        share of vectors whose track's majority station is their own (all vectors)
  Mixed         tracks holding vectors of more than one station
  k             most tracks one (receiver, station) is in at the same time, over honest vectors
                (attackType 0); reported as its mean and as the share of objects with k >= 2
  Honest rejected  honest vectors associated to a track and rejected by its checks, per origin

k replays the engine's track cache from the records, in processing order: a track starts with its
`started` vector, `fused` vectors renew it (track time = latest vector time), and before each message
the tracks whose time is more than RETIRE_AFTER before the message's reception time are retired. A
station is in a track from each of its vectors there until RETIRE_AFTER after the last one, or until
the track retires. Fused vectors without velocity, which the engine does not fuse, are not told apart.

The fault-free baseline (Honest) comes first and stays out of the total, which sums the fault subsets.
"""
import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path

BASELINE = "Honest"
TYPES = ["ConstPos", "ConstPosOffset", "RandomPos", "RandomPosOffset",
         "ConstSpeed", "ConstSpeedOffset", "RandomSpeed", "RandomSpeedOffset"]

# include/MechanismConstants.h, ConstantsAssociation::DEFAULT_RETIRE_AFTER [s]
RETIRE_AFTER = 3.1

K_COLUMNS = (1, 2, 3, 4)  # last column: k >= 4


def load(path):
    """Vectors that reached a track, in processing order."""
    with open(path) as f:
        return [r for r in csv.DictReader(f) if r["outcome"] != "dropped" and r["trackId"] != "0"]


def simultaneous(rows):
    """{(receiver, station): k}, the most live tracks holding the station's honest vectors at once."""
    k = Counter()
    receiver = None
    now = None
    trackTime = {}                   # live track -> latest fused (or starting) vector time
    member = defaultdict(dict)       # station -> {live track: latest vector time of the station there}
    for r in rows:
        if r["receiver"] != receiver:
            receiver, now = r["receiver"], None
            trackTime.clear()
            member.clear()
        rcv = float(r["rcvTime"])
        if rcv != now:
            # a new message: the engine retires stale tracks first
            now = rcv
            for tid in [tid for tid, t in trackTime.items() if now - t > RETIRE_AFTER]:
                del trackTime[tid]
        tid, t = r["trackId"], float(r["t"])
        if r["outcome"] == "started":
            trackTime[tid] = t
        elif r["outcome"] == "fused":
            trackTime[tid] = max(trackTime.get(tid, t), t)
        if r["attackType"] != "0" or r["trueStation"] == "-1":
            continue
        tracks = member[r["trueStation"]]
        tracks[tid] = max(tracks.get(tid, t), t)
        for other in [o for o, last in tracks.items() if o not in trackTime or now - last > RETIRE_AFTER]:
            del tracks[other]
        key = (receiver, r["trueStation"])
        k[key] = max(k[key], len(tracks))
    return k


def stats(rows):
    """Counts, summable across subsets."""
    stations = defaultdict(Counter)  # (receiver, trackId) -> stations of its vectors
    rejected, seen = Counter(), Counter()
    vectors = 0
    for r in rows:
        if r["trueStation"] == "-1":
            continue
        vectors += 1
        stations[(r["receiver"], r["trackId"])][r["trueStation"]] += 1
        if r["attackType"] == "0":
            seen[r["origin"]] += 1
            rejected[r["origin"]] += r["outcome"] == "rejected"
    k = simultaneous(rows)
    s = Counter({
        "vectors": vectors,
        "tracks": len(stations),
        "majority": sum(c.most_common(1)[0][1] for c in stations.values()),
        "mixed": sum(1 for c in stations.values() if len(c) > 1),
        "objects": len(k),
        "kSum": sum(k.values()),
        "camSeen": seen["cam"], "camRejected": rejected["cam"],
        "cpmSeen": seen["cpm"], "cpmRejected": rejected["cpm"],
    })
    for v in k.values():
        s[f"k{min(v, K_COLUMNS[-1])}"] += 1
    return s


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def row(name, s):
    return "| {} | {} | {} | {:.2f} | {} | {:.2f} | {:.2f} | {:.2f} | {:.2f} |".format(
        name, s["vectors"], s["tracks"], pct(s["majority"], s["vectors"]), s["mixed"],
        s["kSum"] / s["objects"] if s["objects"] else 0.0, 100.0 - pct(s["k1"], s["objects"]),
        pct(s["camRejected"], s["camSeen"]), pct(s["cpmRejected"], s["cpmSeen"]))


def kRow(name, s):
    return "| {} | {} | ".format(name, s["objects"]) + " | ".join(
        "{} ({:.1f}%)".format(s[f"k{c}"], pct(s[f"k{c}"], s["objects"])) for c in K_COLUMNS) + " |"


def table(root):
    lines = ["| Fault | Vectors | Tracks | Purity (%) | Mixed | Mean k | Objects with k >= 2 (%) "
             "| Honest CAM rejected (%) | Honest CPM rejected (%) |",
             "|---|---|---|---|---|---|---|---|---|"]
    kLines = ["| Fault | Objects | " + " | ".join(f"k = {c}" for c in K_COLUMNS[:-1])
              + f" | k >= {K_COLUMNS[-1]} |",
              "|---|---|" + "---|" * len(K_COLUMNS)]
    total = Counter()
    for kind in [BASELINE] + TYPES:
        s = stats(load(root / kind / "records.csv"))
        if kind != BASELINE:
            total.update(s)
        lines.append(row(kind, s))
        kLines.append(kRow(kind, s))
    lines.append(row("**all**", total))
    kLines.append(kRow("**all**", total))
    return "\n".join(lines) + "\n\nk: most tracks one (receiver, station) is in at the same time\n\n" \
        + "\n".join(kLines) + "\n"


def main():
    root = Path(sys.argv[1])
    text = table(root)
    (root / "association.md").write_text(text)
    print(text)
    print(f"wrote {root / 'association.md'}")


if __name__ == "__main__":
    main()
