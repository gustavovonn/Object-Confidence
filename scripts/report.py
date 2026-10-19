#!/usr/bin/env python3
"""Summary of a replay against the objectStation oracle.

Usage: report.py <records.csv>
"""
import csv
import sys
from collections import Counter, defaultdict


def pct(values, q):
    if not values:
        return float("nan")
    values = sorted(values)
    return values[min(len(values) - 1, int(q * (len(values) - 1) + 0.5))]


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    print(f"{len(rows)} motion vectors, {len({r['receiver'] for r in rows})} receivers")

    # outcomes per container
    by_origin = defaultdict(Counter)
    for r in rows:
        by_origin[r["origin"]][r["outcome"]] += 1
    for origin, c in sorted(by_origin.items()):
        total = sum(c.values())
        print(f"  {origin}: " + ", ".join(f"{k} {100.0 * v / total:.1f}%" for k, v in sorted(c.items())))
    print(f"  consistency failures: {sum(r['consistencyFailed'] == '1' for r in rows)}")

    # residual to the projected track
    for outcome in ("fused", "rejected"):
        res = [float(r["residual"]) for r in rows if r["outcome"] == outcome]
        print(f"  residual {outcome}: p50 {pct(res, .5):.2f} m, p95 {pct(res, .95):.2f} m, n {len(res)}")

    # purity: share of a track's vectors from its majority true station
    tracks = defaultdict(Counter)
    for r in rows:
        if r["trackId"] != "0" and r["trueStation"] != "-1":
            tracks[(r["receiver"], r["trackId"])][r["trueStation"]] += 1
    purity = [c.most_common(1)[0][1] / sum(c.values()) for c in tracks.values()]
    print(f"  tracks {len(tracks)}, purity p5 {pct(purity, .05):.2f}, mean {sum(purity) / len(purity):.3f}")

    # fragmentation: tracks used per (receiver, true object)
    per_object = defaultdict(set)
    for (receiver, track), c in tracks.items():
        per_object[(receiver, c.most_common(1)[0][0])].add(track)
    frag = [len(s) for s in per_object.values()]
    print(f"  tracks per true object: mean {sum(frag) / len(frag):.2f}, max {max(frag)}")

    # final confidence of each track
    last = {}
    for r in rows:
        if r["trackId"] != "0":
            last[(r["receiver"], r["trackId"])] = float(r["confidence"])
    conf = list(last.values())
    print(f"  final confidence: p5 {pct(conf, .05):.3f}, p50 {pct(conf, .5):.3f}, p95 {pct(conf, .95):.3f}")


if __name__ == "__main__":
    main()
