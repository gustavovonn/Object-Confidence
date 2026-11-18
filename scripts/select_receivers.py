#!/usr/bin/env python3
"""Receivers for the latency benchmark, stratified by neighbourhood size: whole receivers, never the first rows of
a file (the engine's state, and with it the neighbourhood, builds up over a receiver's own history).

Usage: select_receivers.py <receptions.csv> <latency.csv> <out.csv> [--receivers N]
  receptions.csv  scripts/flatten_dataset.py output (rows grouped by receiver)
  latency.csv     build/replay <receptions.csv> /dev/null --latency-out <latency.csv>, on the same file
  out.csv         header and every row of the chosen receivers, in the input order
The receivers are sorted by their mean live tracks per message and one is taken at each of N evenly spaced
quantiles ((i + 0.5) / N), so sparse, median and dense neighbourhoods are all present. Default N: 20.
"""
import csv
import sys
from collections import defaultdict

from common import option


def main():
    args = sys.argv[1:]
    n = int(option(args, "--receivers", "20"))
    if len(args) != 3:
        sys.exit(__doc__)
    receptions, latency, out = args

    tracks, messages = defaultdict(int), defaultdict(int)
    with open(latency) as f:
        for r in csv.DictReader(f):
            tracks[r["receiver"]] += int(r["liveTracks"])
            messages[r["receiver"]] += 1
    ranked = sorted(messages, key=lambda rcv: tracks[rcv] / messages[rcv])
    chosen = sorted({ranked[min(len(ranked) - 1, int((i + 0.5) * len(ranked) / n))] for i in range(n)},
                    key=lambda rcv: tracks[rcv] / messages[rcv])

    with open(receptions) as fi, open(out, "w") as fo:
        fo.write(fi.readline())
        keep = set(chosen)
        for line in fi:
            if line.split(",", 2)[1] in keep:
                fo.write(line)

    print("| Receiver | Messages | Mean live tracks |\n|---|---|---|")
    for rcv in chosen:
        print(f"| {rcv} | {messages[rcv]} | {tracks[rcv] / messages[rcv]:.1f} |")
    print(f"\n{len(chosen)} of {len(ranked)} receivers, {sum(messages[r] for r in chosen)} messages -> {out}")


if __name__ == "__main__":
    main()
