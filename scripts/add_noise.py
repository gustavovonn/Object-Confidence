#!/usr/bin/env python3
"""White Gaussian position noise on flattened receptions (scripts/flatten_dataset.py).

Usage: add_noise.py <receptions.csv> <out.csv> <sigma [m]> [seed, default 1]
Replay the output with `replay ... --meas-noise <sigma>`, so the UKF's position R matches the injected noise.

One draw (east, north) ~ N(0, sigma^2) per message, added to its lat/lon: a CAM's position, a CPM's reference
position. The objects of a CPM inherit their author's error through the reference position; the perceived offsets
(x, y), speeds and headings stay exact (no perception noise). Draws are keyed by (seed, kind, messageID), so every
receiver of a message sees the same noisy message, and a rerun gives the same file. Streaming, row by row.
"""
import csv
import hashlib
import random
import sys

from common import shift



def draw(seed, kind, message_id, sigma):
    rng = random.Random(hashlib.sha256(f"{seed}|{kind}|{message_id}".encode()).digest())
    return rng.gauss(0.0, sigma), rng.gauss(0.0, sigma)


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    src, out, sigma = sys.argv[1], sys.argv[2], float(sys.argv[3])
    seed = sys.argv[4] if len(sys.argv) == 5 else "1"
    with open(src, newline="") as fi, open(out, "w", newline="") as fo:
        reader, writer = csv.reader(fi), csv.writer(fo)
        head = next(reader)
        writer.writerow(head)
        col = {name: i for i, name in enumerate(head)}
        kind, mid, lat, lon = col["kind"], col["messageID"], col["lat"], col["lon"]
        for row in reader:
            if sigma > 0.0:
                east, north = draw(seed, row[kind], row[mid], sigma)
                new_lat, new_lon = shift(float(row[lat]), float(row[lon]), east, north)
                row[lat], row[lon] = repr(new_lat), repr(new_lon)
            writer.writerow(row)


if __name__ == "__main__":
    main()
