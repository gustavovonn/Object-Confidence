#!/usr/bin/env python3
"""Object-Confidence filter over the generator's receiver view: one CSV row per received motion vector, no conversion.

Usage: flatten_dataset.py <run dir with receivers/, from libs/Artery-Dataset-Generator/scripts/receiver_view.py> <out.csv>
Rows sorted by (receiver, rcvTime, messageID); fields as in libs/Artery-Dataset-Generator/trace_schema.md.
Dropped here, not in the generator: CPMs without a reference position, and CPMs without objects (no motion vector).
attackType from an injected subset (0 = genuine); a CPM object row takes its message's.
"""
import csv
import json
import sys
from pathlib import Path

COLUMNS = [
    "kind", "receiver", "rcvTime", "messageID", "sender", "t", "lat", "lon",
    "hasVelocity", "speed", "heading", "yawRate", "length",
    "objectId", "x", "y", "direction", "trueStation", "attackType",
]


def num(value):
    return "nan" if value is None else value


def main():
    run, out = Path(sys.argv[1]), Path(sys.argv[2])
    view = run / "receivers"
    if not view.is_dir():
        sys.exit(f"{view} missing: run libs/Artery-Dataset-Generator/scripts/receiver_view.py {run}")

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        # one receiver in memory at a time
        for path in sorted(view.glob("*.jsonl"), key=lambda p: int(p.stem)):
            with open(path) as r:
                receptions = sorted(map(json.loads, r), key=lambda m: (m["rcvTime"], m["messageID"]))
            for m in receptions:
                head = [m["receiver"], m["rcvTime"], m["messageID"]]
                if m["msg"] == "cam":
                    has_velocity = m["speed"] is not None and m["heading"] is not None
                    w.writerow(["cam", *head, m["sender"], m["refSimTime"], m["lat"], m["lon"],
                                int(has_velocity), num(m["speed"]), num(m["heading"]), num(m["yawRate"]),
                                num(m["vehicleLength"]), -1, "nan", "nan", "nan", m["sender"],
                                m.get("attackType", 0)])
                elif m["lat"] is not None:
                    for o in m["objects"]:
                        has_velocity = o["speed"] is not None and o["direction"] is not None
                        station = o["objectStation"] if o["objectStation"] is not None else -1
                        w.writerow(["cpm", *head, m["sender"], o["measTime"], m["lat"], m["lon"],
                                    int(has_velocity), num(o["speed"]), "nan", "nan", "nan",
                                    o["objectId"] if o["objectId"] is not None else -1,
                                    o["x"], o["y"], num(o["direction"]), station, m.get("attackType", 0)])


if __name__ == "__main__":
    main()
