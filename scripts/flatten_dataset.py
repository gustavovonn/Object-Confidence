#!/usr/bin/env python3
"""Joins receptions with message content: one CSV row per received motion vector, no conversion.

Usage: flatten_dataset.py <run dir: cam_sent.jsonl, cpm_sent.jsonl, reception.jsonl> <out.csv>
Rows sorted by (receiver, rcvTime, messageID); fields as in Artery-Dataset-Generator/trace_schema.md;
attackType from an injected subset (0 = genuine, CPM objects always 0).
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


def load(path):
    with open(path) as f:
        return {m["messageID"]: m for m in map(json.loads, f)}


def main():
    run, out = Path(sys.argv[1]), Path(sys.argv[2])
    cams = load(run / "cam_sent.jsonl")
    cpms = load(run / "cpm_sent.jsonl")
    with open(run / "reception.jsonl") as f:
        receptions = sorted(map(json.loads, f), key=lambda r: (r["receiver"], r["rcvTime"], r["messageID"]))

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for rec in receptions:
            head = [rec["receiver"], rec["rcvTime"], rec["messageID"]]
            if rec["msg"] == "cam":
                m = cams.get(rec["messageID"])
                if m is None:
                    continue
                has_velocity = m["speed"] is not None and m["heading"] is not None
                w.writerow(["cam", *head, m["sender"], m["refSimTime"], m["lat"], m["lon"],
                            int(has_velocity), num(m["speed"]), num(m["heading"]), num(m["yawRate"]),
                            num(m["vehicleLength"]), -1, "nan", "nan", "nan", m["sender"],
                            m.get("attackType", 0)])
            else:
                m = cpms.get(rec["messageID"])
                if m is None or m["lat"] is None:
                    continue
                for o in m["objects"]:
                    has_velocity = o["speed"] is not None and o["direction"] is not None
                    station = o["objectStation"] if o["objectStation"] is not None else -1
                    w.writerow(["cpm", *head, m["sender"], o["measTime"], m["lat"], m["lon"],
                                int(has_velocity), num(o["speed"]), "nan", "nan", "nan",
                                o["objectId"] if o["objectId"] is not None else -1,
                                o["x"], o["y"], num(o["direction"]), station, 0])


if __name__ == "__main__":
    main()
