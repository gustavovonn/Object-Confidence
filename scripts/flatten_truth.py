#!/usr/bin/env python3
"""Ground truth for the UKF calibration (tools/calibrate_ukf.cc): simulator poses and report times, no conversion.

Usage: flatten_truth.py <ground-truth run dir> <out dir>
Writes to <out dir>:
  truth.csv         station,time,lat,lon,heading,speed,accel,yawRate,length: vehicle_pose.jsonl (oracle, every
                    poseInterval) with the SUMO length of vehicles.jsonl; sorted by (station, time)
  report_times.csv  station,t,kind: the times a station is reported, CAM refSimTime of its own CAMs and CPM
                    measTime of the objects whose objectStation it is; sorted by (station, t)
Fields as in libs/Artery-Dataset-Generator/trace_schema.md; poses with a null field are dropped.
"""
import csv
import json
import sys
from pathlib import Path

POSE = ["station", "time", "lat", "lon", "heading", "speed", "accel", "yawRate"]


def main():
    run, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)

    length = {}
    with open(run / "vehicles.jsonl") as f:
        for v in map(json.loads, f):
            length[v["station"]] = v["length"]

    poses = []
    with open(run / "vehicle_pose.jsonl") as f:
        for p in map(json.loads, f):
            if all(p[k] is not None for k in POSE) and length.get(p["station"]) is not None:
                poses.append([p[k] for k in POSE] + [length[p["station"]]])
    poses.sort(key=lambda r: (r[0], r[1]))
    with open(out / "truth.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(POSE + ["length"])
        w.writerows(poses)

    reports = []
    with open(run / "cam_sent.jsonl") as f:
        for m in map(json.loads, f):
            reports.append((m["sender"], m["refSimTime"], "cam"))
    with open(run / "cpm_sent.jsonl") as f:
        for m in map(json.loads, f):
            for o in m["objects"] or []:
                if o["objectStation"] is not None:
                    reports.append((o["objectStation"], o["measTime"], "cpm"))
    reports.sort()
    with open(out / "report_times.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["station", "t", "kind"])
        w.writerows(reports)

    print(f"{len(poses)} poses, {len(reports)} reports -> {out}")


if __name__ == "__main__":
    main()
