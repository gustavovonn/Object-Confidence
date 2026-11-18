#!/usr/bin/env python3
"""Episodes of the fault-free run (confidence drops, or rejected vectors), as material to diagnose them: the
attributes of each episode's onset, the same attributes for a random control sample, and the vehicle's ground
truth aligned at the onset.

Usage: error_diagnosis.py <records.csv> <receptions.csv> <subset dir> <out dir> [--episodes <criterion>]
records.csv (build/replay) and receptions.csv (scripts/flatten_dataset.py) are row-aligned; the subset
dir holds the generator's vehicle_pose.jsonl. Writes to <out dir>:
  episodes.csv              one row per episode: onset attributes, length, duration, physically possible
  controls.csv              the same attributes for vectors fused with confidence >= THRESHOLD
  onset_ground_truth.png    vehicle ground truth from -WINDOW to +WINDOW around the onset: median,
                            mean and interquartile range, episodes against controls
  summary.md                onset attributes, failed checks, lengths, lateral-acceleration bands
  events.csv, events.md     episodes of one vehicle starting within EVENT_GROUP s of each other (receivers
                            seeing the same moment) as one event: SUMO time and id, receivers, and the
                            scripts/view_in_sumo.sh command that opens sumo-gui on it

Episode: a maximal run of one track's vectors meeting the criterion, per receiver; its onset is the first of them.
Criteria (--episodes, default confidence):
  confidence   confidence < THRESHOLD (common.py)
  rejected     rejected: a check failed
  predictive   the predictive check failed: residual above maxPosDev (predictiveFailed)
Onset attributes:
  check        check failed by the onset vector (consistency, predictive, both), empty when it was fused
  residual     position residual to the projected track [m]
  dtSource     time since the previous report of the same source [s]
  refAge       time since the track's latest fused report [s]
  trackAge     time since the track started [s]
  sources      sources fused into the track within RETIRE_AFTER
  speedGap     |reported speed - speed implied by the displacement since the source's previous report| [m/s]
  gt*          ground truth of the true station at the onset: speed, accel, yaw rate (from the heading)
               and lateral acceleration v * yaw rate
  latAccelPeak largest ground-truth |lateral acceleration| within +-PEAK_WINDOW of the onset [m/s^2]
  sumoArtifact 1 when the ground truth is not physically possible over the window the onset's checks compared:
               between the track's latest fused report and the onset (every check is against the track; an
               out-of-sequence onset is older than that report), STEP of margin. A step is impossible when its displacement departs
               from the ground-truth speed by more than MAX_STEP_GAP (a position jump: SUMO's length-geometry
               mismatch on a lane change in a curve), |accel| exceeds MAX_ACCEL, or v * heading change / STEP
               exceeds MAX_LAT_ACCEL (tyre-road friction; SUMO's ramp-node connectors).
An episode is physically possible when its onset has no sumoArtifact: the drop is then the mechanism's to explain.
Streaming: the files are read row by row and the controls drawn by reservoir sampling.
"""
import csv
import json
import math
import random
import sys
from collections import defaultdict
from itertools import zip_longest
from pathlib import Path

import numpy as np

from common import RETIRE_AFTER, STEP, THRESHOLD, east_north, open_records, sumo_clock

CRITERIA = {
    "confidence": (f"confidence < {THRESHOLD}", lambda a: a["confidence"] < THRESHOLD),
    "rejected": ("rejected by a check", lambda a: a["outcome"] == "rejected"),
    "predictive": ("predictive check failed (residual above maxPosDev)", lambda a: a["predictiveFailed"]),
}

WINDOW = 3.0        # aligned profiles from -WINDOW to +WINDOW around the onset [s]
PEAK_WINDOW = 0.5   # lateral-acceleration peak around the onset [s]
CONTROLS_PER_EPISODE = 5
SEED = 1
RESERVOIR = 20000   # controls kept per origin while streaming, then drawn to CONTROLS_PER_EPISODE per episode

# Friction limit mu * g, mu = 0.8 for a passenger car on dry asphalt (Gillespie, Fundamentals of
# Vehicle Dynamics, SAE 1992) [m/s^2]
MAX_LAT_ACCEL = 8.0
# Drivers rarely exceed it (Bosetti, Da Lio & Saroldi, Eur. Transp. Res. Rev. 2014) [m/s^2]
COMFORT_LAT_ACCEL = 4.0
# |displacement / STEP - speed| above it is a position jump, not motion [m/s]
MAX_STEP_GAP = 1.0
# Beyond a passenger car's emergency braking (~0.9 g) [m/s^2]
MAX_ACCEL = 9.0

# Events and their SUMO time. The state recorded at dataset time t is the latest SUMO step at or before it,
# t0 + floor(t / STEP) * STEP, t0 the run's common.sumo_clock (TraCI positions identical, lust-full-run0)
EVENT_GROUP = 0.5     # [s]

FIELDS = ["receiver", "trackId", "station", "origin", "t", "outcome", "check", "residual", "confidence",
          "dtSource", "refAge", "trackAge", "sources", "speedReported", "speedImplied", "speedGap",
          "gtSpeed", "gtAccel", "gtYawRate", "gtLatAccel", "latAccelPeak"]
SUMMARY = ["residual", "dtSource", "refAge", "trackAge", "sources", "speedGap", "gtSpeed", "gtAccel",
           "gtYawRate", "gtLatAccel"]
ABSOLUTE = ("gtAccel", "gtYawRate", "gtLatAccel")


def load_pose(path):
    """station -> (times, speed, accel, yaw rate [deg/s] from the heading)."""
    raw = defaultdict(list)
    with open(path) as f:
        for line in f:
            p = json.loads(line)
            raw[p["station"]].append((p["time"], p["speed"], p["accel"], p["heading"]))
    pose = {}
    for station, samples in raw.items():
        samples.sort()
        t = np.array([s[0] for s in samples])
        heading = np.unwrap(np.radians([s[3] for s in samples]))
        # compass heading grows clockwise: the yaw rate (counter-clockwise) is its negative derivative
        yaw = -np.degrees(np.gradient(heading, t)) if len(t) > 1 else np.zeros_like(t)
        pose[station] = (t, np.array([s[1] for s in samples]), np.array([s[2] for s in samples]), yaw)
    return pose


def step_of(t):
    return round(math.floor(t / STEP + 1e-6) * STEP, 1)


def impossible_steps(path):
    """(station, step) of every ground-truth step a vehicle cannot drive (see sumoArtifact)."""
    steps = {}
    with open(path) as f:
        for line in f:
            p = json.loads(line)
            steps[(p["station"], step_of(p["time"]))] = (p["x"], p["y"], p["speed"], p["accel"], p["heading"])
    bad = set()
    for (station, t), a in steps.items():
        b = steps.get((station, round(t + STEP, 1)))
        if b is None:
            continue
        turn = math.radians(abs((b[4] - a[4] + 180.0) % 360.0 - 180.0))
        if (abs(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP - 0.5 * (a[2] + b[2])) > MAX_STEP_GAP
                or max(abs(a[3]), abs(b[3])) > MAX_ACCEL or b[2] * turn / STEP > MAX_LAT_ACCEL):
            bad.add((station, t))
    return bad


def artifact(bad, row):
    """1 when the window the row's checks compared holds an impossible ground-truth step (see sumoArtifact)."""
    age = row["refAge"] if math.isfinite(row["refAge"]) else 0.0
    s, end = step_of(row["t"] - max(age, 0.0) - STEP), step_of(row["t"] + max(-age, 0.0))
    while s <= end + 1e-9:
        if (row["station"], round(s, 1)) in bad:
            return 1
        s += STEP
    return 0


def ground_truth(pose, station, times):
    """(speed, accel, yaw rate) at the pose samples nearest to `times`, NaN farther than STEP."""
    out = np.full((3, len(times)), np.nan)
    if station not in pose:
        return out
    t, speed, accel, yaw = pose[station]
    i = np.clip(np.searchsorted(t, times), 1, len(t) - 1) if len(t) > 1 else np.zeros(len(times), int)
    if len(t) > 1:
        i = np.where(np.abs(t[i - 1] - times) < np.abs(t[i] - times), i - 1, i)
    near = np.abs(t[i] - times) <= STEP
    out[0, near], out[1, near], out[2, near] = speed[i[near]], accel[i[near]], yaw[i[near]]
    return out


def enu(m, origin):
    """Vector position [m], east and north of `origin` (true ENU, as the CPM offsets)."""
    x, y = east_north(float(m["lat"]), float(m["lon"]), origin)
    if m["kind"] == "cam":
        return x, y
    return x + float(m["x"]), y + float(m["y"])


def number(text):
    return float(text) if text not in ("", "nan") else math.nan


def check_of(r):
    """Failed checks of a record; records without predictiveFailed (older replays) infer it from the outcome."""
    if r["outcome"] != "rejected":
        return ""
    consistency = r["consistencyFailed"] == "1"
    predictive = r["predictiveFailed"] == "1" if "predictiveFailed" in r else not consistency
    return "both" if consistency and predictive else "consistency" if consistency else "predictive"


def attributes(records, receptions, pose):
    """Attributes of every vector at its processing time, None when dropped, in processing order (a generator
    over row-aligned record and reception rows)."""
    origin = None
    receiver = None
    track_time, track_start, track_sources, last_report = {}, {}, {}, {}
    for r, m in zip_longest(records, receptions):
        if r is None or m is None:
            sys.exit("records and receptions are not row-aligned")
        if origin is None:
            origin = (float(m["lat"]), float(m["lon"]))
        if r["receiver"] != receiver:
            receiver = r["receiver"]
            track_time.clear(); track_start.clear(); track_sources.clear(); last_report.clear()
        if r["outcome"] == "dropped":
            yield None
            continue
        t, rcv = float(r["t"]), float(r["rcvTime"])
        # the engine retires stale tracks before each message (association_eval.simultaneous)
        for tid in [k for k, v in track_time.items() if rcv - v > RETIRE_AFTER]:
            del track_time[tid]
        tid, source = r["trackId"], (r["author"], r["objectId"])
        x, y = enu(m, origin)
        prev = last_report.get(source)
        dt_source = t - prev[0] if prev else math.nan
        implied = math.hypot(x - prev[1], y - prev[2]) / dt_source if prev and dt_source > 0 else math.nan
        speed = number(m["speed"])
        station = int(r["trueStation"])
        gt = ground_truth(pose, station, np.array([t]))[:, 0]
        yield {
            "receiver": receiver, "trackId": tid, "station": station, "origin": r["origin"], "t": t,
            "outcome": r["outcome"],
            "check": check_of(r),
            "predictiveFailed": r.get("predictiveFailed") == "1",
            "residual": float(r["residual"]), "confidence": float(r["confidence"]),
            "dtSource": dt_source, "refAge": t - track_time[tid] if tid in track_time else math.nan,
            "trackAge": t - track_start[tid] if tid in track_start else 0.0,
            "sources": sum(1 for last in track_sources.get(tid, {}).values() if t - last <= RETIRE_AFTER),
            "speedReported": speed, "speedImplied": implied,
            "speedGap": abs(speed - implied) if math.isfinite(speed) and math.isfinite(implied) else math.nan,
            "gtSpeed": gt[0], "gtAccel": gt[1], "gtYawRate": gt[2], "gtLatAccel": gt[0] * math.radians(gt[2]),
        }
        # engine state after this vector
        if r["outcome"] == "started":
            track_time[tid] = track_start[tid] = t
        elif r["outcome"] == "fused":
            track_time[tid] = max(track_time.get(tid, t), t)
            track_sources.setdefault(tid, {})[source] = t
        last_report[source] = (t, x, y)


def add_lat_accel_peak(pose, rows):
    """latAccelPeak of each row: largest ground-truth |v * yaw rate| within +-PEAK_WINDOW, NaN unknown."""
    offsets = np.arange(-PEAK_WINDOW, PEAK_WINDOW + STEP / 2, STEP)
    for r in rows:
        speed, _, yaw = ground_truth(pose, r["station"], r["t"] + offsets)
        with np.errstate(all="ignore"):
            r["latAccelPeak"] = np.nanmax(np.abs(speed * np.radians(yaw))) if np.isfinite(speed).any() else math.nan


def episodes_of(attrs, criterion, rng):
    """Onset attributes of every episode, with its length [vectors] and duration [s]; and per origin a uniform
    reservoir of control candidates (fused, confidence >= THRESHOLD, a known station)."""
    inside = CRITERIA[criterion][1]
    episodes, open_episode = [], {}
    reservoir, seen = {"cam": [], "cpm": []}, {"cam": 0, "cpm": 0}
    for a in attrs:
        if a is None:
            continue
        if a["outcome"] == "fused" and a["confidence"] >= THRESHOLD and a["station"] >= 0 and a["origin"] in seen:
            o = a["origin"]
            seen[o] += 1
            if len(reservoir[o]) < RESERVOIR:
                reservoir[o].append(a)
            else:
                k = rng.randrange(seen[o])
                if k < RESERVOIR:
                    reservoir[o][k] = a
        key = (a["receiver"], a["trackId"])
        if inside(a):
            if key not in open_episode:
                open_episode[key] = dict(a, length=0, duration=0.0)
                episodes.append(open_episode[key])
            open_episode[key]["length"] += 1
            open_episode[key]["duration"] = a["t"] - open_episode[key]["t"]
        else:
            open_episode.pop(key, None)
    return episodes, reservoir


def profiles(pose, rows, grid):
    """|v - v(0)|, |accel|, |yaw rate|, |lateral accel| on the aligned grid, one stack per row."""
    stack = []
    for r in rows:
        speed, accel, yaw = ground_truth(pose, r["station"], r["t"] + grid)
        v0 = ground_truth(pose, r["station"], np.array([r["t"]]))[0, 0]
        stack.append([np.abs(speed - v0), np.abs(accel), np.abs(yaw), np.abs(speed * np.radians(yaw))])
    return np.array(stack)


def plot(pose, episodes, controls, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    grid = np.arange(-WINDOW, WINDOW + STEP / 2, STEP)
    titles = ["|v - v(0)| [m/s]", "|accel| [m/s²]", "|yaw rate| [deg/s]", "|lateral accel| [m/s²]"]
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5), sharex=True)
    for rows, label, color in [(episodes, f"episodes (n={len(episodes)})", "#c0392b"),
                               (controls, f"controls (n={len(controls)})", "#2c7fb8")]:
        stack = profiles(pose, rows, grid)
        with np.errstate(all="ignore"):
            median, mean = np.nanmedian(stack, axis=0), np.nanmean(stack, axis=0)
            q1, q3 = np.nanpercentile(stack, 25, axis=0), np.nanpercentile(stack, 75, axis=0)
        for k, ax in enumerate(axes.flat):
            ax.plot(grid, median[k], color=color, label=f"{label}, median")
            ax.plot(grid, mean[k], color=color, ls="--", lw=1.2, label=f"{label}, mean")
            ax.fill_between(grid, q1[k], q3[k], color=color, alpha=0.2)
    for ax, title in zip(axes.flat, titles):
        ax.set_title(title, fontsize=10)
        ax.axvline(0, color="grey", lw=0.8, ls="--")
        ax.grid(alpha=0.3)
    for ax in axes[1]:
        ax.set_xlabel("time from onset [s]")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Ground truth of the vehicle around the onset (median, mean, interquartile range)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)


QUARTILES = ("Q1 (0-25 %)", "Q2 (25-50 %)", "Q3 (50-75 %)", "Q4 (75-100 %)")


def values(rows, field):
    v = np.array([abs(r[field]) if field in ABSOLUTE else r[field] for r in rows], dtype=float)
    return v[np.isfinite(v)]


def quartile_cells(rows, field):
    """[low, high] of each quartile of the field, Q1 to Q4; "-" when there are no values."""
    v = values(rows, field)
    if not len(v):
        return ["-"] * len(QUARTILES)
    edges = np.percentile(v, [0, 25, 50, 75, 100])
    return [f"{edges[i]:.2f} - {edges[i + 1]:.2f}" for i in range(len(QUARTILES))]


def control_bands(episodes, controls, field):
    """Bands at the controls' quartiles, plus below their min and above their max: (range, % of episodes)
    for each. A value goes to the first band whose upper edge it does not exceed, so tied edges leave
    empty bands."""
    c, e = values(controls, field), values(episodes, field)
    if not len(c):
        return [("-", "-")] * (len(QUARTILES) + 2)
    edges = np.percentile(c, [0, 25, 50, 75, 100])
    ranges = [f"< {edges[0]:.2f}"] + [f"{edges[i]:.2f} - {edges[i + 1]:.2f}" for i in range(4)] + [f"> {edges[4]:.2f}"]
    counts = [int(np.sum(e < edges[0]))]
    for i in range(4):
        above_low = e >= edges[0] if i == 0 else e > edges[i]
        counts.append(int(np.sum(above_low & (e <= edges[i + 1]))))
    counts.append(int(np.sum(e > edges[4])))
    return [(r, f"{100.0 * n / len(e):.1f}" if len(e) else "-") for r, n in zip(ranges, counts)]


def share(rows, test):
    return f"{100.0 * sum(1 for r in rows if test(r)) / len(rows):.1f}" if rows else "-"


def summary(source, episodes, controls, criterion):
    lines = [f"# Episodes ({criterion}): {source}", "",
             f"{len(episodes)} episodes ({CRITERIA[criterion][0]}), {len(controls)} controls "
             f"(fused, confidence >= {THRESHOLD}).", "",
             "## Onset attributes, quartile intervals (absolute accelerations and yaw rate)", "",
             "Each cell is the [min, max] of that quarter of its own column; n counts the finite values.", ""]
    groups = [[r for r in rows if r["origin"] == origin] for origin in ("cam", "cpm") for rows in (episodes, controls)]
    for field in SUMMARY:
        columns = [quartile_cells(g, field) for g in groups]
        lines += [f"### {field}", "",
                  "| Quartile | Episodes CAM | Controls CAM | Episodes CPM | Controls CPM |", "|---|---|---|---|---|",
                  "| n | " + " | ".join(str(len(values(g, field))) for g in groups) + " |"]
        for i, name in enumerate(QUARTILES):
            lines.append(f"| {name} | " + " | ".join(c[i] for c in columns) + " |")
        bands = [control_bands(groups[k], groups[k + 1], field) for k in (0, 2)]
        lines += ["", "Episodes in the controls' quartile bands (% of the origin's episodes):", "",
                  "| Control band | CAM range | Episodes CAM (%) | CPM range | Episodes CPM (%) |", "|---|---|---|---|---|"]
        for name, cam, cpm in zip(("below min",) + QUARTILES + ("above max",), *bands):
            lines.append(f"| {name} | {cam[0]} | {cam[1]} | {cpm[0]} | {cpm[1]} |")
        lines.append("")
    lines += ["## Check failed at the onset", "", "| Origin | Consistency | Predictive | Both | Fused |", "|---|---|---|---|---|"]
    for origin in ("cam", "cpm"):
        sub = [e for e in episodes if e["origin"] == origin]
        lines.append(f"| {origin} | " + " | ".join(str(sum(e["check"] == c for e in sub))
                                                   for c in ("consistency", "predictive", "both", "")) + " |")
    lines += ["", "## Episode length [vectors]", "", "| 1 | 2-6 | 7+ |", "|---|---|---|",
              f"| {sum(e['length'] == 1 for e in episodes)} | {sum(2 <= e['length'] <= 6 for e in episodes)} "
              f"| {sum(e['length'] >= 7 for e in episodes)} |",
              "", f"## Ground-truth lateral acceleration peak within +-{PEAK_WINDOW} s of the onset", "",
              f"Above {MAX_LAT_ACCEL} m/s² (tyre-road friction) the trajectory is not physically possible.", "",
              "| Band [m/s²] | Episodes (%) | Controls (%) |", "|---|---|---|"]
    bands = [(f"> {MAX_LAT_ACCEL}", lambda r: r["latAccelPeak"] > MAX_LAT_ACCEL),
             (f"{COMFORT_LAT_ACCEL} to {MAX_LAT_ACCEL}", lambda r: COMFORT_LAT_ACCEL < r["latAccelPeak"] <= MAX_LAT_ACCEL),
             (f"<= {COMFORT_LAT_ACCEL}", lambda r: r["latAccelPeak"] <= COMFORT_LAT_ACCEL),
             ("unknown", lambda r: not math.isfinite(r["latAccelPeak"]))]
    for name, test in bands:
        lines.append(f"| {name} | {share(episodes, test)} | {share(controls, test)} |")
    lines += ["", f"Physically possible episodes (no ground-truth artifact over the window the check compared): "
                  f"{sum(e['physicallyPossible'] for e in episodes)} of {len(episodes)}."]
    return "\n".join(lines) + "\n"


def events_of(episodes, vehicles, t0, subset):
    """Episodes of one vehicle starting within EVENT_GROUP s of the event's first, one row per event."""
    groups = []
    for e in sorted(episodes, key=lambda e: (e["station"], e["t"])):
        if groups and groups[-1][0]["station"] == e["station"] and e["t"] - groups[-1][0]["t"] <= EVENT_GROUP:
            groups[-1].append(e)
        else:
            groups.append([e])
    rows = []
    for eps in sorted(groups, key=lambda g: g[0]["t"]):
        first = eps[0]
        sumo = t0 + math.floor(first["t"] / STEP + 1e-6) * STEP
        peaks = [x["latAccelPeak"] for x in eps if math.isfinite(x["latAccelPeak"])]
        h, rem = divmod(sumo, 3600)
        rows.append({
            "t": round(first["t"], 2), "sumoTime": round(sumo, 1),
            "clock": f"{int(h):02d}:{int(rem // 60):02d}:{rem % 60:04.1f}",
            "station": first["station"], "sumoId": vehicles.get(first["station"], {}).get("sumoId", "?"),
            "receivers": " ".join(str(r) for r in sorted({int(x["receiver"]) for x in eps})),
            "selfTracking": sum(int(x["receiver"]) == first["station"] for x in eps),
            "episodes": len(eps), "flaggedVectors": sum(x["length"] for x in eps),
            "origins": "/".join(sorted({x["origin"] for x in eps})),
            "checks": "/".join(sorted({x["check"] or "fused" for x in eps})),
            "physicallyPossible": int(all(x["physicallyPossible"] for x in eps)),
            "latAccelPeak": round(max(peaks), 1) if peaks else "",
            "gtSpeed": round(first["gtSpeed"], 1) if math.isfinite(first["gtSpeed"]) else "",
            "view": f"GT={subset} scripts/view_in_sumo.sh {first['station']} {first['t']:.2f}",
        })
    return rows


def events_md(source, episodes, rows, criterion, t0):
    def section(title, note, part):
        lines = [f"## {title}: {len(part)} events, {sum(r['episodes'] for r in part)} episodes, "
                 f"{sum(r['flaggedVectors'] for r in part)} flagged vectors", "", note, ""]
        if not part:
            return lines + ["None.", ""]
        lines += ["| t [s] | SUMO time | Clock | Station | SUMO id | Receivers | Self-tracking | Episodes | Flagged "
                  "| Origin | Check | a_lat peak [m/s²] | v [m/s] | Open in sumo-gui |", "|---" * 14 + "|"]
        for r in part:
            lines.append(f"| {r['t']} | {r['sumoTime']} | {r['clock']} | {r['station']} | `{r['sumoId']}` "
                         f"| {r['receivers']} | {r['selfTracking']} of {r['episodes']} | {r['episodes']} "
                         f"| {r['flaggedVectors']} | {r['origins']} | {r['checks']} | {r['latAccelPeak']} "
                         f"| {r['gtSpeed']} | `{r['view']}` |")
        return lines + [""]
    lines = [f"# Events ({CRITERIA[criterion][0]}): {source}", "",
             f"{len(episodes)} episodes in {len(rows)} events (same vehicle within {EVENT_GROUP} s); "
             f"{sum(r['selfTracking'] for r in rows)} episodes are a receiver tracking itself. "
             f"SUMO time = {t0:.0f} + floor(t / {STEP}) * {STEP} s.", ""]
    lines += section("Physically possible", "No ground-truth artifact over the window the check compared: the drop "
                     "is the mechanism's to explain.", [r for r in rows if r["physicallyPossible"]])
    lines += section("Physically impossible", f"A ground-truth step no vehicle can drive in that window: a position jump "
                     f"(> {MAX_STEP_GAP} m/s off the speed), |accel| > {MAX_ACCEL} m/s² or lateral acceleration > "
                     f"{MAX_LAT_ACCEL} m/s² (scenario limitation).", [r for r in rows if not r["physicallyPossible"]])
    return "\n".join(lines)


def write(path, rows, fields):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def main():
    args = sys.argv[1:]
    criterion = "confidence"
    if "--episodes" in args:
        i = args.index("--episodes")
        criterion = args[i + 1] if i + 1 < len(args) else ""
        del args[i:i + 2]
    if len(args) != 4 or criterion not in CRITERIA:
        sys.exit(__doc__)
    records_path, receptions_path, subset, out = args[0], args[1], Path(args[2]), Path(args[3])
    out.mkdir(parents=True, exist_ok=True)
    pose = load_pose(subset / "vehicle_pose.jsonl")
    rng = random.Random(SEED)
    with open_records(records_path) as fr, open(receptions_path) as fm:
        records, receptions = csv.DictReader(fr), csv.DictReader(fm)
        if criterion == "predictive" and "predictiveFailed" not in (records.fieldnames or []):
            sys.exit(f"{records_path} has no predictiveFailed column: replay it with the current build/replay")
        episodes, reservoir = episodes_of(attributes(records, receptions, pose), criterion, rng)
    controls = []
    for origin in ("cam", "cpm"):
        n = CONTROLS_PER_EPISODE * sum(1 for e in episodes if e["origin"] == origin)
        controls += rng.sample(reservoir[origin], min(n, len(reservoir[origin])))
    add_lat_accel_peak(pose, episodes + controls)
    bad = impossible_steps(subset / "vehicle_pose.jsonl")
    for e in episodes:
        e["sumoArtifact"] = artifact(bad, e)
        e["physicallyPossible"] = 1 - e["sumoArtifact"]

    write(out / "episodes.csv", episodes, FIELDS + ["length", "duration", "sumoArtifact", "physicallyPossible"])
    write(out / "controls.csv", controls, FIELDS)
    plot(pose, episodes, controls, out / "onset_ground_truth.png")
    (out / "summary.md").write_text(summary(records_path, episodes, controls, criterion))

    vehicles = {}
    with open(subset / "vehicles.jsonl") as f:
        for line in f:
            v = json.loads(line)
            vehicles[v["station"]] = v
    t0 = sumo_clock(subset)[1]
    events = events_of(episodes, vehicles, t0, subset)
    if events:
        write(out / "events.csv", events, list(events[0].keys()))
    (out / "events.md").write_text(events_md(records_path, episodes, events, criterion, t0))


if __name__ == "__main__":
    main()
