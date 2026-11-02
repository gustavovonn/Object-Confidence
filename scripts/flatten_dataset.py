#!/usr/bin/env python3
"""Object-Confidence filter over the generator's receiver view: one CSV row per received motion vector, no conversion.

Usage (single run):
    flatten_dataset.py <run dir with receivers/> <out.csv>

Usage (sweep mode):
    flatten_dataset.py --sweep [input_dir] [--out-dir output_dir]

Defaults for sweep mode:
    input_dir:  libs/Artery-Dataset-Generator/data/subsets/
    output_dir: results/flattened/
"""
import argparse
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


def flatten_run(run_dir: Path, out_csv: Path) -> bool:
    """Flattens a single run directory containing a receivers/ subdirectory into out_csv."""
    view = run_dir / "receivers"
    if not view.is_dir():
        print(
            f"Error: {view} missing. Run libs/Artery-Dataset-Generator/scripts/receiver_view.py {run_dir} first.",
            file=sys.stderr,
        )
        return False

    out_csv.parent.mkdir(parents=True, exist_ok=True)

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        # Process one receiver file at a time to optimize memory usage
        for path in sorted(view.glob("*.jsonl"), key=lambda p: int(p.stem)):
            with open(path) as r:
                receptions = sorted(map(json.loads, r), key=lambda m: (m["rcvTime"], m["messageID"]))
            for m in receptions:
                head = [m["receiver"], m["rcvTime"], m["messageID"]]
                if m["msg"] == "cam":
                    has_velocity = m["speed"] is not None and m["heading"] is not None
                    w.writerow([
                        "cam", *head, m["sender"], m["refSimTime"], m["lat"], m["lon"],
                        int(has_velocity), num(m["speed"]), num(m["heading"]), num(m["yawRate"]),
                        num(m["vehicleLength"]), -1, "nan", "nan", "nan", m["sender"],
                        m.get("attackType", 0)
                    ])
                elif m["lat"] is not None:
                    for o in m["objects"]:
                        has_velocity = o["speed"] is not None and o["direction"] is not None
                        station = o["objectStation"] if o["objectStation"] is not None else -1
                        w.writerow([
                            "cpm", *head, m["sender"], o["measTime"], m["lat"], m["lon"],
                            int(has_velocity), num(o["speed"]), "nan", "nan", "nan",
                            o["objectId"] if o["objectId"] is not None else -1,
                            o["x"], o["y"], num(o["direction"]), station, m.get("attackType", 0)
                        ])
    return True


def sweep_directories(in_dir: Path, out_dir: Path):
    """Finds immediate subdirectories with receivers/ inside in_dir and flattens each to out_dir/<subset_name>.csv."""
    if not in_dir.exists() or not in_dir.is_dir():
        sys.exit(f"Error: input directory '{in_dir}' does not exist or is not a directory.")

    # Find immediate child subdirectories containing a receivers/ directory
    subdirs = [p for p in sorted(in_dir.iterdir()) if p.is_dir() and (p / "receivers").is_dir()]

    if not subdirs:
        print(f"No subdirectories with 'receivers/' found under {in_dir}", file=sys.stderr)
        return

    print(f"Sweeping {len(subdirs)} subset directories from '{in_dir}' into '{out_dir}'...")
    count = 0
    for run_dir in subdirs:
        out_csv = out_dir / f"{run_dir.name}.csv"
        print(f"  [{count + 1}/{len(subdirs)}] Flattening {run_dir.name} -> {out_csv}")
        if flatten_run(run_dir, out_csv):
            count += 1

    print(f"Successfully processed {count}/{len(subdirs)} subsets.")


def main():
    parser = argparse.ArgumentParser(
        description="Flatten receiver-view datasets into CSV files for Object-Confidence replay."
    )
    parser.add_argument(
        "run",
        nargs="?",
        type=Path,
        help="Single run directory containing receivers/ (used when --sweep is omitted)",
    )
    parser.add_argument(
        "out",
        nargs="?",
        type=Path,
        help="Output CSV file path (used when --sweep is omitted)",
    )
    parser.add_argument(
        "--sweep",
        nargs="?",
        const=Path("libs/Artery-Dataset-Generator/data/subsets/"),
        type=Path,
        help="Flatten all child subdirectories in input dir (default: libs/Artery-Dataset-Generator/data/subsets/)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("results/flattened/"),
        help="Output directory for sweep mode (default: results/flattened/)",
    )

    args = parser.parse_args()

    if args.sweep is not None:
        sweep_directories(args.sweep, args.out_dir)
    elif args.run and args.out:
        if not flatten_run(args.run, args.out):
            sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
