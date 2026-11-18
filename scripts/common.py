"""Paths, constants, subset making, build, metric and process helpers shared by the experiment scripts."""
import csv
import gzip
import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GENERATOR = ROOT / "libs" / "Artery-Dataset-Generator"
# Data, outside the submodules (which a re-clone would wipe) and apart from results/: the generator's ground truth and
# SUMO logs (its container mounts this as its data/, scripts/setup_docker.sh DATA_DIR), the fault-injected subsets
# and the flattened receptions the replay reads. results/ holds only what the experiments conclude
DATA = ROOT / "data"
PYTHON = ROOT / ".venv" / "bin" / "python"  # error_diagnosis.py needs matplotlib (requirements.txt)
SUBSETS = ["Honest", "ConstPos", "ConstPosOffset", "RandomPos", "RandomPosOffset",
           "ConstSpeed", "ConstSpeedOffset", "RandomSpeed", "RandomSpeedOffset"]  # the fault-free baseline first

# The generator's SUMO clock: vehicle_pose every STEP; the state recorded at dataset time t is the latest SUMO step
# at or before it, t0 + floor(t / STEP) * STEP, t0 the SUMO time of dataset time 0 (sumo_clock)
STEP = 0.1  # SUMO step and vehicle_pose interval (generator poseInterval) [s]

EARTH_RADIUS = 6371000.0  # [m], spherical: displacements of metres


def sumo_clock(run):
    """(SUMO --begin [s], SUMO time of dataset time 0 [s], SUMO configuration relative to the generator) of a
    ground-truth run or of a subset made from one, from the run's omnetpp.ini snapshot (config/): its config's
    launcher --begin plus sink.startTime (the warm-up), following `extends` down to [General]."""
    run = Path(run)
    meta = json.loads((run / "metadata.json").read_text())
    if not (run / "config" / "omnetpp.ini").exists() and "injection" in meta:
        run = Path(meta["injection"]["source"])  # a subset: its ground-truth run
        if not (run / "metadata.json").is_file():
            run = DATA / "groundtruth" / run.name  # recorded before the data moved
    sections, name = {}, None
    for line in (run / "config" / "omnetpp.ini").read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line.startswith("["):
            name = line.strip("[]").removeprefix("Config ").strip()
            sections[name] = {}
        elif "=" in line and name is not None:
            key, value = (s.strip() for s in line.split("=", 1))
            sections[name][key] = value

    def value(key):
        config = meta["configName"]
        while config is not None:
            if key in sections.get(config, {}):
                return sections[config][key]
            config = sections.get(config, {}).get("extends", "General" if config != "General" else None)
        raise KeyError(f"{key} not in {run}/config/omnetpp.ini [{meta['configName']}]")

    begin = float(re.search(r"--begin (\d+(?:\.\d+)?)", value("*.traci.launcher.command")).group(1))
    start = float(value("*.sink.startTime").removesuffix("s"))
    return begin, begin + start, value("*.traci.launcher.sumocfg").strip('"')


def engine_constant(name):
    """A constexpr double of include/MechanismConstants.h, so the scripts follow the engine."""
    text = (ROOT / "include" / "MechanismConstants.h").read_text()
    match = re.search(rf"constexpr double {name} = ([-0-9.eE+]+);", text)
    if match is None:
        raise KeyError(f"{name} not in include/MechanismConstants.h")
    return float(match.group(1))


RETIRE_AFTER = engine_constant("DEFAULT_RETIRE_AFTER")  # track retirement [s]

# Detection as paper 1: one decision per received vector, flagged when the confidence after the vector is below
# THRESHOLD; positive when the vector's content was falsified (attackType > 0; a CPM object takes its message's)
THRESHOLD = 0.5


def records_path(path):
    """A records.csv, or its gzip (records.csv.gz, see compress) when only that exists; None when neither."""
    path = Path(path)
    gz = path.with_name(path.name + ".gz")
    return path if path.is_file() else gz if gz.is_file() else None


def open_records(path):
    """Text handle on a records.csv or on its gzip."""
    found = records_path(path)
    if found is None:
        raise FileNotFoundError(path)
    return gzip.open(found, "rt", newline="") if found.suffix == ".gz" else open(found, newline="")


# Confidence paradigm (replay --confidence) -> author column its confidence takes the lower of with the track's
PARADIGM_COLUMNS = {"object": None, "author": "authorConfidence", "author-kind": "authorKindConfidence"}


def with_paradigm(r, paradigm):
    """r with its confidence as paradigm reports it, from the three the replay records whatever its --confidence:
    the track's, or the lower of the track's and the author's (author and kind's), as the engine combines them."""
    column = PARADIGM_COLUMNS[paradigm]
    if r["outcome"] != "dropped":
        r["confidence"] = r["objectConfidence"] if column is None else min(r["objectConfidence"], r[column], key=float)
    return r


def stream(path, paradigm=None):
    """The rows of a records.csv (or its gzip), one at a time; with paradigm, its confidence (with_paradigm)."""
    with open_records(path) as f:
        for r in csv.DictReader(f):
            yield r if paradigm is None else with_paradigm(r, paradigm)


class Rows:
    """The rows of a records.csv, streamed again on every iteration: several passes without holding the file
    in memory (a test-window run has ~10^7 rows); keep filters them, paradigm as in stream."""

    def __init__(self, path, keep=None, paradigm=None):
        self.path, self.keep, self.paradigm = path, keep, paradigm

    def __iter__(self):
        return (r for r in stream(self.path, self.paradigm) if self.keep is None or self.keep(r))


def load(path, paradigm=None):
    return Rows(path, paradigm=paradigm)


def compress(path):
    """Replaces a records.csv by its gzip, read back transparently by stream/open_records (disk: ~1 GB per
    test-window replay uncompressed)."""
    run(["gzip", "-1", "-f", path])


def counts(rows, origin=None):
    """(tp, fp, tn, fn) over the vectors that reached the checks, of one origin (cam, cpm) or all."""
    tp = fp = tn = fn = 0
    for r in rows:
        if r["outcome"] == "dropped" or origin not in (None, r["origin"]):
            continue
        flagged = float(r["confidence"]) < THRESHOLD
        faulty = r["attackType"] != "0"
        tp += flagged and faulty
        fp += flagged and not faulty
        tn += not flagged and not faulty
        fn += not flagged and faulty
    return tp, fp, tn, fn


def metrics(tp, fp, tn, fn):
    """(F1, FPR, accuracy, precision, recall): rates in %, F1 in [0, 1]; None when undefined."""
    precision = 100.0 * tp / (tp + fp) if tp + fp else None
    recall = 100.0 * tp / (tp + fn) if tp + fn else None
    fpr = 100.0 * fp / (fp + tn) if fp + tn else None
    accuracy = 100.0 * (tp + tn) / (tp + fp + tn + fn) if tp + fp + tn + fn else None
    f1 = 2 * precision * recall / (precision + recall) / 100.0 if precision and recall else None
    if f1 is None and recall is not None:
        f1 = 0.0
    return f1, fpr, accuracy, precision, recall


def shift(lat, lon, east, north):
    """lat/lon moved by east, north [m]."""
    return (lat + math.degrees(north / EARTH_RADIUS),
            lon + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat)))))


def east_north(lat, lon, origin):
    """Displacement [m] of lat/lon east and north of origin (lat, lon), the inverse of shift."""
    return (math.radians(lon - origin[1]) * EARTH_RADIUS * math.cos(math.radians(origin[0])),
            math.radians(lat - origin[0]) * EARTH_RADIUS)


def option(args, name, default):
    """The value after `name` in args, both removed from args; default when absent."""
    if name in args:
        i = args.index(name)
        value = args[i + 1]
        del args[i:i + 2]
        return value
    return default


# Evaluation protocol: every parameter is calibrated or chosen on the validation window and the results are measured
# on the test window, never the other way (generator omnetpp.ini [Config veremi-val], [Config veremi-test])
TEST_CONFIG = "veremi-test"        # 08:00-08:15, evaluation
VALIDATION_CONFIG = "veremi-val"   # 15:30-15:45, calibration and parameter choice


def default_gt():
    """The generator's ground truth of config CONFIG (default TEST_CONFIG), run RUN (default 0)."""
    config, seed = os.environ.get("CONFIG", TEST_CONFIG), os.environ.get("RUN", "0")
    return DATA / "groundtruth" / f"{config}-run{seed}"


def validation_gt():
    """The generator's ground truth of VALIDATION_CONFIG, run RUN (default 0)."""
    return DATA / "groundtruth" / f"{VALIDATION_CONFIG}-run{os.environ.get('RUN', '0')}"


def ground_truth_dir(args):
    """The ground truth dir given as the only argument left in args, else default_gt()."""
    return Path(args[0]).resolve() if args else default_gt()


def calibration_dir(args):
    """The ground truth dir to calibrate on: the only argument left in args, else validation_gt(); exits on a
    test-window run (or a subset of one), which is never used to calibrate."""
    gt = Path(args[0]).resolve() if args else validation_gt()
    meta = gt / "metadata.json"
    config = json.loads(meta.read_text()).get("configName") if meta.is_file() else None
    if config == TEST_CONFIG:
        raise SystemExit(f"{gt} is the test window ({TEST_CONFIG}): calibrate on the validation window "
                         f"({VALIDATION_CONFIG}) only")
    return gt


def run(cmd, **kw):
    subprocess.run([str(c) for c in cmd], check=True, **kw)


JOBS = int(os.environ.get("JOBS", "3"))  # parallel workers of the per-subset work (scripts/veremi_eval.py)
MIN_FREE_MB = 800  # a worker starts its next unit only with this much memory available


def wait_for_memory():
    """Blocks while MemAvailable is below MIN_FREE_MB, so parallel workers never push the machine into swap."""
    import time
    while True:
        with open("/proc/meminfo") as f:
            free = next(int(line.split()[1]) // 1024 for line in f if line.startswith("MemAvailable:"))
        if free >= MIN_FREE_MB:
            return
        time.sleep(10)


def low_priority():
    """Lowest CPU and I/O priority for this process and its children."""
    os.nice(19)
    subprocess.run(["ionice", "-c3", "-p", str(os.getpid())], check=False)


def setup():
    """Lowest priority, then replay built in BUILD (default build/); returns its path."""
    low_priority()
    return build(Path(os.environ.get("BUILD", ROOT / "build")).resolve())


def build(build_dir, target="replay"):
    """Configures and builds target (default replay) in build_dir, two jobs at most; returns its path."""
    run(["cmake", "-S", ROOT, "-B", build_dir], stdout=subprocess.DEVNULL)
    run(["cmake", "--build", build_dir, "--target", target, "-j2"], stdout=subprocess.DEVNULL)
    return build_dir / target


def receptions_dir(experiment, gt):
    """Where an experiment's flattened receptions (the replay's input) and their faults.toml go."""
    return DATA / "receptions" / f"{experiment}-{Path(gt).name}"


def receptions(gt, subset, out, sensor_errors=False):
    """The subset of ground truth gt (faults injected, receiver view), flattened to <out>/<subset>.csv; with
    sensor_errors, the generator's VeReMi Extension sensor errors ([sensor_errors] of its faults.toml) are kept.
    Returns the generator's subset dir."""
    target = DATA / "subsets" / f"{gt.name}-{subset}{'-sensor-errors' if sensor_errors else ''}"
    shutil.rmtree(target, ignore_errors=True)
    text = (GENERATOR / "config" / "faults.toml").read_text()
    text = re.sub(r'^input = .*$', f'input = "{gt}"', text, flags=re.M)
    text = re.sub(r'^output = .*$', f'output = "{target}"', text, flags=re.M)
    types = "[]" if subset == "Honest" else f'["{subset}"]'
    text = re.sub(r'^types = \[.*?\]', f'types = {types}', text, flags=re.M | re.S)
    if not sensor_errors:
        text = re.sub(r'\[sensor_errors\].*?(?=\n\[)', '', text, flags=re.S)
    faults = out / f"{subset}.faults.toml"
    faults.write_text(text)
    quiet = {"stdout": subprocess.DEVNULL}
    run(["python3", GENERATOR / "scripts" / "inject_faults.py", faults], **quiet)
    run(["python3", GENERATOR / "scripts" / "receiver_view.py", target], **quiet)
    run(["python3", ROOT / "scripts" / "flatten_dataset.py", target, out / f"{subset}.csv"], **quiet)
    # read only to flatten (several GB on the test window); vehicles, metadata and the pose links stay
    shutil.rmtree(target / "receivers")
    for name in ("cam_sent.jsonl", "cpm_sent.jsonl"):
        (target / name).unlink()
    return target


if __name__ == "__main__":
    # shell use (view_in_sumo.sh): python3 scripts/common.py sumo-clock <run or subset dir> -> begin t0 sumocfg
    import sys
    if sys.argv[1:2] != ["sumo-clock"] or len(sys.argv) != 3:
        sys.exit("usage: common.py sumo-clock <run or subset dir>")
    print(*sumo_clock(sys.argv[2]))
