#!/usr/bin/env python3
"""Reproduces the paper's experiments, one step after the other, at the lowest CPU and I/O priority: the
calibration on the validation window, then the evaluation on the test window.

Usage: [CONFIG=veremi-test] [RUN=0] [BUILD=build] [SKIP_VALIDATION=1] [KEEP_RECEPTIONS=1] ./reproduce_paper.py
       [ground truth dir]
Default ground truth: the generator's config CONFIG, run RUN, data/groundtruth/<CONFIG>-run<RUN>; when missing,
it is simulated first with the generator's scripts (docker setup, build, run). A given ground truth dir must exist.
The validation window (veremi-val, run RUN) is simulated the same way when missing.

Before the steps: the submodules are checked, .venv is made from requirements.txt when missing (the scoring
scripts need matplotlib), and replay and calibrate_ukf are built.

Validation (veremi-val; skipped with SKIP_VALIDATION=1). Every parameter of the engine is calibrated or chosen
here, never on the test window, which these scripts refuse:
  V1 scripts/flatten_truth.py + build/calibrate_ukf   UKF Q and P0 (include/MechanismConstants.h ConstantsUkf):
                                 results/calibration/<val>/ukf_calibration.md
  V2 scripts/calib_maxposdev.py  residual quantiles behind DEFAULT_MAX_POS_DEV: results/calibration/<val>/maxposdev.md
  V3 scripts/sensitivity.py      frozen bound, retirement time and maxPosDev sweeps:
                                 results/sensitivity-<val>/sensitivity.md
  The reports do not change the constants: the values chosen from them are fixed in include/MechanismConstants.h
  with their reason. Compare the reports with them before reading the test results.
Test (default CONFIG veremi-test):
  1 scripts/kalman_eval.py       Kalman x no Kalman on Honest, per level of white position noise
  2 scripts/error_diagnosis.py   the rejections of the cells in CELLS, written to <cell>/error_diagnosis
  3 scripts/association_eval.py  association quality of the cells in CELLS, written to
                                 s<sigma>/<variant>/association.md
  4 scripts/score_paradigm_ablation.py  detection (table.md), fate, association and ROC on Honest and the 8 VeReMi
                                 subsets with the VeReMi sensor errors (scripts/veremi_eval.py), per confidence
                                 paradigm (object, min(object, author), min(object, author x kind)) and variant of
                                 DETECTION; score-paradigms-<variant>.md. Each subset made and flattened once and
                                 replayed per paradigm: the object paradigm is veremi_eval.py's own run
Standard condition, everything but step 1's noise levels: the VeReMi Extension sensor errors, R from their jitter
(scripts/veremi_eval.py STANDARD).
Outputs: as each script documents, under results/kalman-eval-<ground truth name>/ (1-3),
and results/veremi-eval-<ground truth name>/ (4).
The noised receptions are kept for step 2 and deleted after it, unless KEEP_RECEPTIONS=1.
"""
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "scripts"))  # the experiment scripts

import kalman_eval  # noqa: E402
from common import (DATA, GENERATOR, PYTHON, ROOT, VALIDATION_CONFIG, build, default_gt, low_priority,  # noqa: E402
                    receptions_dir, run, validation_gt)

DETECTION = ["kalman"]  # variants of step 4
CELLS = [("no-kalman", 0.0), ("kalman", 0.3)]  # (variant, sigma [m]) cells of kalman_eval.py, steps 2 and 3
SEED = os.environ.get("RUN", "0")


def simulate(gt, config):
    """gt, simulated first with the generator (config, run SEED) when missing."""
    if gt.is_dir():
        return gt
    print(f"missing {gt}: simulating config {config}, run {SEED}", flush=True)
    DATA.mkdir(parents=True, exist_ok=True)
    run([GENERATOR / "scripts" / "setup_docker.sh"], env={**os.environ, "DATA_DIR": str(DATA)})
    run([GENERATOR / "scripts" / "build.sh"])
    run([GENERATOR / "scripts" / "run.sh", "-c", config, "-r", SEED])
    if not gt.is_dir():
        sys.exit(f"the generator did not write {gt}")
    return gt


def prepare(given):
    """Submodules, virtual environment, builds and the test ground truth; returns the build dir and that dir."""
    if not (GENERATOR / "scripts" / "inject_faults.py").is_file() or not (ROOT / "libs" / "ukf" / "ukf.cpp").is_file():
        sys.exit("submodules missing: git submodule update --init (the generator also needs its own, see its README)")
    if not PYTHON.is_file():
        print("making .venv from requirements.txt", flush=True)
        run([sys.executable, "-m", "venv", ROOT / ".venv"])
        run([PYTHON, "-m", "pip", "install", "-r", ROOT / "requirements.txt"])
    build_dir = Path(os.environ.get("BUILD", ROOT / "build")).resolve()
    build(build_dir)
    build(build_dir, "calibrate_ukf")

    if given is not None:
        gt = Path(given).resolve()
        if not gt.is_dir():
            sys.exit(f"missing ground truth {gt}")
        return build_dir, gt
    return build_dir, simulate(default_gt(), os.environ.get("CONFIG", "veremi-test"))


def validate(build_dir):
    """Steps V1-V3 on the validation window."""
    gt = simulate(validation_gt(), VALIDATION_CONFIG)
    truth = DATA / "calibration" / gt.name
    report = ROOT / "results" / "calibration" / gt.name / "ukf_calibration.md"
    report.parent.mkdir(parents=True, exist_ok=True)

    print("V1/3 UKF calibration (flatten_truth, calibrate_ukf)", flush=True)
    run([PYTHON, ROOT / "scripts" / "flatten_truth.py", gt, truth], stdout=subprocess.DEVNULL)
    run([build_dir / "calibrate_ukf", truth, report], stdout=subprocess.DEVNULL)
    print("V2/3 calib_maxposdev", flush=True)
    run([PYTHON, ROOT / "scripts" / "calib_maxposdev.py", gt], stdout=subprocess.DEVNULL)
    print("V3/3 sensitivity", flush=True)
    run([PYTHON, ROOT / "scripts" / "sensitivity.py", gt], stdout=subprocess.DEVNULL)
    print(f"validation reports: {report.parent}, {ROOT / 'results' / f'sensitivity-{gt.name}'}; "
          "compare them with include/MechanismConstants.h", flush=True)


def main():
    if len(sys.argv) > 2:
        sys.exit(__doc__)
    low_priority()
    build_dir, gt = prepare(sys.argv[1] if len(sys.argv) == 2 else None)
    if os.environ.get("SKIP_VALIDATION") != "1":
        validate(build_dir)
    keep = os.environ.get("KEEP_RECEPTIONS") == "1"
    sigmas = sorted({sigma for _, sigma in CELLS} | set(kalman_eval.SIGMAS))

    print("1/4 kalman_eval", flush=True)
    run([PYTHON, ROOT / "scripts" / "kalman_eval.py", gt, "--sigmas", ",".join(f"{s:g}" for s in sigmas)],
        env={**os.environ, "KEEP_RECEPTIONS": "1"})

    subset = DATA / "subsets" / f"{gt.name}-Honest"  # made by kalman_eval.py
    for variant, sigma in CELLS:
        print(f"2/4 error_diagnosis {variant} s={sigma:g}", flush=True)
        cell = kalman_eval.cell_dir(gt, variant, sigma)
        run([PYTHON, ROOT / "scripts" / "error_diagnosis.py", cell / "records.csv", kalman_eval.noised(gt, sigma),
             subset, cell / "error_diagnosis"], stdout=subprocess.DEVNULL)

    for variant, sigma in CELLS:
        print(f"3/4 association_eval {variant} s={sigma:g}", flush=True)
        run([PYTHON, ROOT / "scripts" / "association_eval.py", kalman_eval.cell_dir(gt, variant, sigma).parent],
            stdout=subprocess.DEVNULL)

    if not keep:
        rec = receptions_dir("kalman-eval", gt)
        for path in [*rec.glob("Honest*.csv")]:
            path.unlink()

    for variant in DETECTION:
        print(f"4/4 score_paradigm_ablation {variant}", flush=True)
        run([PYTHON, ROOT / "scripts" / "score_paradigm_ablation.py", gt, "--variant", variant])


if __name__ == "__main__":
    main()
