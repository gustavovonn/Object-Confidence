#!/usr/bin/env python3
"""Reproduces the paper's experiments, one step after the other, at the lowest CPU and I/O priority.

Usage: [CONFIG=veremi-test] [RUN=0] [BUILD=build] [KEEP_RECEPTIONS=1] ./reproduce_paper.py [ground truth dir]
Default ground truth: the generator's config CONFIG, run RUN, libs/Artery-Dataset-Generator/data/groundtruth/
<CONFIG>-run<RUN>; when missing, it is simulated first with the generator's scripts (docker setup, build, run).
A given ground truth dir must exist.

Before the steps: the submodules are checked, .venv is made from requirements.txt when missing (the scoring
scripts need matplotlib), and replay is built.

Steps:
  1 scripts/kalman_eval.py       Kalman x no Kalman on Honest, per level of white position noise
  2 scripts/error_diagnosis.py   the rejections of the cells in CELLS, written to <cell>/error_diagnosis
  3 scripts/association_eval.py  association quality of the cells in CELLS, written to
                                 s<sigma>/<variant>/association.md
  4 scripts/score_paradigm_ablation.py  detection (table.md), fate, association and ROC on Honest and the 8 VeReMi
                                 subsets with the VeReMi sensor errors (scripts/veremi_eval.py), per confidence
                                 paradigm (object, min(object, author), min(object, author x kind)) and variant of
                                 DETECTION; score-paradigms-<variant>.md. Each subset made and flattened once and
                                 replayed per paradigm: the object paradigm is veremi_eval.py's own run
Evaluation on the test window only (default CONFIG veremi-test). Every parameter is calibrated or chosen before,
on the validation window (veremi-val), never on the test window, which those scripts refuse:
  scripts/flatten_truth.py + build/calibrate_ukf   UKF Q and P0 (include/MechanismConstants.h ConstantsUkf)
  scripts/calib_maxposdev.py                       residual quantiles behind DEFAULT_MAX_POS_DEV
  scripts/sensitivity.py                           frozen bound, retirement time and maxPosDev sweeps
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
from common import GENERATOR, PYTHON, ROOT, build, default_gt, low_priority, run  # noqa: E402

DETECTION = ["kalman"]  # variants of step 4
CELLS = [("no-kalman", 0.0), ("kalman", 0.3)]  # (variant, sigma [m]) cells of kalman_eval.py, steps 2 and 3


def prepare(given):
    """Submodules, virtual environment, replay build and ground truth; returns the ground truth dir."""
    if not (GENERATOR / "scripts" / "inject_faults.py").is_file() or not (ROOT / "libs" / "ukf" / "ukf.cpp").is_file():
        sys.exit("submodules missing: git submodule update --init (the generator also needs its own, see its README)")
    if not PYTHON.is_file():
        print("making .venv from requirements.txt", flush=True)
        run([sys.executable, "-m", "venv", ROOT / ".venv"])
        run([PYTHON, "-m", "pip", "install", "-r", ROOT / "requirements.txt"])
    build(Path(os.environ.get("BUILD", ROOT / "build")).resolve())

    if given is not None:
        gt = Path(given).resolve()
        if not gt.is_dir():
            sys.exit(f"missing ground truth {gt}")
        return gt
    gt = default_gt()
    config, seed = os.environ.get("CONFIG", "veremi-test"), os.environ.get("RUN", "0")
    if not gt.is_dir():
        print(f"missing {gt}: simulating config {config}, run {seed}", flush=True)
        run([GENERATOR / "scripts" / "setup_docker.sh"])
        run([GENERATOR / "scripts" / "build.sh"])
        run([GENERATOR / "scripts" / "run.sh", "-c", config, "-r", seed])
        if not gt.is_dir():
            sys.exit(f"the generator did not write {gt}")
    return gt


def main():
    if len(sys.argv) > 2:
        sys.exit(__doc__)
    low_priority()
    gt = prepare(sys.argv[1] if len(sys.argv) == 2 else None)
    keep = os.environ.get("KEEP_RECEPTIONS") == "1"
    sigmas = sorted({sigma for _, sigma in CELLS} | set(kalman_eval.SIGMAS))

    print("1/4 kalman_eval", flush=True)
    run([sys.executable, ROOT / "scripts" / "kalman_eval.py", gt, "--sigmas", ",".join(f"{s:g}" for s in sigmas)],
        env={**os.environ, "KEEP_RECEPTIONS": "1"})

    subset = GENERATOR / "data" / "subsets" / f"{gt.name}-Honest"  # made by kalman_eval.py
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
        rec = kalman_eval.out_dir(gt) / "receptions"
        for path in [*rec.glob("Honest*.csv")]:
            path.unlink()

    for variant in DETECTION:
        print(f"4/4 score_paradigm_ablation {variant}", flush=True)
        run([sys.executable, ROOT / "scripts" / "score_paradigm_ablation.py", gt, "--variant", variant])


if __name__ == "__main__":
    main()
