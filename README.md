# Object Confidence

Per-object confidence for CAM and CPM motion vectors: reports are associated to fused tracks
(UKF, CTRA) and each track is scored by how consistently they agree with it.

## How to run

```sh
git submodule update --init
cmake -S . -B build && cmake --build build
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

scripts/run_preliminary.sh                  # 8 VeReMi Extension fault subsets -> results/preliminary/
.venv/bin/python scripts/evaluate.py        # results/preliminary/table.md, confidence_over_time.png
```

The ground truth comes from `libs/Artery-Dataset-Generator` (`data/groundtruth/lust-run0`);
generating it needs that submodule's own submodules and its README.
