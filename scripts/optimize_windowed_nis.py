#!/usr/bin/env python3
"""
optimize_windowed_nis.py — Grid-search optimizer for WindowedNISScorer parameters.

Sweeps innovationSigma (σ), nisWindowSize (W), and nisThreshold (τ) by running the
replay binary for each combination and evaluating detection metrics.  Results are
ranked by F1 (primary) and AUC-ROC (secondary).

Usage
-----
  # Single dataset
  python3 scripts/optimize_windowed_nis.py results/flattened/subset.csv

  # Sweep every CSV in a directory (macro-averages across subsets)
  python3 scripts/optimize_windowed_nis.py --sweep results/flattened/

  # Narrow the grid or tune the objective
  python3 scripts/optimize_windowed_nis.py subset.csv \\
      --sigma-values 1.5 3.0 5.0 \\
      --window-values 5 10 20 \\
      --tau-values 3.84 5.99 7.82 \\
      --metric auc

Outputs (in --out-dir, default results/windowed-nis-opt/)
---------
  grid_results.csv       every (σ, W, τ) combo × metric, sorted by objective
  best_params.json       top-1 config ready to paste into EngineParams
  heatmap_f1.png         σ vs W heat-map coloured by F1 (--plot, τ fixed to best)
  sweep/                 per-subset sub-directories with their own grid_results.csv
  sweep/summary.csv      macro-average metrics across all subsets

Notes
-----
* --nis-threshold and --nis-min-samples require the replay patch described in
  the implementation plan (tools/replay.cc + include/EngineParams.h +
  include/ScorerFactory.h).  If the patched binary is not available, τ is fixed
  to its theoretical default (5.991) and --tau-values / --min-samples-values are
  silently clamped to a single entry.
"""

import argparse
import itertools
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.metrics import (
    f1_score,
    precision_score,
    recall_score,
    roc_auc_score,
    accuracy_score,
    precision_recall_curve,
)

DEFAULT_SIGMA_VALUES       = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
DEFAULT_WINDOW_VALUES      = [5, 7, 10, 15, 20, 30]
DEFAULT_TAU_VALUES         = [5.991]
DEFAULT_MIN_SAMPLES_VALUES = [3]

DEFAULT_CLASSIFICATION_THRESHOLD = 0.5
DEFAULT_METRIC   = "f1"   # primary ranking metric
VALID_METRICS    = {"f1", "auc", "precision", "recall", "accuracy"}


def find_replay_binary(custom_path=None):
    """Locates the replay executable (mirrors compare_scorers.find_replay_binary)."""
    if custom_path:
        p = Path(custom_path)
        if p.exists() and os.access(p, os.X_OK):
            return str(p)
        raise FileNotFoundError(
            f"Specified replay binary not found or not executable: {custom_path}"
        )
    candidate = Path(__file__).parent.parent / "build" / "replay"
    if candidate.exists() and os.access(candidate, os.X_OK):
        return str(candidate)
    raise FileNotFoundError(
        "Could not locate 'replay' binary.  Build first (cmake -S . -B build && "
        "cmake --build build) or pass --replay-bin."
    )


def evaluate_records(records_csv, threshold=0.5):
    """
    Evaluates a records CSV against ground truth.

    Ground truth : attackType != 0  → attack  (positive class, label 1)
    Prediction   : confidence <= threshold → attack (label 1)
    Score column : 1 - confidence  (higher = more likely attack, for AUC)
    """
    df = pd.read_csv(records_csv)
    if df.empty:
        raise ValueError(f"Records CSV '{records_csv}' is empty.")

    y_true  = (df["attackType"] != 0).astype(int)
    y_pred  = (df["confidence"] <= threshold).astype(int)
    y_score = 1.0 - df["confidence"]

    acc   = accuracy_score(y_true, y_pred)
    prec  = precision_score(y_true, y_pred, zero_division=0)
    rec   = recall_score(y_true, y_pred, zero_division=0)
    f1    = f1_score(y_true, y_pred, zero_division=0)
    auc   = roc_auc_score(y_true, y_score) if len(np.unique(y_true)) > 1 else float("nan")

    pr_curve, rc_curve = None, None
    if len(np.unique(y_true)) > 1:
        pr_curve, rc_curve, _ = precision_recall_curve(y_true, y_score)

    return {
        "accuracy": acc, "precision": prec, "recall": rec,
        "f1": f1, "auc": auc,
        "pr_curve": pr_curve, "rc_curve": rc_curve,
    }


def _probe_flag(replay_bin, flag):
    """Returns True when replay accepts the flag without the 'unknown option' exit."""
    # Pass the flag with a dummy value; replay exits 1 with usage() for unknown flags.
    result = subprocess.run(
        [replay_bin, "/dev/null", "/dev/null", flag, "0"],
        capture_output=True, text=True,
    )
    # Unknown flag → usage printed to stderr and non-zero exit
    return flag in result.stderr or result.returncode == 0


def probe_extended_flags(replay_bin):
    """
    Checks whether the patched replay binary exposes --nis-threshold and
    --nis-min-samples.  Returns (has_threshold, has_min_samples).
    """
    has_threshold   = _probe_flag(replay_bin, "--nis-threshold")
    has_min_samples = _probe_flag(replay_bin, "--nis-min-samples")
    return has_threshold, has_min_samples


def run_one_config(
    replay_bin,
    receptions_csv,
    records_out,
    sigma,
    window,
    tau,
    min_samples,
    max_msgs=None,
    has_threshold_flag=False,
    has_min_samples_flag=False,
):
    """Runs replay for a single WindowedNIS parameter combination."""
    cmd = [
        replay_bin,
        str(receptions_csv),
        str(records_out),
        "--scorer",          "windowed-nis",
        "--innovation-sigma", str(sigma),
        "--nis-window",       str(window),
    ]
    if has_threshold_flag:
        cmd += ["--nis-threshold", str(tau)]
    if has_min_samples_flag:
        cmd += ["--nis-min-samples", str(min_samples)]
    if max_msgs and max_msgs > 0:
        cmd += ["--max-msgs", str(max_msgs)]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"replay failed (sigma={sigma} W={window} tau={tau}):\n{result.stderr}"
        )


def build_grid(sigma_values, window_values, tau_values, min_samples_values):
    return list(itertools.product(sigma_values, window_values, tau_values, min_samples_values))


def grid_search_dataset(
    replay_bin,
    receptions_csv,
    grid,
    args,
    has_threshold_flag,
    has_min_samples_flag,
    verbose=True,
):
    """
    Runs the full grid over one receptions CSV.
    Returns a DataFrame with one row per (sigma, window, tau, min_samples).
    """
    rows = []
    n = len(grid)

    with tempfile.TemporaryDirectory() as tmpdir:
        for i, (sigma, window, tau, min_samp) in enumerate(grid, 1):
            if verbose:
                print(
                    f"  [{i:>{len(str(n))}}/{n}]  σ={sigma:<5}  W={window:<4}"
                    f"  τ={tau:<6}  min_samples={min_samp}",
                    end="\r",
                    flush=True,
                )
            tmp_csv = Path(tmpdir) / f"rec_{i}.csv"

            try:
                run_one_config(
                    replay_bin,
                    receptions_csv,
                    tmp_csv,
                    sigma=sigma,
                    window=window,
                    tau=tau,
                    min_samples=min_samp,
                    max_msgs=args.max_msgs,
                    has_threshold_flag=has_threshold_flag,
                    has_min_samples_flag=has_min_samples_flag,
                )
                m = evaluate_records(tmp_csv, threshold=args.threshold)
            except Exception as exc:
                print(f"\n  WARNING: config σ={sigma} W={window} τ={tau} failed: {exc}",
                      file=sys.stderr)
                m = {"accuracy": float("nan"), "precision": float("nan"),
                     "recall": float("nan"), "f1": float("nan"), "auc": float("nan"),
                     "pr_curve": None, "rc_curve": None}

            rows.append({
                "sigma":       sigma,
                "window":      window,
                "tau":         tau,
                "min_samples": min_samp,
                "f1":          m["f1"],
                "auc":         m["auc"],
                "precision":   m["precision"],
                "recall":      m["recall"],
                "accuracy":    m["accuracy"],
            })

    if verbose:
        print()  # newline after the carriage-return progress line

    df = pd.DataFrame(rows)
    return df


def rank_results(df, primary_metric):
    """Sorts by primary_metric desc, then AUC desc as tie-breaker."""
    secondary = "auc" if primary_metric != "auc" else "f1"
    return df.sort_values([primary_metric, secondary], ascending=False).reset_index(drop=True)


def print_top(df, n, primary_metric, dataset_label=""):
    header = f"  {'rank':<5} {'sigma':<7} {'W':<5} {'tau':<7} {'min_s':<6} " \
             f"{'f1':<7} {'auc':<7} {'prec':<7} {'rec':<7} {'acc':<7}"
    label = f"  Top-{n} configs" + (f" — {dataset_label}" if dataset_label else "")
    print(label)
    print(header)
    print("  " + "─" * (len(header) - 2))
    for rank, row in df.head(n).iterrows():
        auc_str = f"{row['auc']:.4f}" if not np.isnan(row["auc"]) else "  N/A "
        marker = "◀ best" if rank == 0 else ""
        print(
            f"  {rank+1:<5} {row['sigma']:<7} {int(row['window']):<5} {row['tau']:<7.3f} "
            f"{int(row['min_samples']):<6} "
            f"{row['f1']:<7.4f} {auc_str:<7} {row['precision']:<7.4f} "
            f"{row['recall']:<7.4f} {row['accuracy']:<7.4f}  {marker}"
        )
    print()


def save_results(df, out_dir, primary_metric):
    """Writes grid_results.csv and best_params.json to out_dir."""
    out_dir.mkdir(parents=True, exist_ok=True)
    ranked = rank_results(df, primary_metric)
    ranked.to_csv(out_dir / "grid_results.csv", index=False)

    best = ranked.iloc[0]
    best_params = {
        "innovationSigma": best["sigma"],
        "nisWindowSize":   int(best["window"]),
        "nisThreshold":    best["tau"],
        "nisMinSamples":   int(best["min_samples"]),
        f"_{primary_metric}_achieved": round(best[primary_metric], 4),
    }
    with open(out_dir / "best_params.json", "w") as f:
        json.dump(best_params, f, indent=2)

    return ranked


def plot_heatmap(df, out_dir, primary_metric, tau_fixed=None):
    """
    Saves a sigma × window heat-map of the primary metric to out_dir/heatmap_<metric>.png.
    When multiple τ values exist, uses tau_fixed (the best τ) to slice the data.
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors
    except ImportError:
        print("Warning: matplotlib not installed — skipping heat-map.", file=sys.stderr)
        return

    subset = df if tau_fixed is None else df[df["tau"] == tau_fixed]
    # Average over min_samples if it was swept
    pivot = (
        subset.groupby(["sigma", "window"])[primary_metric]
        .mean()
        .unstack("window")
    )

    fig, ax = plt.subplots(figsize=(9, 6))
    cmap = "YlGnBu" if primary_metric not in ("recall",) else "PuRd"
    im = ax.imshow(pivot.values, aspect="auto", cmap=cmap,
                   vmin=np.nanmin(pivot.values), vmax=np.nanmax(pivot.values))

    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([str(int(c)) for c in pivot.columns])
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([str(v) for v in pivot.index])
    ax.set_xlabel("Window size W")
    ax.set_ylabel("Innovation σ [m]")
    tau_label = f" (τ = {tau_fixed})" if tau_fixed is not None else ""
    ax.set_title(f"WindowedNIS — {primary_metric.upper()} heat-map{tau_label}")

    for r in range(pivot.shape[0]):
        for c in range(pivot.shape[1]):
            val = pivot.values[r, c]
            if not np.isnan(val):
                ax.text(c, r, f"{val:.3f}", ha="center", va="center",
                        fontsize=7,
                        color="white" if val > (np.nanmax(pivot.values) * 0.75) else "black")

    fig.colorbar(im, ax=ax, label=primary_metric.upper())
    fig.tight_layout()
    out_path = out_dir / f"heatmap_{primary_metric}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Heat-map saved → {out_path}")


def sweep(replay_bin, in_dir, out_base_dir, grid, args, has_threshold_flag, has_min_samples_flag):
    csv_files = sorted(in_dir.glob("*.csv"))
    if not csv_files:
        sys.exit(f"No CSV files found in '{in_dir}'.")

    print(f"Sweeping {len(csv_files)} datasets from '{in_dir}'…\n")
    all_best = []

    for idx, csv_file in enumerate(csv_files, 1):
        print(f"[{idx}/{len(csv_files)}] {csv_file.name}")
        subset_dir = out_base_dir / "sweep" / csv_file.stem
        df = grid_search_dataset(
            replay_bin, csv_file, grid, args,
            has_threshold_flag, has_min_samples_flag,
            verbose=True,
        )
        ranked = save_results(df, subset_dir, args.metric)
        print_top(ranked, n=3, primary_metric=args.metric, dataset_label=csv_file.stem)
        best = ranked.iloc[0].to_dict()
        best["dataset"] = csv_file.stem
        all_best.append(best)
        if args.plot:
            best_tau = ranked.iloc[0]["tau"]
            plot_heatmap(ranked, subset_dir, args.metric, tau_fixed=best_tau)

    # Macro-average summary across all datasets
    df_all = pd.DataFrame(all_best)
    summary_rows = []
    for (sigma, window, tau, min_samp), grp in df_all.groupby(
            ["sigma", "window", "tau", "min_samples"]):
        summary_rows.append({
            "sigma": sigma, "window": window, "tau": tau, "min_samples": min_samp,
            "f1":        grp["f1"].mean(),
            "auc":       grp["auc"].mean(),
            "precision": grp["precision"].mean(),
            "recall":    grp["recall"].mean(),
            "accuracy":  grp["accuracy"].mean(),
            "n_datasets": len(grp),
        })
    df_summary = pd.DataFrame(summary_rows)
    ranked_summary = rank_results(df_summary, args.metric)
    (out_base_dir / "sweep").mkdir(parents=True, exist_ok=True)
    ranked_summary.to_csv(out_base_dir / "sweep" / "summary.csv", index=False)

    print("=" * 60)
    print(f"Sweep complete — macro-average over {len(csv_files)} datasets")
    print_top(ranked_summary, n=args.top_n, primary_metric=args.metric,
              dataset_label="macro-average")

    best_overall = ranked_summary.iloc[0]
    best_params = {
        "innovationSigma": best_overall["sigma"],
        "nisWindowSize":   int(best_overall["window"]),
        "nisThreshold":    best_overall["tau"],
        "nisMinSamples":   int(best_overall["min_samples"]),
        f"_{args.metric}_macro_avg": round(best_overall[args.metric], 4),
        "n_datasets": int(best_overall["n_datasets"]),
    }
    with open(out_base_dir / "sweep" / "best_params.json", "w") as f:
        json.dump(best_params, f, indent=2)
    print(f"Best params written → {out_base_dir / 'sweep' / 'best_params.json'}")


def main():
    parser = argparse.ArgumentParser(
        description="Grid-search optimizer for WindowedNISScorer parameters.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Input / mode
    parser.add_argument(
        "receptions", nargs="?", type=Path, default=Path("results/flattened/lust-full-run0-veremi.csv"),
        help="Path to a single receptions.csv dataset file.",
    )
    parser.add_argument(
        "--sweep", nargs="?", const=Path("results/flattened/"), type=Path,
        metavar="DIR",
        help="Sweep every *.csv in DIR (default: results/flattened/).",
    )

    # Output
    parser.add_argument(
        "--out-dir", type=Path, default=Path("results/windowed-nis-opt/"),
        help="Base output directory (default: results/windowed-nis-opt/).",
    )
    parser.add_argument(
        "--replay-bin", default=None,
        help="Path to the replay executable (auto-detected if omitted).",
    )

    # Grid definition
    parser.add_argument(
        "--sigma-values", nargs="+", type=float,
        default=DEFAULT_SIGMA_VALUES, metavar="SIGMA",
        help="innovationSigma values to sweep [m].",
    )
    parser.add_argument(
        "--window-values", nargs="+", type=int,
        default=DEFAULT_WINDOW_VALUES, metavar="W",
        help="nisWindowSize values to sweep.",
    )
    parser.add_argument(
        "--tau-values", nargs="+", type=float,
        default=DEFAULT_TAU_VALUES, metavar="TAU",
        help="nisThreshold (τ) values to sweep.  Requires patched replay.",
    )
    parser.add_argument(
        "--min-samples-values", nargs="+", type=int,
        default=DEFAULT_MIN_SAMPLES_VALUES, metavar="M",
        help="nisMinSamples values to sweep.  Requires patched replay.",
    )

    # Evaluation
    parser.add_argument(
        "--metric", choices=sorted(VALID_METRICS), default=DEFAULT_METRIC,
        help="Primary optimisation metric (default: f1).",
    )
    parser.add_argument(
        "--threshold", type=float, default=DEFAULT_CLASSIFICATION_THRESHOLD,
        help="Confidence threshold for binary classification (default: 0.5).",
    )
    parser.add_argument(
        "--max-msgs", type=int, default=None,
        help="Stop replay after N messages (useful for fast smoke-tests).",
    )

    # Reporting
    parser.add_argument(
        "--top-n", type=int, default=5,
        help="Number of top configs to print (default: 5).",
    )
    parser.add_argument(
        "--plot", action="store_true",
        help="Save sigma × window heat-map PNG(s).",
    )

    args = parser.parse_args()

    if args.receptions is None and args.sweep is None:
        parser.print_help()
        sys.exit(1)

    replay_bin = find_replay_binary(args.replay_bin)
    has_threshold_flag, has_min_samples_flag = probe_extended_flags(replay_bin)

    # Warn and clamp if extended flags are unavailable
    if len(args.tau_values) > 1 and not has_threshold_flag:
        print(
            "WARNING: replay does not expose --nis-threshold (patch not applied). "
            "Fixing τ to its theoretical default (5.991) and ignoring --tau-values.",
            file=sys.stderr,
        )
        args.tau_values = [5.991]
    if len(args.min_samples_values) > 1 and not has_min_samples_flag:
        print(
            "WARNING: replay does not expose --nis-min-samples (patch not applied). "
            "Using default minSamples=3 and ignoring --min-samples-values.",
            file=sys.stderr,
        )
        args.min_samples_values = [3]

    grid = build_grid(
        args.sigma_values, args.window_values,
        args.tau_values, args.min_samples_values,
    )
    total_runs = len(grid)
    print(
        f"Grid: {len(args.sigma_values)} σ × {len(args.window_values)} W × "
        f"{len(args.tau_values)} τ × {len(args.min_samples_values)} min_samples "
        f"= {total_runs} configs"
    )
    if not has_threshold_flag:
        print("  (τ fixed to 5.991 — apply replay.cc patch to enable τ sweep)")
    print()

    if args.sweep is not None:
        if not args.sweep.exists() or not args.sweep.is_dir():
            sys.exit(f"Sweep directory '{args.sweep}' does not exist.")
        sweep(replay_bin, args.sweep, args.out_dir, grid, args,
              has_threshold_flag, has_min_samples_flag)
        return

    # Single-dataset path
    if not args.receptions.exists():
        sys.exit(f"File '{args.receptions}' does not exist.")

    print(f"Dataset: {args.receptions}")
    df = grid_search_dataset(
        replay_bin, args.receptions, grid, args,
        has_threshold_flag, has_min_samples_flag,
        verbose=True,
    )
    ranked = save_results(df, args.out_dir, args.metric)
    print_top(ranked, n=args.top_n, primary_metric=args.metric)

    if args.plot:
        best_tau = ranked.iloc[0]["tau"]
        plot_heatmap(ranked, args.out_dir, args.metric, tau_fixed=best_tau)

    print(f"Results → {args.out_dir}/")
    print(f"  grid_results.csv   full ranked table")
    print(f"  best_params.json   top-1 config")
    if args.plot:
        print(f"  heatmap_{args.metric}.png")


if __name__ == "__main__":
    main()
