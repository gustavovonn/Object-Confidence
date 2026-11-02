#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.metrics import (
    accuracy_score,
    f1_score,
    precision_recall_curve,
    precision_score,
    recall_score,
    roc_auc_score,
)

SCORERS = ["beta", "gaussian-nis", "windowed-nis"]


def find_replay_binary(custom_path=None):
    """Locates the replay executable."""
    if custom_path:
        p = Path(custom_path)
        if p.exists() and os.access(p, os.X_OK):
            return str(p)
        raise FileNotFoundError(f"Specified replay binary not found or not executable: {custom_path}")

    candidate = Path(__file__).parent.parent / "build" / "replay"
    if candidate.exists() and os.access(candidate, os.X_OK):
        return str(candidate)

    raise FileNotFoundError(
        "Could not locate 'replay' binary. Ensure it is built (e.g., build/replay) "
        "or specify path via --replay-bin."
    )


def run_scorer_replay(replay_bin, receptions_csv, output_csv, scorer, max_msgs=None, extra_flags=None):
    """Executes the replay binary for a specific scorer variant."""
    cmd = [replay_bin, str(receptions_csv), str(output_csv), "--scorer", scorer]
    if max_msgs is not None and max_msgs > 0:
        cmd.extend(["--max-msgs", str(max_msgs)])

    if extra_flags:
        cmd.extend(extra_flags)

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error running replay for scorer '{scorer}':", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(result.returncode)


def evaluate_records(records_csv, threshold=0.5):
    """
    Evaluates scorer records against ground truth.
    - Ground truth: attackType != 0 -> attack (1), 0 -> legitimate (0)
    - Prediction: confidence > threshold -> legitimate (0), <= threshold -> attack (1)
    """
    df = pd.read_csv(records_csv)
    if df.empty:
        raise ValueError(f"Output records CSV '{records_csv}' is empty.")

    # Ground truth: attack (!= 0) vs honest (0)
    y_true = (df["attackType"] != 0).astype(int)

    # Binary classification threshold
    y_pred = (df["confidence"] <= threshold).astype(int)

    # Attack score (1.0 - confidence) for continuous ranking metrics
    y_score = 1.0 - df["confidence"]

    acc = accuracy_score(y_true, y_pred)
    prec = precision_score(y_true, y_pred, zero_division=0)
    rec = recall_score(y_true, y_pred, zero_division=0)
    f1 = f1_score(y_true, y_pred, zero_division=0)

    if len(np.unique(y_true)) > 1:
        auc = roc_auc_score(y_true, y_score)
        prec_curve, rec_curve, _ = precision_recall_curve(y_true, y_score)
    else:
        auc = float("nan")
        prec_curve, rec_curve = None, None

    return {
        "accuracy": acc,
        "precision": prec,
        "recall": rec,
        "f1": f1,
        "auc": auc,
        "prec_curve": prec_curve,
        "rec_curve": rec_curve,
    }


def plot_pr_curves(results, output_path):
    """Generates Precision-Recall curves plot for all scorers."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Warning: matplotlib is not installed. Skipping PR curve plot.", file=sys.stderr)
        return

    plt.figure(figsize=(8, 6))
    for scorer, metrics in results.items():
        if metrics["prec_curve"] is not None and metrics["rec_curve"] is not None:
            plt.plot(
                metrics["rec_curve"],
                metrics["prec_curve"],
                label=f"{scorer} (AUC = {metrics['auc']:.3f})",
            )

    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.title("Precision-Recall Curve Comparison by Scorer")
    plt.legend(loc="lower left")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def save_reports(results, output_dir: Path, generate_plot=False):
    """Saves metrics.csv, table.md, and optionally pr_curves.png inside output_dir."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Build DataFrame
    rows = []
    for scorer in SCORERS:
        m = results[scorer]
        rows.append({
            "scorer": scorer,
            "accuracy": round(m["accuracy"], 3),
            "precision": round(m["precision"], 3),
            "recall": round(m["recall"], 3),
            "f1": round(m["f1"], 3),
            "auc": round(m["auc"], 3) if not np.isnan(m["auc"]) else "N/A",
        })
    df_metrics = pd.DataFrame(rows)

    # 1. Save CSV
    csv_path = output_dir / "metrics.csv"
    df_metrics.to_csv(csv_path, index=False)

    # 2. Save Markdown table
    md_path = output_dir / "table.md"
    with open(md_path, "w") as f:
        f.write("# Scorer Comparison Metrics\n\n")
        f.write(df_metrics.to_markdown(index=False))
        f.write("\n")

    # 3. Save PR plot if requested
    if generate_plot:
        plot_path = output_dir / "pr_curves.png"
        plot_pr_curves(results, plot_path)


def generate_sweep_summary(all_results, output_base_dir: Path):
    """Calculates macro-average meta-metrics across all sweep subsets and saves summary_metrics.csv and summary_table.md."""
    summary_rows = []

    for scorer in SCORERS:
        accs, precs, recs, f1s, aucs = [], [], [], [], []

        for _stem, res in all_results.items():
            m = res[scorer]
            accs.append(m["accuracy"])
            precs.append(m["precision"])
            recs.append(m["recall"])
            f1s.append(m["f1"])
            if not np.isnan(m["auc"]):
                aucs.append(m["auc"])

        macro_acc = np.mean(accs) if accs else 0.0
        macro_prec = np.mean(precs) if precs else 0.0
        macro_rec = np.mean(recs) if recs else 0.0
        macro_f1 = np.mean(f1s) if f1s else 0.0
        macro_auc = np.mean(aucs) if aucs else np.nan

        summary_rows.append({
            "scorer": scorer,
            "accuracy": round(macro_acc, 3),
            "precision": round(macro_prec, 3),
            "recall": round(macro_rec, 3),
            "f1": round(macro_f1, 3),
            "auc": round(macro_auc, 3) if not np.isnan(macro_auc) else "N/A",
        })

    df_summary = pd.DataFrame(summary_rows)

    # Output to terminal
    num_datasets = len(all_results)
    print(f"\n=== Sweep Macro-Averages Across {num_datasets} Datasets ===")
    header = f"{'scorer':<16}{'accuracy':<10}{'precision':<11}{'recall':<10}{'f1':<8}{'auc':<8}"
    print(header)
    for row in summary_rows:
        auc_str = f"{row['auc']:.3f}" if isinstance(row['auc'], (float, int)) else str(row['auc'])
        print(
            f"{row['scorer']:<16}{row['accuracy']:<10.3f}{row['precision']:<11.3f}"
            f"{row['recall']:<10.3f}{row['f1']:<8.3f}{auc_str:<8}"
        )

    # Save summary files
    output_base_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_base_dir / "summary_metrics.csv"
    df_summary.to_csv(csv_path, index=False)

    md_path = output_base_dir / "summary_table.md"
    with open(md_path, "w") as f:
        f.write(f"# Sweep Summary: Macro-Average Metrics ({num_datasets} Subsets)\n\n")
        f.write(df_summary.to_markdown(index=False))
        f.write("\n")

    print(f"\nSaved sweep summary to: {output_base_dir}/ (summary_metrics.csv, summary_table.md)")


def process_dataset(receptions_path: Path, output_base_dir: Path, replay_bin: str, args, extra_flags):
    """Processes a single receptions.csv file across all scorers."""
    stem_name = receptions_path.stem
    target_dir = output_base_dir / stem_name

    results = {}
    temp_files = []

    for scorer in SCORERS:
        out_csv = receptions_path.parent / f"records_{scorer}_{stem_name}.csv"
        temp_files.append(out_csv)

        run_scorer_replay(
            replay_bin,
            receptions_path,
            out_csv,
            scorer,
            max_msgs=args.max_msgs,
            extra_flags=extra_flags,
        )

        metrics = evaluate_records(out_csv, threshold=args.threshold)
        results[scorer] = metrics

    # Format side-by-side metric output table to terminal
    print(f"\n=== Dataset: {stem_name} ===")
    header = f"{'scorer':<16}{'accuracy':<10}{'precision':<11}{'recall':<10}{'f1':<8}{'auc':<8}"
    print(header)
    for scorer in SCORERS:
        m = results[scorer]
        auc_str = f"{m['auc']:.3f}" if not np.isnan(m["auc"]) else "N/A"
        print(
            f"{scorer:<16}{m['accuracy']:<10.3f}{m['precision']:<11.3f}"
            f"{m['recall']:<10.3f}{m['f1']:<8.3f}{auc_str:<8}"
        )

    save_reports(results, target_dir, generate_plot=args.plot)
    print(f"Saved results to: {target_dir}/")

    if not args.keep_records:
        for f in temp_files:
            if f.exists():
                try:
                    f.unlink()
                except OSError:
                    pass

    return results


def sweep_datasets(in_dir: Path, out_base_dir: Path, replay_bin: str, args, extra_flags):
    """Finds all CSV dataset files in in_dir, evaluates each, and generates macro-average meta-metrics."""
    if not in_dir.exists() or not in_dir.is_dir():
        sys.exit(f"Error: input sweep directory '{in_dir}' does not exist.")

    csv_files = sorted(list(in_dir.glob("*.csv")))
    if not csv_files:
        print(f"No '.csv' files found under {in_dir}", file=sys.stderr)
        return

    print(f"Sweeping {len(csv_files)} datasets from '{in_dir}' into '{out_base_dir}'...")
    all_results = {}

    for idx, csv_file in enumerate(csv_files, 1):
        print(f"\n[{idx}/{len(csv_files)}] Processing {csv_file.name}...")
        results = process_dataset(csv_file, out_base_dir, replay_bin, args, extra_flags)
        if results:
            all_results[csv_file.stem] = results

    if all_results:
        generate_sweep_summary(all_results, out_base_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark and compare Object Confidence Engine scorers."
    )
    parser.add_argument("receptions", nargs="?", type=Path, help="Path to receptions.csv dataset file")
    parser.add_argument(
        "--sweep",
        nargs="?",
        const=Path("results/flattened/"),
        type=Path,
        help="Run sweep over all CSV datasets in input directory (default: results/flattened/)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("results/score-comparison/"),
        help="Base output directory for stored results (default: results/score-comparison/)",
    )
    parser.add_argument("--max-msgs", type=int, default=None, help="Stop replay after N messages")
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Confidence threshold: > threshold is legitimate, <= is attack (default: 0.5)",
    )
    parser.add_argument("--replay-bin", type=str, default=None, help="Path to custom replay executable")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate PR curve plot (pr_curves.png) in each target result directory",
    )
    parser.add_argument("--innovation-sigma", type=float, default=None, help="Forwarded to replay")
    parser.add_argument("--forgetting", type=float, default=None, help="Forwarded to replay")
    parser.add_argument("--nis-window", type=int, default=None, help="Forwarded to replay")
    parser.add_argument(
        "--keep-records",
        action="store_true",
        help="Retain intermediate records_<scorer>.csv files after evaluation",
    )

    args = parser.parse_args()
    replay_bin = find_replay_binary(args.replay_bin)

    extra_flags = []
    if args.innovation_sigma is not None:
        extra_flags.extend(["--innovation-sigma", str(args.innovation_sigma)])
    if args.forgetting is not None:
        extra_flags.extend(["--forgetting", str(args.forgetting)])
    if args.nis_window is not None:
        extra_flags.extend(["--nis-window", str(args.nis_window)])

    if args.sweep is not None:
        sweep_datasets(args.sweep, args.out_dir, replay_bin, args, extra_flags)
    elif args.receptions:
        if not args.receptions.exists():
            sys.exit(f"Error: input file '{args.receptions}' does not exist.")
        process_dataset(args.receptions, args.out_dir, replay_bin, args, extra_flags)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
