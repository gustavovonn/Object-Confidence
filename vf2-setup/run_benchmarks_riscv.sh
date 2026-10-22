#!/bin/sh
# ==============================================================================
# ObjectConfidenceEngine Latency Benchmark Runner
# Targets: StarFive VisionFive 2 (JH7110)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

REPO_ROOT="$(cd "$SCRIPT_DIR/.." 2>/dev/null && pwd || echo "$SCRIPT_DIR")"

DEFAULT_BIN=""
if [ -x "$SCRIPT_DIR/replay" ]; then
    DEFAULT_BIN="$SCRIPT_DIR/replay"
elif [ -x "$REPO_ROOT/build/replay" ]; then
    DEFAULT_BIN="$REPO_ROOT/build/replay"
elif [ -x "$REPO_ROOT/build-riscv/replay_stripped" ]; then
    DEFAULT_BIN="$REPO_ROOT/build-riscv/replay_stripped"
fi

# Results directory: tmpfs on the board, results/benchmark on the host
if [ -d "$SCRIPT_DIR/data" ]; then
    RESULTS_DIR="$SCRIPT_DIR/results"
else
    RESULTS_DIR="$REPO_ROOT/results/benchmark"
fi

BIN=""
DATA_DIR=""
PROFILE=""
MAX_MSGS=0
REPEAT=1
CORE_ID=""
CUSTOM_OUT=""

# Parse CLI arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --bin|-b)
            BIN="$2"
            shift 2
            ;;
        --dir|-d)
            DATA_DIR="$2"
            shift 2
            ;;
        --profile|-p)
            PROFILE="$2"
            shift 2
            ;;
        --max-msgs|-n)
            MAX_MSGS="$2"
            shift 2
            ;;
        --repeat|-r)
            REPEAT="$2"
            shift 2
            ;;
        --core|-c)
            CORE_ID="$2"
            shift 2
            ;;
        --out|-o)
            CUSTOM_OUT="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -p, --profile <name>   Run a single fault subset (e.g. ConstPos); default: all"
            echo "  -d, --dir <path>       Directory with <subset>.csv receptions (default: $SCRIPT_DIR/data)"
            echo "  -b, --bin <path>       Path to the replay binary (default: $DEFAULT_BIN)"
            echo "  -n, --max-msgs <N>     Maximum messages per subset, 0 for all (default: 0)"
            echo "  -r, --repeat <N>       Repetitions for stability (default: 1)"
            echo "  -c, --core <id>        Pin process to specific CPU core (e.g. 1-3 on Quad-Core SBC)"
            echo "  -o, --out <csv_path>   Custom path for CSV output"
            echo "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Resolve binary
[ -z "$BIN" ] && BIN="$DEFAULT_BIN"

if [ ! -f "$BIN" ]; then
    echo "[!] Error: replay binary not found at '$BIN'."
    echo "    Specify via --bin <path> or compile using cmake -B build-riscv -S . -DSTATIC_BUILD=ON"
    exit 1
fi

if [ ! -x "$BIN" ]; then
    echo "[!] Error: Binary is not executable: $BIN"
    exit 1
fi

DATASETS=""
if [ -n "$DATA_DIR" ] || [ -d "$SCRIPT_DIR/data" ]; then
    [ -z "$DATA_DIR" ] && DATA_DIR="$SCRIPT_DIR/data"
    for f in "$DATA_DIR"/*.csv; do
        [ -f "$f" ] && DATASETS="$DATASETS $f"
    done
else
    for d in $(ls -td "$REPO_ROOT"/results/*/ 2>/dev/null); do
        for f in "$d"*/receptions.csv; do
            [ -f "$f" ] && DATASETS="$DATASETS $f"
        done
        [ -n "$DATASETS" ] && break
    done
fi

subset_of() {
    case "$1" in
        */receptions.csv) basename "$(dirname "$1")" ;;
        *) basename "$1" .csv ;;
    esac
}

if [ -n "$PROFILE" ]; then
    SELECTED=""
    for f in $DATASETS; do
        [ "$(subset_of "$f")" = "$PROFILE" ] && SELECTED="$f"
    done
    if [ -z "$SELECTED" ]; then
        echo "[!] Error: subset '$PROFILE' not found among:$(for f in $DATASETS; do printf ' %s' "$(subset_of "$f")"; done)"
        exit 1
    fi
    DATASETS="$SELECTED"
fi

if [ -z "$DATASETS" ]; then
    echo "[!] Error: no receptions found."
    echo "    Specify via --dir <path> or generate them with KEEP_RECEPTIONS=1 scripts/run_preliminary.sh"
    exit 1
fi

mkdir -p "$RESULTS_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S" 2>/dev/null || date +"%s")
if [ -n "$CUSTOM_OUT" ]; then
    CSV_OUT="$CUSTOM_OUT"
else
    CSV_OUT="$RESULTS_DIR/benchmark_riscv_${TIMESTAMP}.csv"
fi
rm -f "$CSV_OUT"

# Core pinning (taskset)
CMD_PREFIX=""
if [ -n "$CORE_ID" ]; then
    if command -v taskset >/dev/null 2>&1; then
        CMD_PREFIX="taskset -c $CORE_ID"
        echo "[*] Pinning benchmark process to CPU Core $CORE_ID via taskset"
    else
        echo "[!] Warning: taskset not available, running without core affinity."
    fi
fi

# Elevate priority if root
if [ "$(id -u)" = "0" ]; then
    if command -v nice >/dev/null 2>&1; then
        CMD_PREFIX="nice -n -20 $CMD_PREFIX"
        echo "[*] Running with maximum process priority"
    fi
    if [ -w /proc/sys/vm/drop_caches ]; then
        sync
        echo 3 > /proc/sys/vm/drop_caches
        echo "[*] Page caches flushed."
    fi
fi

echo "=============================================================================="
echo "      OBJECT CONFIDENCE LATENCY BENCHMARK SUITE (STARFIVE RISC-V SBC)"
echo "=============================================================================="
echo " Architecture     : $(uname -m) (Kernel: $(uname -r))"
echo " Replay Binary    : $BIN"
echo " Subsets          :$(for f in $DATASETS; do printf ' %s' "$(subset_of "$f")"; done)"
echo " Max Messages     : $MAX_MSGS (0: all)"
echo " Repetitions      : $REPEAT"
echo " Output CSV       : $CSV_OUT"
echo "=============================================================================="

run_bench() {
    _data="$1"
    _subset="$(subset_of "$_data")"
    _run=1
    while [ "$_run" -le "$REPEAT" ]; do
        printf "[-] [%-18s] run %s/%s ... " "$_subset" "$_run" "$REPEAT"
        _status=0
        _out=$($CMD_PREFIX "$BIN" "$_data" /dev/null --max-msgs "$MAX_MSGS" --csv-out "$CSV_OUT" \
            --dataset "$_subset" 2>&1) || _status=$?
        if [ $_status -ne 0 ]; then
            echo "FAILED (exit code $_status)!"
            echo "$_out" | sed 's/^/    /'
            exit 1
        fi
        echo "Done."
        _run=$((_run + 1))
    done
}

for f in $DATASETS; do
    # Warm-up pass (discarded)
    $CMD_PREFIX "$BIN" "$f" /dev/null --max-msgs 1000 >/dev/null 2>&1 || true
    run_bench "$f"
done

printf "\n==============================================================================\n"
echo "              WHOLE PIPELINE (onMotionVectors) PER SUBSET, MEAN OVER RUNS"
echo "=============================================================================="

awk -F, '
BEGIN {
    printf "%-18s | %-4s | %-9s | %-10s | %-10s | %-10s | %-10s | %-12s\n", "Dataset", "Runs", "Messages", "Mean (us)", "p50 (us)", "p99 (us)", "Max (us)", "Msgs/sec"
    print "-------------------+------+-----------+------------+------------+------------+------------+-------------"
}
NR > 1 && $2 == "onMotionVectors" {
    if (!($1 in runs)) order[++n] = $1
    runs[$1]++; msgs[$1] = $3; mean[$1] += $4; p50[$1] += $5; p99[$1] += $6; mx[$1] += $7
}
END {
    for (i = 1; i <= n; i++) {
        d = order[i]; r = runs[d]; m = mean[d] / r
        printf "%-18s | %4d | %9d | %10.2f | %10.2f | %10.2f | %10.2f | %12.0f\n", d, r, msgs[d], m, p50[d] / r, p99[d] / r, mx[d] / r, (m > 0) ? 1000000.0 / m : 0
    }
}
' "$CSV_OUT"

echo "=============================================================================="
echo "[+] Full raw dataset saved to: $CSV_OUT"

awk -F, '
NR > 1 {
    if (!($2 in count)) order[++n] = $2
    count[$2]++
    sum_mean[$2] += $4; sumsq_mean[$2] += $4 * $4
    sum_p99[$2] += $6;  sumsq_p99[$2] += $6 * $6
    sum_share[$2] += $8
}
END {
    if (n > 0) {
        printf "\n==============================================================================\n"
        printf "          CROSS-SUBSET PER-STAGE SUMMARY (MEAN +- STD DEV OVER ALL RUNS)\n"
        printf "==============================================================================\n"
        printf "%-18s | %-5s | %-22s | %-22s | %-8s\n", "Stage", "Runs", "Mean latency (us)", "p99 latency (us)", "% whole"
        print "-------------------+-------+------------------------+------------------------+---------"
        for (i = 1; i <= n; i++) {
            s = order[i]; c = count[s]
            m = sum_mean[s] / c; vm = (c > 1) ? (sumsq_mean[s] - sum_mean[s] * sum_mean[s] / c) / (c - 1) : 0
            p = sum_p99[s] / c;  vp = (c > 1) ? (sumsq_p99[s] - sum_p99[s] * sum_p99[s] / c) / (c - 1) : 0
            printf "%-18s | %5d | %9.2f +- %-9.2f | %9.2f +- %-9.2f | %7.1f\n", s, c, m, (vm > 0) ? sqrt(vm) : 0, p, (vp > 0) ? sqrt(vp) : 0, sum_share[s] / c
        }
        printf "==============================================================================\n"
    }
}
' "$CSV_OUT"

MD_OUT="${CSV_OUT%.csv}.md"
awk -F, -v timestamp="$TIMESTAMP" -v maxmsgs="$MAX_MSGS" -v repeat="$REPEAT" -v arch="$(uname -m)" -v csvfile="$(basename "$CSV_OUT")" '
BEGIN {
    print "# ObjectConfidenceEngine Latency (StarFive VisionFive 2 RISC-V SBC)"
    print ""
    print "- **Run Timestamp**: " timestamp
    print "- **Architecture**: " arch
    print "- **Max Messages per Subset**: " maxmsgs " (0: all)"
    print "- **Repetitions**: " repeat
    print "- **CSV**: [`" csvfile "`](" csvfile ")"
    print ""
    print "## Per-stage latency, every run"
    print ""
    print "| Dataset | Stage | Samples | Mean (us) | p50 (us) | p99 (us) | Max (us) | % of whole |"
    print "| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |"
}
NR > 1 {
    printf "| %s | %s | %s | %.2f | %.2f | %.2f | %.2f | %.1f |\n", $1, $2, $3, $4, $5, $6, $7, $8
}
' "$CSV_OUT" > "$MD_OUT"

echo "[+] Markdown report saved to: $MD_OUT"
echo "=============================================================================="
