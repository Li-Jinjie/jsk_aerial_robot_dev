#!/usr/bin/env bash
# Per-control-round NMPC timing sweep for different ERK integration steps.
#
# Default configuration:
#   controller:       model 1 (servo NMPC with current cost)
#   t_servo:          0.086 s
#   ERK steps:        1 2 3 4 5 6
#   task:             servo_delay_sweep, cold start (same reference as legacy/default)
#   actuator bounds:  0..30 N and +/-90 deg
#   control period:   10 ms (100 Hz)
#
# Each round CSV records:
#   - wall_time_ms: complete reference/parameter update plus solve_for_x0()
#   - acados_time_tot_ms: time_tot reported by acados for the solve itself
#   - acados_time_lin_ms and acados_time_qp_ms
#   - simulation time, reference phase, and solver status
#
# The summary reports all rounds and steady-state statistics after discarding the
# first TIMING_WARMUP_ROUNDS rounds (20 by default). Wall time depends on host load;
# acados time_tot is the cleaner measure for comparing ERK configurations.
#
# Results recorded on 2026-07-18 (model 1, first 20 rounds excluded):
#
# | ERK | Wall mean/P95/P99/max [ms]     | acados mean/P95/P99/max [ms] | >10 ms |
# |  1  | 0.616 / 0.770 / 0.931 / 24.810 | 0.386 / 0.535 / 0.695 / 0.997 |    1   |
# |  2  | 0.730 / 0.896 / 1.039 / 19.040 | 0.510 / 0.667 / 0.793 / 1.799 |    1   |
# |  3  | 0.899 / 1.063 / 1.356 / 20.294 | 0.670 / 0.837 / 1.031 / 2.660 |    1   |
# |  4  | 1.373 / 1.963 / 2.408 / 26.847 | 1.047 / 1.491 / 1.787 / 4.426 |    1   |
# |  5  | 1.108 / 1.244 / 1.422 / 23.309 | 0.884 / 1.029 / 1.168 / 2.582 |    1   |
# |  6  | 1.273 / 1.467 / 1.737 / 18.825 | 1.047 / 1.233 / 1.424 / 3.685 |    1   |
#
# All six runs completed 1500/1500 rounds with zero solver failures. Each wall
# max above 10 ms is an isolated host scheduling outlier: acados max remained
# below 4.5 ms. The small ERK=4/5 non-monotonicity indicates measurement noise;
# use repeated, randomized-order runs for a rigorous hardware benchmark.
#
# Run:
#   ./run_erk_timing_sweep.sh
#   ./run_erk_timing_sweep.sh /absolute/output/directory
#
# Optional overrides:
#   TIMING_MODELS="1 92" ERK_STEPS="1 2 3 4 5 6" TIME_SERVO=0.086 \
#     TIMING_WARMUP_ROUNDS=20 ./run_erk_timing_sweep.sh /tmp/erk_timing

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/erk_timing_tau_0p086}"
LOG_DIR="${RESULT_DIR}/logs"
RAW_DIR="${RESULT_DIR}/round_times"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TIME_SERVO="${TIME_SERVO:-0.086}"
TIMING_WARMUP_ROUNDS="${TIMING_WARMUP_ROUNDS:-20}"

read -r -a timing_models <<< "${TIMING_MODELS:-1}"
read -r -a erk_steps <<< "${ERK_STEPS:-1 2 3 4 5 6}"

mkdir -p "${LOG_DIR}" "${RAW_DIR}"

controller_name() {
    case "$1" in
        0) echo "no_servo" ;;
        1) echo "servo_current_cost" ;;
        92) echo "servo_old_cost" ;;
        *) echo "model_$1" ;;
    esac
}

for model in "${timing_models[@]}"; do
    name="$(controller_name "${model}")"
    for steps in "${erk_steps[@]}"; do
        raw_csv="${RAW_DIR}/model_${model}_${name}_erk_${steps}.csv"
        log_file="${LOG_DIR}/model_${model}_${name}_erk_${steps}.log"
        echo "Running model=${model}, t_servo=${TIME_SERVO}, ERK steps=${steps}"
        set +e
        MPLCONFIGDIR="${TMPDIR:-/tmp}/nmpc_erk_timing_mpl" \
            "${PYTHON_BIN}" "${SCRIPT_DIR}/sim_nmpc.py" "${model}" \
            --scenario servo_delay_sweep \
            --servo-time-constant "${TIME_SERVO}" \
            --ocp-sim-num-steps "${steps}" \
            --startup-mode cold \
            --test-thrust-max 30 \
            --servo-angle-max-deg 90 \
            --solve-time-csv "${raw_csv}" \
            --no_viz > "${log_file}" 2>&1
        status=$?
        set -e
        if [[ ${status} -ne 0 ]]; then
            echo "  FAILED (exit=${status}); see ${log_file}"
        fi
    done
done

"${PYTHON_BIN}" - "${RESULT_DIR}" "${TIME_SERVO}" "${TIMING_WARMUP_ROUNDS}" <<'PY'
import csv
import pathlib
import statistics
import sys

result_dir = pathlib.Path(sys.argv[1])
tau = float(sys.argv[2])
warmup_rounds = int(sys.argv[3])
raw_files = sorted((result_dir / "round_times").glob("model_*_erk_*.csv"))
controller_names = {0: "No-servo", 1: "Servo/current cost", 92: "Servo/old cost"}


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def describe(values, prefix):
    return {
        f"{prefix}_mean_ms": statistics.fmean(values) if values else None,
        f"{prefix}_p50_ms": percentile(values, 50),
        f"{prefix}_p95_ms": percentile(values, 95),
        f"{prefix}_p99_ms": percentile(values, 99),
        f"{prefix}_max_ms": max(values) if values else None,
    }


all_rows = []
summary_rows = []
for path in raw_files:
    parts = path.stem.split("_")
    model = int(parts[1])
    steps = int(parts[-1])
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row["model"] = model
        row["controller"] = controller_names.get(model, str(model))
        row["erk_steps"] = steps
        row["servo_time_constant_s"] = tau
        all_rows.append(row)

    valid = [row for row in rows if int(row["solver_status"]) == 0]
    steady = [row for row in valid if int(row["control_round"]) >= warmup_rounds]
    wall_all = [float(row["wall_time_ms"]) for row in valid]
    wall_steady = [float(row["wall_time_ms"]) for row in steady]
    acados_all = [float(row["acados_time_tot_ms"]) for row in valid if row["acados_time_tot_ms"]]
    acados_steady = [float(row["acados_time_tot_ms"]) for row in steady if row["acados_time_tot_ms"]]
    summary = {
        "model": model,
        "controller": controller_names.get(model, str(model)),
        "servo_time_constant_s": tau,
        "erk_steps": steps,
        "rounds": len(rows),
        "valid_rounds": len(valid),
        "failed_rounds": len(rows) - len(valid),
        "warmup_rounds_excluded": warmup_rounds,
        "deadline_misses_all": sum(value > 10.0 for value in wall_all),
        "deadline_misses_steady": sum(value > 10.0 for value in wall_steady),
    }
    summary.update(describe(wall_all, "wall_all"))
    summary.update(describe(wall_steady, "wall_steady"))
    summary.update(describe(acados_all, "acados_all"))
    summary.update(describe(acados_steady, "acados_steady"))
    summary_rows.append(summary)

all_rows.sort(key=lambda row: (int(row["model"]), int(row["erk_steps"]), int(row["control_round"])))
summary_rows.sort(key=lambda row: (row["model"], row["erk_steps"]))

if all_rows:
    with (result_dir / "all_round_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
if summary_rows:
    with (result_dir / "timing_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)


def fmt(value, digits=3):
    return "-" if value is None else f"{value:.{digits}f}"


lines = [
    f"# NMPC timing versus ERK steps ($t_{{servo}}={tau:.3f}$ s)",
    "",
    f"Statistics below exclude the first {warmup_rounds} controller rounds. The control deadline is 10 ms.",
    "",
    "| Model | ERK steps | Valid/total | Wall mean | Wall P95 | Wall P99 | Wall max | acados mean | acados P95 | acados P99 | acados max | >10 ms |",
    "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
]
for row in summary_rows:
    lines.append(
        f"| {row['controller']} | {row['erk_steps']} | {row['valid_rounds']}/{row['rounds']} | "
        f"{fmt(row['wall_steady_mean_ms'])} | {fmt(row['wall_steady_p95_ms'])} | "
        f"{fmt(row['wall_steady_p99_ms'])} | {fmt(row['wall_steady_max_ms'])} | "
        f"{fmt(row['acados_steady_mean_ms'])} | {fmt(row['acados_steady_p95_ms'])} | "
        f"{fmt(row['acados_steady_p99_ms'])} | {fmt(row['acados_steady_max_ms'])} | "
        f"{row['deadline_misses_steady']} |"
    )
(result_dir / "timing_summary.md").write_text("\n".join(lines) + "\n")
PY

{
    echo "Generated: $(date --iso-8601=seconds)"
    echo "Command: $0 $*"
    echo "TIME_SERVO: ${TIME_SERVO}"
    echo "TIMING_MODELS: ${timing_models[*]}"
    echo "ERK_STEPS: ${erk_steps[*]}"
    echo "TIMING_WARMUP_ROUNDS: ${TIMING_WARMUP_ROUNDS}"
} > "${RESULT_DIR}/run_manifest.txt"

echo "Timing sweep complete. Results: ${RESULT_DIR}"
