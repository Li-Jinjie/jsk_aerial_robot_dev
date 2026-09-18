#!/usr/bin/env bash
# Formal ERK convergence and timing experiment for servo-aware NMPC.
#
# Fixed experiment configuration:
#   controller:       model 1 (servo-current NMPC)
#   platform:         Beetle-art
#   startup:          cold (zero state, zero input, and zero OCP initial guess)
#   thrust bounds:    0..30 N per rotor
#   servo bounds:     +/-90 deg
#   t_servo [s]:      0.086 and 0.008
#   ERK steps:        every integer from 1 through 20
#   repetitions:      3, shuffled independently with fixed seed 20260719
#   timing warm-up:   first 20 control rounds excluded from each repetition
#
# The convergence metric is the normalized one-shooting-interval error of the
# four-stage ERK discretization of da/dt=(a_cmd-a)/t_servo relative to its exact
# solution. The shooting interval is 0.1 s. The closed-loop plant is integrated
# separately by the 1 ms simulator. acados time is pooled over all three valid
# repetitions after the timing warm-up. A configuration with any OCP failure is
# reported with N/A timing and closed-loop convergence, while its independent
# analytical ERK error remains available.
#
# Formal results recorded on 2026-07-19. Analytical error is normalized RK4
# servo-state error [%]; acados entries are pooled mean/P95 time_tot [ms].
#
# | ERK | Error, t_servo=0.086 | acados mean/P95 | Error, t_servo=0.008 | acados mean/P95 |
# |---:|---:|---:|---:|---:|
# |   1 |          2.150e+00   | 0.399 / 0.733   |          7.584e+04   |       N/A       |
# |   2 |          8.204e-02   | 0.499 / 0.667   |          1.382e+05   |       N/A       |
# |   3 |          1.376e-02   | 0.617 / 0.772   |          2.178e+04   |       N/A       |
# |   4 |          4.013e-03   | 0.733 / 0.882   |          7.326e+02   |       N/A       |
# |   5 |          1.565e-03   | 0.937 / 1.484   |          1.146e+01   | 0.969 / 1.651   |
# |   6 |          7.308e-04   | 1.231 / 1.829   |          2.349e-01   | 1.065 / 1.712   |
# |   7 |          3.855e-04   | 1.228 / 1.967   |          1.428e-02   | 1.228 / 1.864   |
# |   8 |          2.221e-04   | 1.258 / 1.488   |          2.517e-03   | 1.285 / 1.523   |
# |   9 |          1.368e-04   | 1.485 / 2.273   |          8.343e-04   | 1.374 / 1.579   |
# |  10 |          8.877e-05   | 1.490 / 1.677   |          3.821e-04   | 1.652 / 2.589   |
# |  11 |          6.010e-05   | 1.671 / 2.685   |          2.084e-04   | 1.755 / 2.718   |
# |  12 |          4.212e-05   | 1.928 / 2.994   |          1.262e-04   | 1.834 / 2.780   |
# |  13 |          3.039e-05   | 2.071 / 3.262   |          8.189e-05   | 1.880 / 2.126   |
# |  14 |          2.248e-05   | 2.171 / 3.312   |          5.586e-05   | 2.005 / 2.245   |
# |  15 |          1.698e-05   | 2.339 / 3.551   |          3.957e-05   | 2.301 / 3.555   |
# |  16 |          1.306e-05   | 2.392 / 3.591   |          2.889e-05   | 2.429 / 3.718   |
# |  17 |          1.021e-05   | 2.339 / 2.575   |          2.162e-05   | 2.761 / 4.022   |
# |  18 |          8.099e-06   | 2.761 / 4.162   |          1.651e-05   | 2.670 / 3.962   |
# |  19 |          6.506e-06   | 2.864 / 4.328   |          1.284e-05   | 2.655 / 2.946   |
# |  20 |          5.285e-06   | 2.844 / 4.388   |          1.014e-05   | 2.884 / 4.281   |
#
# N/A denotes an OCP failure in at least one repetition. At t_servo=0.008 s,
# ERK 1 through 4 failed in all three repetitions; every other configuration
# completed all three repetitions and all 1500 controller rounds per run.
#
# Main observations:
#   - At t_servo=0.008 s, ERK 1 through 4 have RK4 substep amplification
#     magnitude greater than one and fail consistently. ERK 5 is linearly stable
#     but still has 11.46% analytical error and 19.73% maximum closed-loop metric
#     deviation relative to ERK 20.
#   - ERK 6 reduces those errors to 0.235% and 1.04%, respectively; ERK 10
#     reduces the closed-loop deviation to 0.0418%.
#   - At t_servo=0.086 s, ERK 1 already completes reliably with 2.15% analytical
#     error and 0.569% closed-loop deviation. acados mean time rises from about
#     0.4 ms at ERK 1 to about 2.8 ms at ERK 20, with expected timing noise.
#
# Generated results:
#   experiment_results/erk_formal/metrics.jsonl
#   experiment_results/erk_formal/run_schedule.csv
#   experiment_results/erk_formal/erk_formal_summary.{csv,md}
#   experiment_results/erk_formal/run_manifest.txt
#   experiment_results/erk_formal/{logs,round_times}/*
#
# Usage:
#   ./run_ERK_formal.sh
#   ./run_ERK_formal.sh /absolute/output/directory
#   SUMMARIZE_ONLY=1 ./run_ERK_formal.sh /existing/output/directory

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/erk_formal}"
LOG_DIR="${RESULT_DIR}/logs"
RAW_DIR="${RESULT_DIR}/round_times"
METRICS_FILE="${RESULT_DIR}/metrics.jsonl"
SCHEDULE_FILE="${RESULT_DIR}/run_schedule.csv"
PYTHON_BIN="${PYTHON_BIN:-python3}"

readonly RANDOM_SEED=20260719
readonly REPETITIONS=3
readonly EXPECTED_RUNS=120
readonly EXPECTED_CONTROL_ROUNDS=1500
readonly TIMING_WARMUP_ROUNDS=20

mkdir -p "${LOG_DIR}" "${RAW_DIR}"
if [[ "${SUMMARIZE_ONLY:-0}" == "1" ]]; then
    if [[ ! -s "${METRICS_FILE}" || ! -s "${SCHEDULE_FILE}" ]]; then
        echo "SUMMARIZE_ONLY=1 requires existing metrics and schedule files in ${RESULT_DIR}" >&2
        exit 1
    fi
else
    : > "${METRICS_FILE}"
    "${PYTHON_BIN}" - "${SCHEDULE_FILE}" "${RANDOM_SEED}" "${REPETITIONS}" <<'PY'
import csv
import pathlib
import random
import sys

path = pathlib.Path(sys.argv[1])
seed = int(sys.argv[2])
repetitions = int(sys.argv[3])
base = [(tau, steps) for tau in ("0.086", "0.008") for steps in range(1, 21)]
rows = []
order = 0
for repetition in range(1, repetitions + 1):
    configs = base.copy()
    random.Random(seed + repetition - 1).shuffle(configs)
    for tau, steps in configs:
        rows.append({
            "order": order,
            "repetition": repetition,
            "tau_s": tau,
            "erk_steps": steps,
        })
        order += 1

with path.open("w", newline="") as stream:
    writer = csv.DictWriter(
        stream,
        fieldnames=("order", "repetition", "tau_s", "erk_steps"),
        lineterminator="\n",
    )
    writer.writeheader()
    writer.writerows(rows)
PY
fi

record_log() {
    local log_file="$1"
    local raw_csv="$2"
    local order="$3"
    local repetition="$4"
    local tau="$5"
    local steps="$6"
    local process_status="$7"
    "${PYTHON_BIN}" - "${log_file}" "${raw_csv}" "${order}" "${repetition}" \
        "${tau}" "${steps}" "${process_status}" >> "${METRICS_FILE}" <<'PY'
import json
import pathlib
import sys

log_path, raw_path, order, repetition, tau, steps, status = sys.argv[1:]
metrics = None
for line in pathlib.Path(log_path).read_text(errors="replace").splitlines():
    if line.startswith("METRICS_JSON="):
        metrics = json.loads(line.split("=", 1)[1])

metrics_present = metrics is not None
if metrics is None:
    metrics = {
        "scenario": "servo_delay_sweep",
        "model": 1,
        "controller": "servo_current_cost",
        "parameters": {
            "servo_time_constant_s": float(tau),
            "ocp_sim_method_num_steps": int(steps),
            "startup_mode": "cold",
        },
        "startup": None,
        "overall": None,
        "timing": {"solver_failures": 1},
    }

metrics["experiment"] = {
    "phase": "erk_formal",
    "order": int(order),
    "repetition": int(repetition),
    "requested_model": 1,
    "requested_tau_s": float(tau),
    "requested_erk_steps": int(steps),
    "requested_startup_mode": "cold",
    "process_exit_status": int(status),
    "metrics_present": metrics_present,
    "log_file": str(pathlib.Path(log_path).resolve()),
    "round_time_file": str(pathlib.Path(raw_path).resolve()),
}
print(json.dumps(metrics, separators=(",", ":")))
PY
}

run_case() {
    local order="$1"
    local repetition="$2"
    local tau="$3"
    local steps="$4"
    local stem="repeat_${repetition}_tau_${tau}_erk_${steps}"
    local log_file="${LOG_DIR}/${stem}.log"
    local raw_csv="${RAW_DIR}/${stem}.csv"

    echo "Running order=${order}, repeat=${repetition}, tau=${tau}, ERK=${steps}"
    set +e
    MPLCONFIGDIR="${TMPDIR:-/tmp}/nmpc_erk_formal_mpl" \
        "${PYTHON_BIN}" "${SCRIPT_DIR}/sim_nmpc.py" 1 \
        --scenario servo_delay_sweep \
        --servo-time-constant "${tau}" \
        --ocp-sim-num-steps "${steps}" \
        --startup-mode cold \
        --test-thrust-max 30 \
        --servo-angle-max-deg 90 \
        --solve-time-csv "${raw_csv}" \
        --no_viz > "${log_file}" 2>&1
    local status=$?
    set -e

    record_log "${log_file}" "${raw_csv}" "${order}" "${repetition}" \
        "${tau}" "${steps}" "${status}"
    if [[ ${status} -ne 0 ]]; then
        echo "  Process failed (exit=${status}); see ${log_file}"
    fi
}

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    while IFS=, read -r order repetition tau steps; do
        if [[ "${order}" == "order" ]]; then
            continue
        fi
        run_case "${order}" "${repetition}" "${tau}" "${steps}"
    done < "${SCHEDULE_FILE}"
fi

"${PYTHON_BIN}" - "${METRICS_FILE}" "${SCHEDULE_FILE}" "${RESULT_DIR}" \
    "${EXPECTED_RUNS}" "${EXPECTED_CONTROL_ROUNDS}" "${TIMING_WARMUP_ROUNDS}" <<'PY'
import csv
import json
import math
import pathlib
import statistics
import sys

metrics_path = pathlib.Path(sys.argv[1])
schedule_path = pathlib.Path(sys.argv[2])
result_dir = pathlib.Path(sys.argv[3])
expected_runs = int(sys.argv[4])
expected_rounds = int(sys.argv[5])
timing_warmup_rounds = int(sys.argv[6])

records = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]
with schedule_path.open() as stream:
    schedule = list(csv.DictReader(stream))
if len(records) != expected_runs or len(schedule) != expected_runs:
    raise SystemExit(
        f"Expected {expected_runs} records and schedule rows; found {len(records)} and {len(schedule)}"
    )

expected_keys = {
    (repetition, tau, steps)
    for repetition in range(1, 4)
    for tau in (0.086, 0.008)
    for steps in range(1, 21)
}
schedule_keys = {
    (int(row["repetition"]), float(row["tau_s"]), int(row["erk_steps"]))
    for row in schedule
}
if schedule_keys != expected_keys:
    raise SystemExit("Run schedule does not contain the expected 3 x 2 x 20 matrix")
if sorted(int(row["order"]) for row in schedule) != list(range(expected_runs)):
    raise SystemExit("Run schedule order is not a unique contiguous range")

lookup = {}
for record in records:
    experiment = record.get("experiment") or {}
    key = (
        int(experiment.get("repetition")),
        float(experiment.get("requested_tau_s")),
        int(experiment.get("requested_erk_steps")),
    )
    if key in lookup:
        raise SystemExit(f"Duplicate run record for repetition/tau/ERK={key}")
    lookup[key] = record
if set(lookup) != expected_keys:
    raise SystemExit("Run records do not match the expected 3 x 2 x 20 matrix")

closed_loop_keys = (
    "position_rmse_m",
    "attitude_rmse_deg",
    "servo_cmd_rms_deg",
    "servo_cmd_excess_travel_deg",
    "servo_actual_excess_travel_deg",
)

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

def read_run(record):
    experiment = record.get("experiment") or {}
    timing = record.get("timing") or {}
    startup = record.get("startup") or {}
    raw_path = pathlib.Path(experiment.get("round_time_file", ""))
    basic_valid = (
        experiment.get("process_exit_status") == 0
        and experiment.get("metrics_present") is True
        and timing.get("solver_failures") == 0
        and all(startup.get(key) is not None for key in closed_loop_keys)
        and raw_path.is_file()
    )
    rows = []
    if raw_path.is_file():
        with raw_path.open() as stream:
            rows = list(csv.DictReader(stream))
    complete = (
        basic_valid
        and len(rows) == expected_rounds
        and all(int(row["solver_status"]) == 0 for row in rows)
        and all(row["acados_time_tot_ms"] for row in rows)
    )
    if not complete:
        return {"valid": False, "startup": startup, "rows": rows}
    return {"valid": True, "startup": startup, "rows": rows}

def analytical_error(tau, steps):
    shooting_interval = 0.1
    z = -shooting_interval / (steps * tau)
    stability_factor = 1.0 + z + z**2 / 2.0 + z**3 / 6.0 + z**4 / 24.0
    exact_decay = math.exp(-shooting_interval / tau)
    numerical_decay = stability_factor**steps
    normalized_error_pct = 100.0 * abs(numerical_decay - exact_decay) / (1.0 - exact_decay)
    return normalized_error_pct, abs(stability_factor), abs(stability_factor) < 1.0

summaries = {}
for tau in (0.086, 0.008):
    for steps in range(1, 21):
        runs = [read_run(lookup[(repetition, tau, steps)]) for repetition in range(1, 4)]
        config_valid = all(run["valid"] for run in runs)
        error_pct, amplification, linearly_stable = analytical_error(tau, steps)
        summary = {
            "tau_s": tau,
            "erk_steps": steps,
            "analytical_servo_error_pct": error_pct,
            "rk4_substep_amplification_abs": amplification,
            "rk4_linearly_stable": linearly_stable,
            "status": "ok" if config_valid else "failed",
            "valid_repetitions": sum(run["valid"] for run in runs),
            "total_repetitions": len(runs),
            "closed_loop_max_relative_deviation_to_erk20": None,
        }
        for key in closed_loop_keys:
            summary[f"{key}_mean"] = (
                statistics.fmean(run["startup"][key] for run in runs) if config_valid else None
            )
        if config_valid:
            steady_rows = [
                row
                for run in runs
                for row in run["rows"]
                if int(row["control_round"]) >= timing_warmup_rounds
            ]
            expected_steady = len(runs) * (expected_rounds - timing_warmup_rounds)
            if len(steady_rows) != expected_steady:
                raise SystemExit(
                    f"Expected {expected_steady} steady timing rows for tau={tau}, ERK={steps}; "
                    f"found {len(steady_rows)}"
                )
            acados_values = [float(row["acados_time_tot_ms"]) for row in steady_rows]
            wall_values = [float(row["wall_time_ms"]) for row in steady_rows]
            summary.update(describe(acados_values, "acados"))
            summary.update(describe(wall_values, "wall"))
        else:
            summary.update(describe([], "acados"))
            summary.update(describe([], "wall"))
        summaries[(tau, steps)] = summary

for tau in (0.086, 0.008):
    reference = summaries[(tau, 20)]
    if reference["status"] != "ok":
        raise SystemExit(f"ERK20 reference failed for tau={tau}; closed-loop convergence is undefined")
    for steps in range(1, 21):
        summary = summaries[(tau, steps)]
        if summary["status"] != "ok":
            continue
        deviations = []
        for key in closed_loop_keys:
            value = summary[f"{key}_mean"]
            reference_value = reference[f"{key}_mean"]
            deviations.append(abs(value - reference_value) / max(abs(reference_value), 1e-12))
        summary["closed_loop_max_relative_deviation_to_erk20"] = max(deviations)

summary_rows = [summaries[(tau, steps)] for tau in (0.086, 0.008) for steps in range(1, 21)]
with (result_dir / "erk_formal_summary.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
    writer.writeheader()
    writer.writerows(summary_rows)

def error_cell(tau, steps):
    return f'{summaries[(tau, steps)]["analytical_servo_error_pct"]:.3e}'

def timing_cell(tau, steps):
    row = summaries[(tau, steps)]
    if row["status"] != "ok":
        return "N/A"
    return f'{row["acados_mean_ms"]:.3f} / {row["acados_p95_ms"]:.3f}'

lines = [
    "# Formal servo-NMPC ERK convergence and timing",
    "",
    "Analytical error is the normalized one-shooting-interval servo-state RK4 error [%].",
    "acados timing is pooled mean/P95 time_tot [ms] after excluding 20 rounds per repetition.",
    "Each configuration has three repetitions; N/A denotes at least one OCP failure or incomplete run.",
    "",
    "| ERK | Error, $t_{servo}=0.086$ | acados mean/P95 | Error, $t_{servo}=0.008$ | acados mean/P95 |",
    "|---:|---:|---:|---:|---:|",
]
for steps in range(1, 21):
    lines.append(
        f"| {steps} | {error_cell(0.086, steps)} | {timing_cell(0.086, steps)} | "
        f"{error_cell(0.008, steps)} | {timing_cell(0.008, steps)} |"
    )
(result_dir / "erk_formal_summary.md").write_text("\n".join(lines) + "\n")

failed = [row for row in summary_rows if row["status"] != "ok"]
print(f"Summarized {len(summary_rows)} configurations: {len(summary_rows) - len(failed)} ok, {len(failed)} failed")
PY

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    {
        echo "Generated: $(date --iso-8601=seconds)"
        echo "Command: $0 $*"
        echo "Platform: Beetle-art"
        echo "Controller: model 1 servo-current"
        echo "TIME_SERVO: 0.086 0.008"
        echo "ERK_STEPS: 1..20"
        echo "REPETITIONS: ${REPETITIONS}"
        echo "RANDOM_SEED: ${RANDOM_SEED}"
        echo "TIMING_WARMUP_ROUNDS: ${TIMING_WARMUP_ROUNDS}"
        echo "EXPECTED_CONTROL_ROUNDS: ${EXPECTED_CONTROL_ROUNDS}"
        echo "Startup: cold"
        echo "Bounds: thrust=0..30N servo=+/-90deg"
    } > "${RESULT_DIR}/run_manifest.txt"
fi

echo "Formal ERK experiment complete. Results: ${RESULT_DIR}"
