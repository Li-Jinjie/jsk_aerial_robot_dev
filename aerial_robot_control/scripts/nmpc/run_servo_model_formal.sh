#!/usr/bin/env bash
# Formal servo-model/ERK comparison for sim_nmpc.py.
#
# Fixed experiment configuration:
#   platform:       Beetle-art
#   startup:        cold (zero state, zero input, and zero OCP initial guess)
#   thrust bounds:  0..30 N per rotor
#   servo bounds:   +/-90 deg
#   metric window:  first 2 s after the nonzero position command
#   t_servo [s]:    0.200, 0.160, 0.120, 0.086, 0.040, 0.020, 0.012, 0.008
#
# ERK denotes the number of integration substeps in each NMPC prediction
# interval. The closed-loop plant is always integrated with the 1 ms simulator.
# Servo excess is the four-channel mean total travel beyond the direct
# start-to-end change. Each table entry is command/actual excess in degrees.
# A run with any OCP failure is reported as N/A, even when partial metrics exist.
#
# Formal results recorded on 2026-07-19:
#
# | t_servo | No-servo ERK 1 | No-servo ERK 20 | Servo-current ERK 1 | Servo-current ERK 5 | Servo-current ERK 20 |
# |   [s]   | cmd/actual [deg] | cmd/actual [deg] |   cmd/actual [deg]   |   cmd/actual [deg]   |    cmd/actual [deg]   |
# |  0.200  |       N/A        |        N/A        |     42.60 / 12.06    |     42.67 / 12.07    |      42.67 / 12.07    |
# |  0.160  | 3235.52 / 259.87 | 3224.83 / 282.91 |     37.35 / 14.20    |     37.41 / 14.22    |      37.41 / 14.22    |
# |  0.120  | 3031.44 / 269.73 | 2443.45 / 255.62 |     29.73 / 16.83    |     29.83 / 16.87    |      29.83 / 16.87    |
# |  0.086  | 2858.64 / 290.51 | 2574.01 / 290.07 |     29.12 / 21.91    |     29.12 / 22.03    |      29.12 / 22.03    |
# |  0.040  | 2454.83 / 414.81 | 2640.28 / 419.76 |     49.94 / 28.44    |     36.55 / 33.01    |      36.55 / 33.01    |
# |  0.020  | 3275.65 / 914.31 | 2929.53 / 823.94 |          N/A         |     48.39 / 46.78    |      48.43 / 46.84    |
# |  0.012  |3093.49 / 1337.30 |3229.70 / 1386.72 |          N/A         |     57.52 / 56.86    |      57.70 / 57.02    |
# |  0.008  |3732.96 / 2177.60 |4477.88 / 2585.17 |          N/A         |     57.34 / 55.34    |      62.06 / 61.69    |
#
# N/A denotes an OCP failure. Both no-servo runs failed at t_servo=0.200 s;
# servo-current ERK 1 failed at 0.020, 0.012, and 0.008 s. All other runs
# completed without solver failures.
#
# Main observations:
#   - At t_servo <= 0.020 s, both no-servo ERK settings retain large command
#     and actual excess travel; increasing its ERK steps does not remove the
#     oscillation.
#   - Servo-current ERK 1 fails throughout that small-t_servo range, whereas
#     ERK 5 and ERK 20 complete and greatly reduce excess travel.
#   - The data therefore do not support describing no-servo ERK 1 as stable in
#     this formal setup, although it avoids the small-t_servo OCP failures seen
#     with servo-current ERK 1.
#
# Generated results:
#   experiment_results/servo_model_formal/metrics.jsonl
#   experiment_results/servo_model_formal/formal_tau_sweep.{csv,md}
#   experiment_results/servo_model_formal/run_manifest.txt
#   experiment_results/servo_model_formal/logs/*.log
#   experiment_results/servo_model_formal/plots/*.{png,pdf}
#
# Usage:
#   ./run_servo_model_formal.sh
#   ./run_servo_model_formal.sh /absolute/output/directory
#   SUMMARIZE_ONLY=1 ./run_servo_model_formal.sh /existing/output/directory

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/servo_model_formal}"
LOG_DIR="${RESULT_DIR}/logs"
PLOT_DIR="${RESULT_DIR}/plots"
METRICS_FILE="${RESULT_DIR}/metrics.jsonl"
PYTHON_BIN="${PYTHON_BIN:-python3}"

readonly -a TAU_VALUES=(0.200 0.160 0.120 0.086 0.040 0.020 0.012 0.008)
readonly EXPECTED_RUNS=40

mkdir -p "${LOG_DIR}" "${PLOT_DIR}"
if [[ "${SUMMARIZE_ONLY:-0}" == "1" ]]; then
    if [[ ! -s "${METRICS_FILE}" ]]; then
        echo "SUMMARIZE_ONLY=1 requires an existing nonempty ${METRICS_FILE}" >&2
        exit 1
    fi
else
    : > "${METRICS_FILE}"
fi

controller_name() {
    case "$1" in
        0) echo "no_servo" ;;
        1) echo "servo_current" ;;
        *) echo "model_$1" ;;
    esac
}

record_log() {
    local log_file="$1"
    local model="$2"
    local tau="$3"
    local steps="$4"
    local process_status="$5"
    "${PYTHON_BIN}" - "${log_file}" "${model}" "${tau}" "${steps}" \
        "${process_status}" >> "${METRICS_FILE}" <<'PY'
import json
import pathlib
import sys

log_path, model, tau, steps, status = sys.argv[1:]
metrics = None
for line in pathlib.Path(log_path).read_text(errors="replace").splitlines():
    if line.startswith("METRICS_JSON="):
        metrics = json.loads(line.split("=", 1)[1])

metrics_present = metrics is not None
if metrics is None:
    metrics = {
        "scenario": "servo_delay_sweep",
        "model": int(model),
        "controller": {"0": "no_servo", "1": "servo_current_cost"}.get(model),
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
    "phase": "formal",
    "requested_model": int(model),
    "requested_tau_s": float(tau),
    "requested_erk_steps": int(steps),
    "requested_startup_mode": "cold",
    "process_exit_status": int(status),
    "metrics_present": metrics_present,
    "log_file": str(pathlib.Path(log_path).resolve()),
}
print(json.dumps(metrics, separators=(",", ":")))
PY
}

run_case() {
    local model="$1"
    local tau="$2"
    local steps="$3"
    local name
    name="$(controller_name "${model}")"
    local log_file="${LOG_DIR}/tau_${tau}_${name}_erk_${steps}.log"
    local plot_prefix="${PLOT_DIR}/tau_${tau}_${name}_erk_${steps}"

    echo "Running model=${model} (${name}), tau=${tau}, startup=cold, ERK=${steps}"
    : > "${plot_prefix}.png"
    : > "${plot_prefix}.pdf"
    set +e
    MPLBACKEND=Agg MPLCONFIGDIR="${TMPDIR:-/tmp}/nmpc_servo_model_formal_mpl" \
        "${PYTHON_BIN}" "${SCRIPT_DIR}/sim_nmpc.py" "${model}" \
        --scenario servo_delay_sweep \
        --servo-time-constant "${tau}" \
        --ocp-sim-num-steps "${steps}" \
        --startup-mode cold \
        --test-thrust-max 30 \
        --servo-angle-max-deg 90 \
        --plot_type 3 \
        --plot-output "${plot_prefix}" > "${log_file}" 2>&1
    local status=$?
    set -e

    record_log "${log_file}" "${model}" "${tau}" "${steps}" "${status}"
    if [[ ! -s "${plot_prefix}.png" || ! -s "${plot_prefix}.pdf" ]]; then
        echo "  Missing plot output for ${plot_prefix}" >&2
    fi
    if [[ ${status} -ne 0 ]]; then
        echo "  Process failed (exit=${status}); see ${log_file}"
    fi
}

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    for tau in "${TAU_VALUES[@]}"; do
        run_case 0 "${tau}" 1
        run_case 0 "${tau}" 20
        run_case 1 "${tau}" 1
        run_case 1 "${tau}" 5
        run_case 1 "${tau}" 20
    done

    png_count="$(find "${PLOT_DIR}" -maxdepth 1 -type f -name '*.png' -size +0c | wc -l)"
    pdf_count="$(find "${PLOT_DIR}" -maxdepth 1 -type f -name '*.pdf' -size +0c | wc -l)"
    if [[ "${png_count}" -ne "${EXPECTED_RUNS}" || "${pdf_count}" -ne "${EXPECTED_RUNS}" ]]; then
        echo "Expected ${EXPECTED_RUNS} PNG and PDF plots, found ${png_count} PNG and ${pdf_count} PDF." >&2
        exit 1
    fi
fi

"${PYTHON_BIN}" - "${METRICS_FILE}" "${RESULT_DIR}" "${EXPECTED_RUNS}" <<'PY'
import csv
import json
import pathlib
import sys

metrics_path = pathlib.Path(sys.argv[1])
result_dir = pathlib.Path(sys.argv[2])
expected_runs = int(sys.argv[3])
records = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]

if len(records) != expected_runs:
    raise SystemExit(f"Expected {expected_runs} formal records, found {len(records)} in {metrics_path}")

expected_taus = [0.200, 0.160, 0.120, 0.086, 0.040, 0.020, 0.012, 0.008]
groups = [
    (0, 1, "no_servo_erk1"),
    (0, 20, "no_servo_erk20"),
    (1, 1, "servo_current_erk1"),
    (1, 5, "servo_current_erk5"),
    (1, 20, "servo_current_erk20"),
]
expected_keys = {(tau, model, steps) for tau in expected_taus for model, steps, _ in groups}

lookup = {}
for record in records:
    experiment = record.get("experiment") or {}
    key = (
        float(experiment.get("requested_tau_s")),
        int(experiment.get("requested_model")),
        int(experiment.get("requested_erk_steps")),
    )
    if key in lookup:
        raise SystemExit(f"Duplicate formal record for tau/model/ERK={key}")
    lookup[key] = record

if set(lookup) != expected_keys:
    missing = sorted(expected_keys - set(lookup), reverse=True)
    extra = sorted(set(lookup) - expected_keys, reverse=True)
    raise SystemExit(f"Formal matrix mismatch; missing={missing}, extra={extra}")

def flatten(record):
    experiment = record.get("experiment") or {}
    timing = record.get("timing") or {}
    startup = record.get("startup") or {}
    required = ("servo_cmd_excess_travel_deg", "servo_actual_excess_travel_deg", "position_rmse_m")
    valid = (
        experiment.get("process_exit_status") == 0
        and experiment.get("metrics_present") is True
        and timing.get("solver_failures") == 0
        and all(startup.get(key) is not None for key in required)
    )
    return {
        "status": "ok" if valid else "failed",
        "process_exit_status": experiment.get("process_exit_status"),
        "solver_failures": timing.get("solver_failures"),
        "servo_cmd_excess_deg": startup.get("servo_cmd_excess_travel_deg") if valid else None,
        "servo_actual_excess_deg": startup.get("servo_actual_excess_travel_deg") if valid else None,
        "position_rmse_m": startup.get("position_rmse_m") if valid else None,
        "log_file": experiment.get("log_file"),
    }

rows = []
for tau in expected_taus:
    for model, steps, label in groups:
        values = flatten(lookup[(tau, model, steps)])
        rows.append({
            "tau_s": tau,
            "model": model,
            "controller": "No-servo" if model == 0 else "Servo-current",
            "erk_steps": steps,
            "group": label,
            **values,
        })

fieldnames = list(rows[0])
with (result_dir / "formal_tau_sweep.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

def cell(tau, model, steps):
    values = flatten(lookup[(tau, model, steps)])
    if values["status"] != "ok":
        return "N/A"
    return f'{values["servo_cmd_excess_deg"]:.2f} / {values["servo_actual_excess_deg"]:.2f}'

lines = [
    "# Formal servo-model/ERK sweep",
    "",
    "Cold-start metrics use the first 2 s after the nonzero position command.",
    "Each result is servo command/actual excess travel in degrees; any OCP failure is N/A.",
    "",
    "| $t_{servo}$ (s) | No-servo ERK 1 | No-servo ERK 20 | Servo-current ERK 1 | Servo-current ERK 5 | Servo-current ERK 20 |",
    "|---:|---:|---:|---:|---:|---:|",
]
for tau in expected_taus:
    lines.append(
        f"| {tau:.3f} | {cell(tau, 0, 1)} | {cell(tau, 0, 20)} | "
        f"{cell(tau, 1, 1)} | {cell(tau, 1, 5)} | {cell(tau, 1, 20)} |"
    )
(result_dir / "formal_tau_sweep.md").write_text("\n".join(lines) + "\n")

failed = [row for row in rows if row["status"] != "ok"]
print(f"Summarized {len(rows)} runs: {len(rows) - len(failed)} ok, {len(failed)} failed")
PY

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    {
        echo "Generated: $(date --iso-8601=seconds)"
        echo "Command: $0 $*"
        echo "Platform: Beetle-art"
        echo "TAU_VALUES: ${TAU_VALUES[*]}"
        echo "Groups: no-servo/ERK1 no-servo/ERK20 servo-current/ERK1 servo-current/ERK5 servo-current/ERK20"
        echo "Plots: ${PLOT_DIR} (${EXPECTED_RUNS} PNG and ${EXPECTED_RUNS} PDF)"
        echo "Startup: cold"
        echo "Bounds: thrust=0..30N servo=+/-90deg"
        echo "Analysis window: 2s"
    } > "${RESULT_DIR}/run_manifest.txt"
fi

echo "Formal experiment complete. Results: ${RESULT_DIR}"
