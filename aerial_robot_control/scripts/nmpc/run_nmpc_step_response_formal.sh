#!/usr/bin/env bash
# Formal 36-case six-DoF step-response experiment for servo-model NMPC.
#
# Fixed experiment configuration:
#   controller:       model 1, NMPCTiltQdServo (Beetle-art parameters)
#   plant:            sim_model 0, NMPCTiltQdServoThrust
#   control period:   0.010 s (100 Hz, read from BeetleNMPCFull.yaml)
#   plant step:       0.001 s
#   velocity bounds:  -5..+5 m/s (read from BeetleNMPCFull.yaml)
#   step time:        2.0 s
#   duration:         10.0 s
#   steady window:    9.0..10.0 s
#   repetitions:      one deterministic run per condition
#   matrix:           18 zero-workpoint pose steps and 18 position steps at
#                     [30,0,0] and [30,30,0] deg attitude workpoints
#
# Statistics reported in English:
#   - Rise Time (10--90%), Settling Time (2% band), Percentage Overshoot,
#     Steady-State Error, Root Mean Square Error (RMSE), and Integral of
#     Absolute Error (IAE).
#   - Maximum Cross-Axis Deviation, Cross-Axis RMSE, and Geodesic Attitude
#     Error based on the SO(3) logarithmic map.
#   - Peak Control Input, RMS Control Effort, Control-Input Rate / Slew Rate,
#     and Saturation-Time Ratio.
#   - Minimum Constraint Margin, Constraint Violation Count, Mean/P95/Worst-
#     Case Solution Time, Control-Deadline Misses, and Solver Failure Count.
#
# Formal results recorded on 2026-07-22 with +/-5 m/s velocity bounds (36/36
# runs completed and all 36 workpoints passed the pre-step stability test):
#
# | Axis  | Runs | Mean rise [s] | Mean settling [s] | Mean overshoot [%] | Mean RMSE |
# |:------|-----:|--------------:|------------------:|-------------------:|----------:|
# | x     |    9 |        0.6900 |            1.7013 |             2.7176 | 0.1383 m  |
# | y     |    9 |        0.6896 |            1.7132 |             2.6199 | 0.1390 m  |
# | z     |    9 |        0.5546 |            1.4038 |             3.4940 | 0.1212 m  |
# | roll  |    3 |        0.9398 |            1.9680 |             0.0001 | 6.5246 deg|
# | pitch |    3 |        0.9190 |            1.9930 |             0.0007 | 6.5445 deg|
# | yaw   |    3 |        0.5512 |            0.8960 |             1.4579 | 6.7284 deg|
#
# Position-step results by attitude workpoint (nine runs per row):
#
# | Workpoint R/P/Y [deg] | Mean rise [s] | Mean settling [s] | Mean overshoot [%] | Mean RMSE [m] | Max attitude deviation [deg] |
# |:----------------------|--------------:|------------------:|-------------------:|--------------:|-----------------------------:|
# | 0 / 0 / 0             |        0.6472 |            1.5722 |             2.9319 |        0.1337 |                      21.3140 |
# | 30 / 0 / 0            |        0.6392 |            1.6270 |             3.0373 |        0.1325 |                      15.5730 |
# | 30 / 30 / 0           |        0.6478 |            1.6191 |             2.8623 |        0.1324 |                      16.7719 |
#
# Constraint and real-time results:
#   - All 36 runs completed with zero OCP solver failures, zero input-bound
#     violations, zero plant-thrust-state violations, and no acados deadline
#     misses. Mean acados time_tot was 0.2388 ms and the worst case was
#     0.6150 ms, versus the 10 ms control period.
#   - No state bound was violated. Peak absolute linear speeds were
#     [1.5824, 1.5749, 1.8124] m/s, well below the new 5 m/s bound. Peak
#     absolute body rates were [2.4686, 2.6919, 1.9247] rad/s, also below the
#     6 rad/s bound. Maximum thrust and servo commands were 19.9122 N and
#     1.3954 rad, below their 30 N and pi limits.
#   - The wrapper wall-clock measurement recorded 35 >10 ms host-side
#     outliers, while every solver-reported acados time remained below 1 ms.
#
# Main observations:
#   - Removing the active +/-1 m/s bound shortened mean 1.0 m rise time from
#     0.9148 to 0.6773 s for x, from 0.9121 to 0.6824 s for y, and from 0.8418
#     to 0.5552 s for z. This confirms that the old velocity bound materially
#     limited the large-position-step response.
#   - The faster response increased mean position overshoot and cross-axis
#     attitude motion. At the zero-attitude workpoint, maximum attitude
#     deviation increased from 14.2138 to 21.3140 deg. The speed improvement
#     therefore comes with a stronger rotational transient.
#   - The nonzero attitude workpoints still did not degrade mean position RMSE
#     relative to the zero-attitude workpoint. Mean position RMSE remained
#     between 0.1324 and 0.1337 m.
#   - Yaw was the fastest attitude channel. Roll and pitch had essentially no
#     overshoot but slower approximately 2 s settling times.
#   - All maneuvers were feasible under the recorded state and actuator bounds;
#     no tested response approached the new linear-velocity limit.
#
# Recorded result directory:
#   experiment_results/nmpc_step_response_20260722_170709
# Previous +/-1 m/s comparison directory:
#   experiment_results/nmpc_step_response_20260722_155749
#
# Generated results (all beneath one unique result directory):
#   runs/*.npz                    structured time-series bundles
#   logs/*.log                    complete stdout/stderr for every case
#   round_times/*.csv             per-control-update solver timing
#   plot_type_3/*.{png,pdf}       original plot_type==3 style figures
#   metrics.jsonl                 one English metric record per valid run
#   manifest.json                 status and paths for all 36 cases
#   analysis/step_response_metrics.{csv,json}
#   analysis/step_response_summary.md
#   analysis/cases/*.{png,pdf} and aggregate metric figures
#
# Usage:
#   ./run_nmpc_step_response_formal.sh
#   ./run_nmpc_step_response_formal.sh /absolute/new/output/directory
#   SUMMARIZE_ONLY=1 ./run_nmpc_step_response_formal.sh /existing/output/directory

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/nmpc_step_response_${TIMESTAMP}}"
RESULT_DIR="$(realpath -m -- "${RESULT_DIR}")"
RUN_DIR="${RESULT_DIR}/runs"
LOG_DIR="${RESULT_DIR}/logs"
TIMING_DIR="${RESULT_DIR}/round_times"
PLOT_DIR="${RESULT_DIR}/plot_type_3"
ANALYSIS_DIR="${RESULT_DIR}/analysis"
SCHEDULE_FILE="${RESULT_DIR}/run_schedule.tsv"
STATUS_FILE="${RESULT_DIR}/run_status.tsv"
METRICS_FILE="${RESULT_DIR}/metrics.jsonl"
MANIFEST_FILE="${RESULT_DIR}/manifest.json"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MPL_CACHE_DIR="${MPLCONFIGDIR:-${TMPDIR:-/tmp}/nmpc_step_response_mpl}"
cd "${SCRIPT_DIR}"

if [[ "${SUMMARIZE_ONLY:-0}" != "1" && -e "${RESULT_DIR}" ]]; then
    echo "Refusing to overwrite existing result directory: ${RESULT_DIR}" >&2
    exit 1
fi
mkdir -p "${RUN_DIR}" "${LOG_DIR}" "${TIMING_DIR}" "${PLOT_DIR}" "${MPL_CACHE_DIR}"

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    "${PYTHON_BIN}" - "${SCHEDULE_FILE}" <<'PY'
import csv
import pathlib
import sys

from nmpc_tilt_mt.utils.step_response_experiment import base36_cases

path = pathlib.Path(sys.argv[1])
with path.open("w", newline="") as stream:
    writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
    writer.writerow(("order", "case_id", "axis", "amplitude", "unit", "roll_deg", "pitch_deg", "yaw_deg"))
    for order, case in enumerate(base36_cases(), start=1):
        writer.writerow((order, case.slug, case.axis, case.amplitude, case.amplitude_unit, *case.workpoint_rpy_deg))
PY

    printf 'order\tcase_id\tprocess_status\trun_path\tlog_path\n' > "${STATUS_FILE}"
    : > "${METRICS_FILE}"
    while IFS=$'\t' read -r order case_id axis amplitude unit roll pitch yaw; do
        [[ "${order}" == "order" ]] && continue
        run_path="${RUN_DIR}/${case_id}.npz"
        log_path="${LOG_DIR}/${case_id}.log"
        timing_path="${TIMING_DIR}/${case_id}.csv"
        plot_prefix="${PLOT_DIR}/${case_id}"
        build_args=()
        if (( order > 1 )); then
            build_args+=(--no-build)
        fi

        echo "[${order}/36] axis=${axis}, amplitude=${amplitude} ${unit}, workpoint=[${roll},${pitch},${yaw}] deg"
        set +e
        MPLBACKEND=Agg MPLCONFIGDIR="${MPL_CACHE_DIR}" \
            "${PYTHON_BIN}" "${SCRIPT_DIR}/sim_nmpc.py" 1 \
            --sim_model 0 \
            --scenario step_response \
            --step-axis "${axis}" \
            --step-amplitude "${amplitude}" \
            --workpoint-rpy-deg "${roll}" "${pitch}" "${yaw}" \
            --plot_type 3 \
            --plot-output "${plot_prefix}" \
            --save-run "${run_path}" \
            --solve-time-csv "${timing_path}" \
            "${build_args[@]}" > "${log_path}" 2>&1
        status=$?
        set -e

        metrics_line="$(sed -n 's/^METRICS_JSON=//p' "${log_path}" | tail -n 1)"
        if [[ -n "${metrics_line}" ]]; then
            printf '%s\n' "${metrics_line}" >> "${METRICS_FILE}"
        fi
        printf '%s\t%s\t%s\t%s\t%s\n' \
            "${order}" "${case_id}" "${status}" "${run_path}" "${log_path}" >> "${STATUS_FILE}"
        if [[ ${status} -ne 0 ]]; then
            echo "  FAILED (exit=${status}); see ${log_path}" >&2
        fi
    done < "${SCHEDULE_FILE}"
fi

"${PYTHON_BIN}" - "${SCHEDULE_FILE}" "${STATUS_FILE}" "${MANIFEST_FILE}" <<'PY'
import csv
import json
import pathlib
import sys

schedule_path, status_path, manifest_path = map(pathlib.Path, sys.argv[1:])
with schedule_path.open() as stream:
    schedule = {row["case_id"]: row for row in csv.DictReader(stream, delimiter="\t")}
with status_path.open() as stream:
    statuses = list(csv.DictReader(stream, delimiter="\t"))
cases = []
for status in statuses:
    requested = schedule[status["case_id"]]
    run_path = pathlib.Path(status["run_path"])
    returncode = int(status["process_status"])
    cases.append({
        "case_id": status["case_id"],
        "axis": requested["axis"],
        "amplitude": float(requested["amplitude"]),
        "amplitude_unit": requested["unit"],
        "workpoint_rpy_deg": [float(requested[name]) for name in ("roll_deg", "pitch_deg", "yaw_deg")],
        "run": str(run_path.resolve()) if run_path.exists() else None,
        "log": str(pathlib.Path(status["log_path"]).resolve()),
        "returncode": returncode,
        "success": returncode == 0 and run_path.exists(),
    })
manifest = {
    "scenario": "step_response",
    "suite": "base36",
    "controller_model": 1,
    "sim_model": 0,
    "cases": cases,
}
manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

if [[ "${SUMMARIZE_ONLY:-0}" == "1" && -s "${ANALYSIS_DIR}/step_response_metrics.json" ]]; then
    echo "Reusing existing detailed analysis in ${ANALYSIS_DIR}"
else
    if [[ -e "${ANALYSIS_DIR}" && -n "$(find "${ANALYSIS_DIR}" -mindepth 1 -print -quit 2>/dev/null)" ]]; then
        echo "Refusing to overwrite non-empty analysis directory: ${ANALYSIS_DIR}" >&2
        exit 1
    fi
    MPLBACKEND=Agg MPLCONFIGDIR="${MPL_CACHE_DIR}" \
        "${PYTHON_BIN}" "${SCRIPT_DIR}/plot_nmpc_step_response_results.py" \
        --manifest "${MANIFEST_FILE}" --output-dir "${ANALYSIS_DIR}"
fi

"${PYTHON_BIN}" - "${ANALYSIS_DIR}/step_response_metrics.json" \
    "${ANALYSIS_DIR}/step_response_summary.md" <<'PY'
import json
import math
import pathlib
import statistics
import sys

records = json.loads(pathlib.Path(sys.argv[1]).read_text())
output = pathlib.Path(sys.argv[2])
valid = [record for record in records if record["completed"] and record["pre_step_stable"]]

def finite_values(path):
    values = []
    for record in valid:
        value = record
        for key in path:
            value = value[key]
        if isinstance(value, (int, float)) and math.isfinite(value):
            values.append(float(value))
    return values

fields = [
    (("tracking", "rise_time_s"), "Rise Time", "s"),
    (("tracking", "settling_time_s"), "Settling Time", "s"),
    (("tracking", "percentage_overshoot_pct"), "Percentage Overshoot", "%"),
    (("tracking", "steady_state_error_abs"), "Absolute Steady-State Error", "axis unit"),
    (("tracking", "rmse"), "RMSE", "axis unit"),
    (("tracking", "iae"), "IAE", "axis unit s"),
    (("timing", "mean_solution_time_s"), "Mean Solution Time", "s"),
    (("timing", "worst_case_solution_time_s"), "Worst-Case Solution Time", "s"),
]
lines = [
    "# NMPC step-response summary",
    "",
    f"Completed metric bundles: {len(records)}/36; valid pre-step-stable runs: {len(valid)}/36.",
    "",
    "| Statistic | Mean | Median | Maximum | Unit |",
    "|:--|--:|--:|--:|:--|",
]
for path, label, unit in fields:
    values = finite_values(path)
    if values:
        lines.append(
            f"| {label} | {statistics.fmean(values):.6g} | "
            f"{statistics.median(values):.6g} | {max(values):.6g} | {unit} |"
        )
deadline_misses = sum(record["timing"]["acados_deadline_miss_count"] for record in valid)
solver_failures = sum(record["timing"]["solver_failure_count"] for record in records)
state_violations = sum(record["constraints"]["state"]["violation_count"] for record in valid)
input_violations = sum(record["constraints"]["input"]["violation_count"] for record in valid)
lines.extend([
    "",
    f"- Acados control-deadline misses: {deadline_misses}",
    f"- Solver failures: {solver_failures}",
    f"- State constraint violations: {state_violations}",
    f"- Input constraint violations: {input_violations}",
    "",
    "Position and attitude RMSE values retain their commanded-axis units; use the detailed CSV for axis-wise paper tables.",
])
output.write_text("\n".join(lines) + "\n")
PY

successful_runs="$(${PYTHON_BIN} -c 'import json,sys; print(sum(c["success"] for c in json.load(open(sys.argv[1]))["cases"]))' "${MANIFEST_FILE}")"
echo "Completed ${successful_runs}/36 runs. Results: ${RESULT_DIR}"
[[ "${successful_runs}" == "36" ]]
