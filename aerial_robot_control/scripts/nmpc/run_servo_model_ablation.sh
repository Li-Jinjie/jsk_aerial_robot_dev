#!/usr/bin/env bash
# Reproducible servo-model ablation for sim_nmpc.py.
#
# Questions answered:
#   1. Does model=0 approach servo-aware NMPC as t_servo becomes small?
#   2. At which t_servo does ignoring servo dynamics cause actuator oscillation?
#   3. Is the initial oscillation caused by servo mismatch or by a cold start?
#   4. How many ERK substeps are required at t_servo=0.008 s?
#
# Controllers:
#   model=0   no servo state, absolute servo-command cost
#   model=92  servo state, old absolute servo-command cost
#   model=1   servo state, current (a_cmd-a_servo) cost
#
# Default experiment matrix:
#   ERK convergence: t_servo=0.008, models 1/92, steps 1/2/3/5/10/20
#   Main sweep:       t_servo=0.008/0.012/0.020/0.040/0.086/0.120/0.160/0.200
#                     models 0/92/1, cold start, both bound profiles
#   Startup ablation: t_servo=0.008/0.086/0.200, models 0/92/1,
#                     historical bounds, cold versus 2 s hover warm-up
#
# Bound profiles:
#   historical: 0..30 N, +/-90 deg
#   realistic:  0..23 N, +/-180 deg
#
# Adaptive ERK rule:
#   n_base = max(1, ceil(T_step / (2.5*t_servo))), T_step=0.1 s.
#   The ERK convergence phase selects 5, 10, or 20 steps at t_servo=0.008 s.
#   Main-sweep steps are n_base multiplied by selected_steps/5.
#
# Full results recorded on 2026-07-18 (historical bounds, cold-start window):
#
# | t_servo | No-servo cmd/actual excess | Servo-old cmd/actual excess | Servo-current cmd/actual excess |
# |   [s]   |            [deg]           |             [deg]           |               [deg]             |
# |  0.008  |       4477.88 / 2585.17    |        2539.63 / 1464.85    |            62.06 / 61.69         |
# |  0.012  |       3225.91 / 1383.70    |        2597.49 / 1091.41    |            57.70 / 57.02         |
# |  0.020  |       2944.39 /  828.51    |        2685.01 /  700.61    |            48.43 / 46.83         |
# |  0.040  |       2941.60 /  462.47    |        2136.35 /  300.24    |            36.55 / 33.01         |
# |  0.086  |       2325.34 /  276.43    |        2084.86 /  155.17    |            29.11 / 22.02         |
# |  0.120  |       2007.71 /  236.31    |        1605.95 /  106.53    |            29.83 / 16.87         |
# |  0.160  |       4418.18 /  291.32    |         826.41 /   67.40    |            37.41 / 14.22         |
# |  0.200  |       4572.31 /  275.69 *  |         240.06 /   49.28    |            42.67 / 12.07         |
#
# * model 0 had one OCP failure at t_servo=0.200 s. Under realistic 23 N/+/-180 deg
#   bounds it also failed at 0.008, 0.040, 0.160, and 0.200 s; models 1 and 92
#   completed every main-sweep run without solver failures.
#
# ERK result: model 1 required 10 steps for the 2% convergence criterion. Model
# 92 still changed substantially from 10 to 20 steps, so the sweep conservatively
# used a safety factor of 4 (20 steps at t_servo=0.008 s).
#
# Main interpretation:
#   - The no-servo controller remains strongly oscillatory even at 0.008 s.
#   - Adding the servo state while retaining the old absolute-command cost reduces
#     vibration, but does not reproduce the smooth current controller at small tau.
#   - The (a_cmd-a_servo) cost in model 1 is therefore a major part of the observed
#     improvement; the experiment does not support attributing it to the servo model alone.
#   - Hover warm-up reduces some transients but does not eliminate the gap, so cold
#     initialization is not the sole cause.
#
# Generated results:
#   metrics.jsonl, erk_convergence.{csv,md}, tau_sweep.{csv,md},
#   startup_ablation.{csv,md}, run_manifest.txt, and one log per run.
#
# Full run:
#   ./run_servo_model_ablation.sh
#   ./run_servo_model_ablation.sh /absolute/output/directory
#
# Fast smoke/subset run:
#   TAU_VALUES="0.008 0.086" STARTUP_TAU_VALUES="0.086" \
#     ERK_TEST_STEPS="5 10" CONTROLLERS="0 92 1" \
#     ./run_servo_model_ablation.sh /tmp/servo_ablation_subset

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/servo_model_ablation}"
LOG_DIR="${RESULT_DIR}/logs"
METRICS_FILE="${RESULT_DIR}/metrics.jsonl"
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "${LOG_DIR}"
if [[ "${SUMMARIZE_ONLY:-0}" == "1" ]]; then
    if [[ ! -s "${METRICS_FILE}" ]]; then
        echo "SUMMARIZE_ONLY=1 requires an existing nonempty ${METRICS_FILE}" >&2
        exit 1
    fi
else
    : > "${METRICS_FILE}"
fi

read -r -a tau_values <<< "${TAU_VALUES:-0.008 0.012 0.020 0.040 0.086 0.120 0.160 0.200}"
read -r -a startup_tau_values <<< "${STARTUP_TAU_VALUES:-0.008 0.086 0.200}"
read -r -a controllers <<< "${CONTROLLERS:-0 92 1}"
read -r -a erk_models <<< "${ERK_MODELS:-1 92}"
read -r -a erk_test_steps <<< "${ERK_TEST_STEPS:-1 2 3 5 10 20}"

controller_name() {
    case "$1" in
        0) echo "no_servo" ;;
        1) echo "servo_current_cost" ;;
        92) echo "servo_old_cost" ;;
        *) echo "model_$1" ;;
    esac
}

record_log() {
    local log_file="$1"
    local phase="$2"
    local bounds_profile="$3"
    local model="$4"
    local tau="$5"
    local steps="$6"
    local startup_mode="$7"
    local process_status="$8"
    "${PYTHON_BIN}" - "${log_file}" "${phase}" "${bounds_profile}" "${model}" \
        "${tau}" "${steps}" "${startup_mode}" "${process_status}" >> "${METRICS_FILE}" <<'PY'
import json
import pathlib
import sys

log_path, phase, profile, model, tau, steps, startup, status = sys.argv[1:]
metrics = None
for line in pathlib.Path(log_path).read_text(errors="replace").splitlines():
    if line.startswith("METRICS_JSON="):
        metrics = json.loads(line.split("=", 1)[1])
if metrics is None:
    metrics = {
        "scenario": "servo_delay_sweep",
        "model": int(model),
        "controller": {"0": "no_servo", "1": "servo_current_cost", "92": "servo_old_cost"}.get(model),
        "parameters": {
            "servo_time_constant_s": float(tau),
            "ocp_sim_method_num_steps": int(steps),
            "startup_mode": startup,
        },
        "startup": None,
        "overall": None,
        "timing": {"solver_failures": 1},
    }
metrics["experiment"] = {
    "phase": phase,
    "bounds_profile": profile,
    "requested_model": int(model),
    "requested_tau_s": float(tau),
    "requested_erk_steps": int(steps),
    "requested_startup_mode": startup,
    "process_exit_status": int(status),
    "log_file": str(pathlib.Path(log_path).resolve()),
}
print(json.dumps(metrics, separators=(",", ":")))
PY
}

run_case() {
    local phase="$1"
    local bounds_profile="$2"
    local thrust_max="$3"
    local angle_max="$4"
    local model="$5"
    local tau="$6"
    local steps="$7"
    local startup_mode="$8"
    local name
    name="$(controller_name "${model}")"
    local log_file="${LOG_DIR}/${phase}_${bounds_profile}_tau_${tau}_${name}_${startup_mode}_erk_${steps}.log"

    echo "Running phase=${phase}, bounds=${bounds_profile}, model=${model}, tau=${tau}, startup=${startup_mode}, ERK=${steps}"
    set +e
    MPLCONFIGDIR="${TMPDIR:-/tmp}/nmpc_servo_model_ablation_mpl" \
        "${PYTHON_BIN}" "${SCRIPT_DIR}/sim_nmpc.py" "${model}" \
        --scenario servo_delay_sweep \
        --servo-time-constant "${tau}" \
        --ocp-sim-num-steps "${steps}" \
        --startup-mode "${startup_mode}" \
        --test-thrust-max "${thrust_max}" \
        --servo-angle-max-deg "${angle_max}" \
        --no_viz > "${log_file}" 2>&1
    local status=$?
    set -e
    record_log "${log_file}" "${phase}" "${bounds_profile}" "${model}" \
        "${tau}" "${steps}" "${startup_mode}" "${status}"
    if [[ ${status} -ne 0 ]]; then
        echo "  FAILED (exit=${status}); see ${log_file}"
    fi
}

adaptive_base_steps() {
    "${PYTHON_BIN}" - "$1" <<'PY'
import math
import sys
tau = float(sys.argv[1])
print(max(1, math.ceil(0.1 / (2.5 * tau))))
PY
}

if [[ "${SUMMARIZE_ONLY:-0}" != "1" && "${SKIP_ERK:-0}" != "1" ]]; then
    for model in "${erk_models[@]}"; do
        for steps in "${erk_test_steps[@]}"; do
            run_case "erk" "historical" 30 90 "${model}" 0.008 "${steps}" cold
        done
    done
fi

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
"${PYTHON_BIN}" - "${METRICS_FILE}" "${RESULT_DIR}" "${ERK_SAFETY_FACTOR:-}" <<'PY'
import csv
import json
import math
import pathlib
import sys

metrics_path = pathlib.Path(sys.argv[1])
result_dir = pathlib.Path(sys.argv[2])
factor_override = sys.argv[3]
records = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]
records = [r for r in records if r["experiment"]["phase"] == "erk"]
keys = (
    "position_rmse_m",
    "attitude_rmse_deg",
    "servo_cmd_rms_deg",
    "servo_cmd_excess_travel_deg",
    "servo_actual_excess_travel_deg",
)

def relative_difference(a, b):
    return abs(a - b) / max(abs(b), 1e-9)

lookup = {(r["model"], r["parameters"]["ocp_sim_method_num_steps"]): r for r in records}
rows = []
selected = {}
for model in (1, 92):
    for steps in (1, 2, 3, 5, 10, 20):
        record = lookup.get((model, steps))
        startup = record.get("startup") if record else None
        valid = bool(
            startup
            and record["experiment"]["process_exit_status"] == 0
            and (record.get("timing") or {}).get("solver_failures", 1) == 0
        )
        row = {
            "model": model,
            "controller": {1: "servo_current_cost", 92: "servo_old_cost"}[model],
            "erk_steps": steps,
            "status": "ok" if valid else "failed",
        }
        for key in keys:
            row[key] = startup.get(key) if valid else None
        finer = lookup.get((model, steps * 2))
        finer_valid = bool(
            finer
            and finer.get("startup")
            and finer["experiment"]["process_exit_status"] == 0
            and (finer.get("timing") or {}).get("solver_failures", 1) == 0
        )
        if valid and finer_valid:
            deviations = [relative_difference(startup[k], finer["startup"][k]) for k in keys]
            row["max_relative_change_to_next"] = max(deviations)
        else:
            row["max_relative_change_to_next"] = None
        rows.append(row)

    choice = 20
    for candidate in (5, 10):
        row = next(r for r in rows if r["model"] == model and r["erk_steps"] == candidate)
        if row["max_relative_change_to_next"] is not None and row["max_relative_change_to_next"] <= 0.02:
            choice = candidate
            break
    selected[model] = choice

if factor_override:
    factor = int(factor_override)
elif records:
    factor = max(1, math.ceil(max(selected.values()) / 5))
else:
    factor = 1

fieldnames = ["model", "controller", "erk_steps", "status", *keys, "max_relative_change_to_next"]
with (result_dir / "erk_convergence.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

lines = [
    "# ERK convergence at $t_{servo}=0.008$ s",
    "",
    "| Model | Controller | Steps | Status | Pos. RMSE (m) | Att. RMSE (deg) | Servo cmd excess (deg) | Servo actual excess (deg) | Max change to next |",
    "|---:|---|---:|---|---:|---:|---:|---:|---:|",
]
for row in rows:
    def fmt(key, digits=4):
        value = row[key]
        return "-" if value is None else f"{value:.{digits}f}"
    lines.append(
        f"| {row['model']} | {row['controller']} | {row['erk_steps']} | {row['status']} | "
        f"{fmt('position_rmse_m')} | {fmt('attitude_rmse_deg')} | "
        f"{fmt('servo_cmd_excess_travel_deg', 2)} | {fmt('servo_actual_excess_travel_deg', 2)} | "
        f"{fmt('max_relative_change_to_next', 3)} |"
    )
lines.extend([
    "",
    f"Selected steps: model 1 = {selected.get(1, 20)}, model 92 = {selected.get(92, 20)}.",
    f"Main-sweep ERK safety factor: **{factor}**.",
])
(result_dir / "erk_convergence.md").write_text("\n".join(lines) + "\n")
(result_dir / "erk_selection.env").write_text(f"ERK_SAFETY_FACTOR={factor}\n")
PY

# shellcheck disable=SC1090
source "${RESULT_DIR}/erk_selection.env"

for profile in historical realistic; do
    if [[ "${profile}" == "historical" ]]; then
        thrust_max=30
        angle_max=90
    else
        thrust_max=23
        angle_max=180
    fi
    for tau in "${tau_values[@]}"; do
        base_steps="$(adaptive_base_steps "${tau}")"
        steps=$((base_steps * ERK_SAFETY_FACTOR))
        for model in "${controllers[@]}"; do
            run_case "main" "${profile}" "${thrust_max}" "${angle_max}" \
                "${model}" "${tau}" "${steps}" cold
        done
    done
done

for tau in "${startup_tau_values[@]}"; do
    base_steps="$(adaptive_base_steps "${tau}")"
    steps=$((base_steps * ERK_SAFETY_FACTOR))
    for model in "${controllers[@]}"; do
        run_case "startup" "historical" 30 90 "${model}" "${tau}" "${steps}" hover_warm
    done
done
else
    # shellcheck disable=SC1090
    source "${RESULT_DIR}/erk_selection.env"
fi

"${PYTHON_BIN}" - "${METRICS_FILE}" "${RESULT_DIR}" <<'PY'
import csv
import json
import pathlib
import sys

metrics_path = pathlib.Path(sys.argv[1])
result_dir = pathlib.Path(sys.argv[2])
records = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]
names = {0: "No-servo", 1: "Servo/current cost", 92: "Servo/old cost"}

def flatten(record):
    startup = record.get("startup") or {}
    overall = record.get("overall") or {}
    timing = record.get("timing") or {}
    experiment = record["experiment"]
    parameters = record.get("parameters", {})
    valid = experiment["process_exit_status"] == 0 and timing.get("solver_failures", 1) == 0
    return {
        "phase": experiment["phase"],
        "bounds_profile": experiment["bounds_profile"],
        "tau_s": experiment["requested_tau_s"],
        "model": record["model"],
        "controller": names.get(record["model"], str(record["model"])),
        "startup_mode": experiment["requested_startup_mode"],
        "erk_steps": experiment["requested_erk_steps"],
        "process_status": experiment["process_exit_status"],
        "solver_failures": timing.get("solver_failures"),
        "status": "ok" if valid else "failed",
        "thrust_max_n": (parameters.get("thrust_bounds_n") or [None, None])[1],
        "servo_max_deg": (parameters.get("servo_bounds_deg") or [None, None])[1],
        "position_rmse_m": startup.get("position_rmse_m"),
        "attitude_rmse_deg": startup.get("attitude_rmse_deg"),
        "servo_cmd_excess_deg": startup.get("servo_cmd_excess_travel_deg"),
        "servo_actual_excess_deg": startup.get("servo_actual_excess_travel_deg"),
        "servo_cmd_increment_p95_deg": startup.get("servo_cmd_increment_p95_deg"),
        "servo_rate_p95_deg_s": startup.get("servo_rate_p95_deg_s"),
        "thrust_cmd_excess_n": startup.get("thrust_cmd_excess_travel_n"),
        "overall_position_rmse_m": overall.get("position_rmse_m"),
        "overall_attitude_rmse_deg": overall.get("attitude_rmse_deg"),
        "solve_time_p95_ms": timing.get("solve_time_p95_ms"),
    }

main_rows = [flatten(r) for r in records if r["experiment"]["phase"] == "main"]
main_rows.sort(key=lambda r: (r["bounds_profile"], r["tau_s"], r["model"]))
fieldnames = list(main_rows[0]) if main_rows else []
if fieldnames:
    with (result_dir / "tau_sweep.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(main_rows)

def fmt(value, digits=3):
    return "-" if value is None else f"{value:.{digits}f}"

def classify(no_servo, comparator):
    if comparator["status"] != "ok":
        return "controller failed"
    if no_servo["status"] != "ok":
        return "model-0 solver failure"
    needed = ("servo_cmd_excess_deg", "servo_actual_excess_deg", "position_rmse_m")
    if any(no_servo[k] is None or comparator[k] is None for k in needed):
        return "failed"
    clear = (
        comparator["servo_cmd_excess_deg"] <= 0.7 * no_servo["servo_cmd_excess_deg"]
        and comparator["servo_actual_excess_deg"] <= 0.7 * no_servo["servo_actual_excess_deg"]
        and comparator["position_rmse_m"] <= 1.1 * no_servo["position_rmse_m"]
    )
    if clear:
        return "servo-model advantage"
    relative = [
        abs(comparator[k] - no_servo[k]) / max(abs(no_servo[k]), 1e-9)
        for k in needed
    ]
    return "equivalent" if max(relative) < 0.1 else "mixed"

lines = [
    "# Servo time-constant sweep",
    "",
    "Startup metrics use the first 2 s after the nonzero position command.",
    "",
    "| Bounds | $t_{servo}$ (s) | Model | ERK | Status | Pos. RMSE (m) | Att. RMSE (deg) | Servo cmd excess (deg) | Servo actual excess (deg) | Servo rate P95 (deg/s) | Classification vs model 0 |",
    "|---|---:|---|---:|---|---:|---:|---:|---:|---:|---|",
]
groups = {}
for row in main_rows:
    groups.setdefault((row["bounds_profile"], row["tau_s"]), {})[row["model"]] = row
for row in main_rows:
    group = groups[(row["bounds_profile"], row["tau_s"])]
    if row["model"] == 0:
        classification = "reference"
    elif 0 not in group:
        classification = "no model-0 baseline"
    else:
        classification = classify(group[0], row)
    lines.append(
        f"| {row['bounds_profile']} | {row['tau_s']:.3f} | {row['controller']} | {row['erk_steps']} | "
        f"{row['status']} | {fmt(row['position_rmse_m'], 4)} | {fmt(row['attitude_rmse_deg'])} | "
        f"{fmt(row['servo_cmd_excess_deg'], 2)} | {fmt(row['servo_actual_excess_deg'], 2)} | "
        f"{fmt(row['servo_rate_p95_deg_s'], 2)} | {classification} |"
    )
(result_dir / "tau_sweep.md").write_text("\n".join(lines) + "\n")

# Startup table reuses historical cold runs from the main phase.
startup_rows = [
    row for row in main_rows
    if row["bounds_profile"] == "historical"
]
startup_rows.extend(flatten(r) for r in records if r["experiment"]["phase"] == "startup")
requested_warm_taus = {
    r["experiment"]["requested_tau_s"]
    for r in records if r["experiment"]["phase"] == "startup"
}
startup_rows = [row for row in startup_rows if row["tau_s"] in requested_warm_taus]
startup_rows.sort(key=lambda r: (r["tau_s"], r["model"], r["startup_mode"]))
if startup_rows:
    with (result_dir / "startup_ablation.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(startup_rows[0]))
        writer.writeheader()
        writer.writerows(startup_rows)

lines = [
    "# Cold-start versus hover-warm startup",
    "",
    "| $t_{servo}$ (s) | Model | Startup | Status | Pos. RMSE (m) | Servo cmd excess (deg) | Servo actual excess (deg) | Servo rate P95 (deg/s) |",
    "|---:|---|---|---|---:|---:|---:|---:|",
]
for row in startup_rows:
    lines.append(
        f"| {row['tau_s']:.3f} | {row['controller']} | {row['startup_mode']} | "
        f"{row['status']} | {fmt(row['position_rmse_m'], 4)} | {fmt(row['servo_cmd_excess_deg'], 2)} | "
        f"{fmt(row['servo_actual_excess_deg'], 2)} | {fmt(row['servo_rate_p95_deg_s'], 2)} |"
    )
(result_dir / "startup_ablation.md").write_text("\n".join(lines) + "\n")
PY

if [[ "${SUMMARIZE_ONLY:-0}" != "1" ]]; then
    {
        echo "Generated: $(date --iso-8601=seconds)"
        echo "Command: $0 $*"
        echo "TAU_VALUES: ${tau_values[*]}"
        echo "STARTUP_TAU_VALUES: ${startup_tau_values[*]}"
        echo "CONTROLLERS: ${controllers[*]}"
        echo "ERK_TEST_STEPS: ${erk_test_steps[*]}"
        echo "ERK_SAFETY_FACTOR: ${ERK_SAFETY_FACTOR}"
    } > "${RESULT_DIR}/run_manifest.txt"
fi

echo "Experiment complete. Results: ${RESULT_DIR}"
