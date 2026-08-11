#!/usr/bin/env bash
set -euo pipefail

# Reproducible NMPC-vs-geometric experiment at realistic actuator limits.
# Fixed settings:
#   thrust command:       0 .. 23 N per rotor
#   servo command/state: -pi .. +pi rad
#   task:                 default constraint_sweep (25/20/0 deg attitude)
#   simulator:            servo + thrust first-order dynamics (sim_model=0)
#   controller models:    1 = servo-model NMPC, 4 = geometric + pinv
#
# Reference results generated on 2026-07-18:
# RESULTS_BEGIN
# Experiment 1: real-aircraft limits at the nominal t_servo = 0.086 s
#
# | Metric                    | Servo-model NMPC | Geometric + pinv |
# |---------------------------|-----------------:|-----------------:|
# | Position RMSE (m)         |           0.8482 |           0.8116 |
# | Attitude RMSE (deg)       |          14.3892 |           3.3740 |
# | Thrust-active time (%)    |           1.1667 |           1.0833 |
# | Servo-active time (%)     |           0.0000 |           0.0000 |
# | Servo lag RMS (deg)       |           3.8058 |          11.6644 |
# | Servo rate P95 (deg/s)    |         101.3617 |         310.9383 |
#
# Conclusion: at 23 N and +/-pi, actuator constraints are almost inactive. The
# geometric baseline has 4.3% lower position RMSE and 76.5% lower attitude RMSE;
# NMPC does not have a tracking advantage in this test. NMPC does produce 67.4%
# lower servo command-to-state lag, showing the effect of modeling servo delay.
#
# Experiment 2: matched controller/simulator servo time-constant sweep
#
# | t_servo | NMPC pos. | Geom. pos. | G/N pos. | NMPC att. | Geom. att. | G/N att. |
# |   (s)   | RMSE (m)  | RMSE (m)   |  ratio   | RMSE(deg) | RMSE (deg) |  ratio   |
# |--------:|----------:|-----------:|---------:|----------:|-----------:|---------:|
# |   0.200 |    0.8635 |     1.0702 |     1.24 |    17.738 |     48.928 |     2.76 |
# |   0.180 |    0.8621 |     1.8434 |     2.14 |    17.423 |     46.683 |     2.68 |
# |   0.160 |    0.8603 |     0.8321 |     0.97 |    17.054 |      8.640 |     0.51 |
# |   0.120 |    0.8555 |     0.8181 |     0.96 |    16.028 |      5.391 |     0.34 |
# |   0.086 |    0.8482 |     0.8116 |     0.96 |    14.389 |      3.374 |     0.23 |
# |   0.040 |    0.8273 |     0.8070 |     0.98 |     9.579 |      1.726 |     0.18 |
# |   0.020 |    0.8156 |     0.8066 |     0.99 |     7.399 |      1.692 |     0.23 |
# |   0.008 |    0.8143 |     0.8069 |     0.99 |     6.677 |      1.825 |     0.27 |
#
# | t_servo | Controller | Thrust active | Servo active | Servo lag | Servo-rate P95 |
# |   (s)   |            |      (%)      |     (%)      | RMS (deg) |    (deg/s)     |
# |--------:|:-----------|---------------:|--------------:|----------:|---------------:|
# |   0.200 | NMPC       |           3.50 |          0.00 |     3.101 |          35.27 |
# |   0.200 | Geometric  |          13.67 |          7.17 |    31.258 |         347.96 |
# |   0.180 | NMPC       |           3.25 |          0.00 |     3.201 |          40.56 |
# |   0.180 | Geometric  |          13.51 |         13.76 |    35.406 |         430.04 |
# |   0.160 | NMPC       |           3.08 |          0.00 |     3.305 |          47.12 |
# |   0.160 | Geometric  |           1.33 |          0.00 |    21.876 |         314.63 |
# |   0.120 | NMPC       |           2.58 |          0.00 |     3.532 |          69.10 |
# |   0.120 | Geometric  |           1.08 |          0.00 |    15.666 |         312.88 |
# |   0.086 | NMPC       |           1.17 |          0.00 |     3.806 |         101.36 |
# |   0.086 | Geometric  |           1.08 |          0.00 |    11.664 |         310.94 |
# |   0.040 | NMPC       |           0.58 |          0.00 |     3.662 |         177.51 |
# |   0.040 | Geometric  |           1.00 |          0.00 |     6.657 |         288.44 |
# |   0.020 | NMPC       |           0.42 |          0.00 |     2.697 |         217.50 |
# |   0.020 | Geometric  |           1.00 |          0.00 |     4.296 |         259.49 |
# |   0.008 | NMPC       |           0.42 |          0.00 |     1.766 |         249.79 |
# |   0.008 | Geometric  |           1.00 |          0.00 |     2.501 |         224.79 |
#
# Conclusion: the crossover occurs between 0.16 and 0.18 s. At 0.16 s and below,
# geometric still has lower pose RMSE. At 0.18 s, geometric becomes unstable in
# the final high-amplitude segment: NMPC reduces position RMSE by 53.2% and
# attitude RMSE by 62.7%. At 0.20 s, NMPC reduces position RMSE by 19.3% and
# attitude RMSE by 63.7%. NMPC servo lag remains near 3 deg while geometric lag
# rises above 31 deg. Thus delay-aware prediction becomes a clear advantage only
# for sufficiently slow servos in this task; for the real 0.086 s servo, it does
# not overcome the current controller tuning difference.
# RESULTS_END
#
# Usage:
#   ./run_servo_time_sweep.sh
#   ./run_servo_time_sweep.sh /absolute/output/directory
#   SERVO_TIME_CONSTANTS="0.20 0.18" ./run_servo_time_sweep.sh /tmp/subset
#
# Outputs:
#   one complete .log per run, metrics.jsonl, summary.csv, summary.md

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RESULT_DIR="${1:-${SCRIPT_DIR}/experiment_results/servo_time_sweep_real_limits}"
MPL_CACHE_DIR="${MPLCONFIGDIR:-/tmp/matplotlib-nmpc-servo-sweep}"

mkdir -p "${RESULT_DIR}" "${MPL_CACHE_DIR}"
METRICS_FILE="${RESULT_DIR}/metrics.jsonl"
: > "${METRICS_FILE}"

if [[ -n "${SERVO_TIME_CONSTANTS:-}" ]]; then
    read -r -a servo_time_constants <<< "${SERVO_TIME_CONSTANTS}"
else
    servo_time_constants=(0.200 0.180 0.160 0.120 0.086 0.040 0.020 0.008)
fi
models=(1 4)

for servo_time in "${servo_time_constants[@]}"; do
    for model in "${models[@]}"; do
        if [[ "${model}" == "1" ]]; then
            controller_name="nmpc"
        else
            controller_name="geometric"
        fi

        log_file="${RESULT_DIR}/tau_${servo_time}_${controller_name}.log"
        echo "Running model=${model} (${controller_name}), servo_time_constant=${servo_time} s"
        MPLCONFIGDIR="${MPL_CACHE_DIR}" python3 "${SCRIPT_DIR}/sim_nmpc.py" "${model}" \
            --scenario constraint_sweep \
            --no_viz \
            --test-thrust-max 23 \
            --servo-angle-max-deg 180 \
            --servo-time-constant "${servo_time}" | tee "${log_file}"

        metrics_line="$(sed -n 's/^METRICS_JSON=//p' "${log_file}" | tail -n 1)"
        if [[ -z "${metrics_line}" ]]; then
            echo "No METRICS_JSON line found in ${log_file}" >&2
            exit 1
        fi
        printf '%s\n' "${metrics_line}" >> "${METRICS_FILE}"
    done
done

python3 - "${METRICS_FILE}" "${RESULT_DIR}/summary.csv" "${RESULT_DIR}/summary.md" <<'PY'
import csv
import json
import sys
from pathlib import Path

metrics_path = Path(sys.argv[1])
csv_path = Path(sys.argv[2])
markdown_path = Path(sys.argv[3])

records = [json.loads(line) for line in metrics_path.read_text().splitlines() if line.strip()]
records.sort(key=lambda record: (-record["parameters"]["servo_time_constant_s"], record["model"]))

rows = []
for record in records:
    overall = record["overall"]
    rows.append(
        {
            "servo_time_constant_s": record["parameters"]["servo_time_constant_s"],
            "controller": "NMPC" if record["model"] == 1 else "Geometric",
            "position_rmse_m": overall["position_rmse_m"],
            "attitude_rmse_deg": overall["attitude_rmse_deg"],
            "thrust_active_time_pct": overall["thrust_active_time_pct"],
            "servo_active_time_pct": overall["servo_active_time_pct"],
            "servo_lag_rms_deg": overall["servo_lag_rms_deg"],
            "servo_rate_p95_deg_s": overall["servo_rate_p95_deg_s"],
            "solver_failures": overall["solver_failures"],
        }
    )

with csv_path.open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

paired = {}
for row in rows:
    paired.setdefault(row["servo_time_constant_s"], {})[row["controller"]] = row

lines = [
    "# Servo time-constant sweep at ±pi and 23 N",
    "",
    "| t_servo (s) | NMPC pos. RMSE (m) | Geom. pos. RMSE (m) | Pos. ratio G/N | "
    "NMPC att. RMSE (deg) | Geom. att. RMSE (deg) | Att. ratio G/N |",
    "|---:|---:|---:|---:|---:|---:|---:|",
]
for servo_time in sorted(paired, reverse=True):
    nmpc = paired[servo_time]["NMPC"]
    geom = paired[servo_time]["Geometric"]
    lines.append(
        f"| {servo_time:.3f} | {nmpc['position_rmse_m']:.4f} | {geom['position_rmse_m']:.4f} | "
        f"{geom['position_rmse_m'] / nmpc['position_rmse_m']:.2f} | "
        f"{nmpc['attitude_rmse_deg']:.3f} | {geom['attitude_rmse_deg']:.3f} | "
        f"{geom['attitude_rmse_deg'] / nmpc['attitude_rmse_deg']:.2f} |"
    )

lines.extend(
    [
        "",
        "| t_servo (s) | Controller | Thrust active (%) | Servo active (%) | "
        "Servo lag RMS (deg) | Servo rate P95 (deg/s) | Failures |",
        "|---:|:---|---:|---:|---:|---:|---:|",
    ]
)
for row in rows:
    lines.append(
        f"| {row['servo_time_constant_s']:.3f} | {row['controller']} | "
        f"{row['thrust_active_time_pct']:.2f} | {row['servo_active_time_pct']:.2f} | "
        f"{row['servo_lag_rms_deg']:.3f} | {row['servo_rate_p95_deg_s']:.2f} | "
        f"{row['solver_failures']} |"
    )

markdown_path.write_text("\n".join(lines) + "\n")
print(f"Wrote {csv_path}")
print(f"Wrote {markdown_path}")
PY

echo "Experiment complete. Results: ${RESULT_DIR}"
