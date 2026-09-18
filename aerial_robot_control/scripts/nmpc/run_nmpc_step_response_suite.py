#!/usr/bin/env python3
"""Run a selected NMPC step-response matrix through sim_nmpc.py."""

import argparse
import datetime
import json
import os
import subprocess
import sys

from nmpc_tilt_mt.utils.step_response_experiment import base30_cases, base90_cases


CASE_FACTORIES = {
    "base30": base30_cases,
    "base90": base90_cases,
}


def main(args):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.abspath(args.output_dir or os.path.join("sim_data", "step_response", timestamp))
    if os.path.exists(output_dir):
        raise FileExistsError(f"Refusing to overwrite existing output directory: {output_dir}")
    os.makedirs(output_dir)
    run_dir = os.path.join(output_dir, "runs")
    log_dir = os.path.join(output_dir, "logs")
    timing_dir = os.path.join(output_dir, "round_times")
    os.makedirs(run_dir)
    os.makedirs(log_dir)
    os.makedirs(timing_dir)

    script = os.path.abspath(os.path.join(os.path.dirname(__file__), "sim_nmpc.py"))
    manifest = {
        "scenario": "step_response",
        "suite": args.suite,
        "output_dir": output_dir,
        "controller_model": 1,
        "sim_model": 0,
        "cases": [],
    }
    cases = CASE_FACTORIES[args.suite]()
    for index, case in enumerate(cases):
        run_path = os.path.join(run_dir, case.slug + ".npz")
        log_path = os.path.join(log_dir, case.slug + ".log")
        timing_path = os.path.join(timing_dir, case.slug + ".csv")
        command = [
            sys.executable,
            script,
            "1",
            "--sim_model",
            "0",
            "--scenario",
            "step_response",
            "--step-axis",
            case.axis,
            "--step-amplitude",
            str(case.amplitude),
            "--workpoint-rpy-deg",
            *[str(value) for value in case.workpoint_rpy_deg],
            "--save-run",
            run_path,
            "--solve-time-csv",
            timing_path,
            "--no_viz",
        ]
        if index > 0 or args.no_build:
            command.append("--no-build")
        print(f"[{index + 1:02d}/{len(cases)}] {case.slug}", flush=True)
        with open(log_path, "w") as log_stream:
            result = subprocess.run(command, stdout=log_stream, stderr=subprocess.STDOUT, text=True, check=False)
        entry = {
            "case_id": case.slug,
            "axis": case.axis,
            "amplitude": case.amplitude,
            "amplitude_unit": case.amplitude_unit,
            "workpoint_rpy_deg": list(case.workpoint_rpy_deg),
            "run": run_path if os.path.exists(run_path) else None,
            "log": log_path,
            "returncode": result.returncode,
            "success": result.returncode == 0 and os.path.exists(run_path),
        }
        manifest["cases"].append(entry)
        with open(os.path.join(output_dir, "manifest.json"), "w") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)

    successes = sum(case["success"] for case in manifest["cases"])
    print(f"Completed {successes}/{len(cases)} cases. Manifest: {os.path.join(output_dir, 'manifest.json')}")
    return 0 if successes == len(cases) else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None, help="New output directory; defaults to a timestamped path.")
    parser.add_argument(
        "--suite",
        choices=tuple(CASE_FACTORIES),
        default="base90",
        help="Step-response case matrix to run.",
    )
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Assume acados code is already built, including for the first case.",
    )
    raise SystemExit(main(parser.parse_args()))
