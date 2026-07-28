#!/usr/bin/env python3

import argparse
import os
import time

import numpy as np

from nmpc_tilt_mt.dragon import NMPCDragonGimbalServoDist, NMPCDragonGimbalServoThrust


def run_simulation(duration=4.0, scenario="hover", save_run=None):
    controller = NMPCDragonGimbalServoDist(build=False)
    plant = NMPCDragonGimbalServoThrust(build=False)

    ocp_solver = controller.get_ocp_solver()
    ts_control = controller.params["T_samp"]
    ts_sim = 0.0025
    sim_solver = plant.create_acados_sim_solver(ts_sim, build=True)

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    nx_plant = sim_solver.acados_sim.dims.nx
    reference_generator = controller.get_reference_generator()

    hover_thrust, hover_angles = reference_generator.allocate_wrench(
        np.array([0.0, 0.0, controller.phys.mass * controller.phys.gravity, 0.0, 0.0, 0.0])
    )
    state_plant = np.zeros(nx_plant)
    state_plant[6] = 1.0
    state_plant[13:21] = hover_angles
    state_plant[21:25] = hover_thrust
    command = np.concatenate((hover_thrust, hover_angles))

    state_controller = np.zeros(nx)
    state_controller[:21] = state_plant[:21]
    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", state_controller)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", command)

    control_stride = int(round(ts_control / ts_sim))
    steps = int(round(duration / ts_sim))
    state_history = np.zeros((steps + 1, nx_plant))
    input_history = np.zeros((steps, nu))
    solve_time = []
    solve_status = []
    state_history[0] = state_plant

    for step in range(steps):
        current_time = step * ts_sim
        if scenario == "hover":
            target_position = np.array([0.0, 0.0, 1.0])
            target_rpy = np.zeros(3)
        elif scenario == "step":
            target_position = np.array([0.0, 0.0, 1.0])
            target_rpy = np.zeros(3)
            if current_time >= duration / 2.0:
                target_position = np.array([0.3, -0.2, 1.2])
                target_rpy = np.deg2rad([5.0, -5.0, 10.0])
        else:
            raise ValueError(f"Unknown scenario: {scenario}")

        if step % control_stride == 0:
            state_controller.fill(0.0)
            state_controller[:21] = state_plant[:21]
            xr, ur = reference_generator.compute_trajectory(target_position, target_rpy)
            for stage in range(ocp_solver.N):
                ocp_solver.set(stage, "yref", np.concatenate((xr[stage], ur[stage])))
                controller.acados_init_p[:4] = xr[stage, 6:10]
                ocp_solver.set(stage, "p", controller.acados_init_p)
            ocp_solver.set(ocp_solver.N, "yref", xr[-1])
            controller.acados_init_p[:4] = xr[-1, 6:10]
            ocp_solver.set(ocp_solver.N, "p", controller.acados_init_p)

            start = time.perf_counter()
            command = ocp_solver.solve_for_x0(state_controller)
            solve_time.append(time.perf_counter() - start)
            solve_status.append(ocp_solver.status)
            if ocp_solver.status != 0:
                raise RuntimeError(f"acados returned status {ocp_solver.status} at t={current_time:.3f} s")

        sim_solver.set("x", state_plant)
        sim_solver.set("u", command)
        status = sim_solver.solve()
        if status != 0:
            raise RuntimeError(f"plant integrator returned status {status} at t={current_time:.3f} s")
        state_plant = sim_solver.get("x")
        state_plant[6:10] /= np.linalg.norm(state_plant[6:10])
        state_history[step + 1] = state_plant
        input_history[step] = command

    result = {
        "time": np.arange(steps + 1) * ts_sim,
        "state": state_history,
        "input": input_history,
        "solve_time": np.asarray(solve_time),
        "solve_status": np.asarray(solve_status),
        "scenario": np.array(scenario),
        "ts_control": np.array(ts_control),
        "ts_sim": np.array(ts_sim),
    }
    if save_run is not None:
        output = os.path.abspath(save_run)
        os.makedirs(os.path.dirname(output), exist_ok=True)
        np.savez(output, **result)

    final_position_error = np.linalg.norm(state_history[-1, :3] - target_position)
    print(f"final position error: {final_position_error:.6f} m")
    print(
        "solve time [ms]: "
        f"mean={1e3 * np.mean(solve_time):.3f}, "
        f"p99={1e3 * np.percentile(solve_time, 99):.3f}, "
        f"max={1e3 * np.max(solve_time):.3f}"
    )
    return result


def main():
    parser = argparse.ArgumentParser(description="Closed-loop simulation for the rigid-body DRAGON gimbal NMPC.")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--scenario", choices=("hover", "step"), default="hover")
    parser.add_argument("--save-run")
    args = parser.parse_args()
    run_simulation(duration=args.duration, scenario=args.scenario, save_run=args.save_run)


if __name__ == "__main__":
    main()
