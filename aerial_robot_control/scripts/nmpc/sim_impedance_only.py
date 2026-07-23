import copy
import os
import time
import numpy as np
import argparse

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer

from nmpc_tilt_mt.utils.fir_differentiator import FIRDifferentiator
from nmpc_tilt_mt.utils.force_impedance_experiment import (
    SCENARIO_DURATION,
    SCENARIO_NAME,
    default_run_bundle_path,
    get_force_comparison_wrench,
    impedance_parameters,
    save_run_bundle,
)

from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist_imp import NMPCTiltQdServoImpedance
from nmpc_tilt_mt.misc.nominal_impedance import NominalImpedance

np.random.seed(42)


def main(args):
    if args.save_run is not None:
        args.save_run = os.path.abspath(args.save_run)

    if args.scenario == SCENARIO_NAME and args.sim_model != 1:
        raise ValueError("The force comparison truth run requires --sim_model 1.")

    # ========== Init ==========
    # ---------- Simulator ----------
    if args.sim_model == 0:
        sim_nmpc = NMPCTiltQdServoThrustDist()
    elif args.sim_model == 1:
        if args.scenario == SCENARIO_NAME:
            sim_nmpc = NominalImpedance(config_file="BeetleNMPCFullServoForceImp.yaml", force_only=True)
        else:
            sim_nmpc = NominalImpedance()

    if args.scenario == SCENARIO_NAME and args.save_run is None:
        args.save_run = default_run_bundle_path("nominal", sim_nmpc.params)

    # Get time constants
    if sim_nmpc.include_servo_model:
        t_servo_sim = sim_nmpc.phys.t_servo
    else:
        t_servo_sim = 0.0
    if sim_nmpc.include_thrust_model:
        t_rotor_sim = sim_nmpc.phys.t_rotor
    else:
        t_rotor_sim = 0.0

    ts_sim = 0.005  # or 0.001

    if args.scenario == SCENARIO_NAME:
        t_total_sim = SCENARIO_DURATION
    else:
        t_total_sim = 40.0
        if args.plot_type == 1:
            t_total_sim = 4.0
        if args.plot_type == 2:
            t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # Disturbance Initialization
    disturb_init = np.zeros(6)

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Visualization ----------
    viz = Visualizer(
        "",
        N_sim,
        nx_sim,
        0,
        x_init_sim,
        tilt=False,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model,
        include_cog_dist_est=True,
    )

    # ========== Run simulation ==========
    t_ctl = 0.0
    t_sensor = 0.0
    x_now_sim = x_init_sim
    applied_wrench_ee_all = np.zeros((N_sim, 6))
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim
        t_sensor += ts_sim

        # --------- Update disturbance ---------
        if args.scenario == SCENARIO_NAME:
            disturb = get_force_comparison_wrench(t_now)
        else:
            disturb = copy.deepcopy(disturb_init)
            # Simulate fixed disturbance at singular points
            if 2.0 <= t_now < 7.0:
                disturb[0] = 5.0
            if 7.0 <= t_now < 12.0:
                disturb[0:2] = [5.0, -5.0]
            if 12.0 <= t_now < 17.0:
                disturb[0:3] = [5.0, -5.0, -5.0]
            if 20.0 <= t_now < 25.0:
                disturb[3] = 5.0
            if 25.0 <= t_now < 30.0:
                disturb[3:5] = [5.0, -5.0]
            if 30.0 <= t_now < 35.0:
                disturb[3:6] = [5.0, -5.0, 5.0]

        applied_wrench_ee_all[i, :] = disturb

        # --------- Update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", disturb)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- Update visualizer ----------
        viz.update(i, x_now_sim, 0)  # Note: The recording frequency of u_cmd is the same as ts_sim
        viz.update_est_disturb(i, disturb[0:3], disturb[3:6])

    if args.save_run is not None:
        metadata = {
            "kind": "truth",
            "scenario": args.scenario,
            "model": "ideal_second_order_force_impedance",
            "sim_model": sim_solver.model_name,
            "ts_sim": ts_sim,
            "interaction_frame": "ee",
            "plot_state_frame": "ee",
            "scenario_duration": SCENARIO_DURATION,
            "enlarge_factor": sim_nmpc.params.get("enlarge_factor"),
            "impedance": impedance_parameters(sim_nmpc.params),
        }
        save_run_bundle(
            args.save_run,
            metadata,
            time_state=np.arange(viz.data_idx + 1) * ts_sim,
            time_input=np.arange(viz.data_idx) * ts_sim,
            state_raw=viz.x_sim_all[: viz.data_idx + 1, :],
            state_ee=viz.x_sim_all[: viz.data_idx + 1, :13],
            applied_wrench_ee=applied_wrench_ee_all[: viz.data_idx, :],
            estimated_force_w=applied_wrench_ee_all[: viz.data_idx, 0:3],
            torque_compensation_b=np.zeros((viz.data_idx, 3)),
            control=applied_wrench_ee_all[: viz.data_idx, :],
        )

    # ========== Visualize ==========
    if args.plot_type == 0:
        viz.visualize(
            "no_mdl", sim_solver.model_name, 0.0, ts_sim, t_total_sim, t_servo_ctrl=0.0, t_servo_sim=t_servo_sim
        )
    elif args.plot_type == 1:
        viz.visualize_less(ts_sim, t_total_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy("no_mdl", ts_sim, t_total_sim)
    elif args.plot_type == 3:
        viz.visualize_disturb(ts_sim, t_total_sim)
    elif args.plot_type == 4:
        viz.visualize_nothing_but_save("no_mdl", sim_solver.model_name)


if __name__ == "__main__":
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models with impedance control.")

    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. " "Options: 0 (default: servo+thrust+dist), 1 (pure impedance).",
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. " "Options: 0 (default: full), 1 (less), 2 (only rpy).",
    )

    parser.add_argument(
        "--scenario",
        choices=("default", SCENARIO_NAME),
        default="default",
        help="Disturbance scenario. The force comparison scenario is a 20 s force-only experiment.",
    )

    parser.add_argument(
        "--save-run",
        type=str,
        default=None,
        help="Structured NPZ path. The comparison scenario defaults to the organized paper data directory.",
    )

    args = parser.parse_args()
    main(args)
