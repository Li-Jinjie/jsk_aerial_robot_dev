"""
 Created by li-jinjie on 24-11-19.
"""
import copy
import time

import numpy as np
import argparse

from nmpc_viz import Visualizer, SensorVisualizer

from tilt_qd_servo_thrust_dist_imp import NMPCTiltQdServoThrustImpedance
from tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist, FIRDifferentiator

np.random.seed(42)

if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation for impedance control.")

    parser.add_argument("-e", "--est_dist_type", type=int, default=1,
                        help="The type of disturbance estimation. 0 means no, 1 means acc, 2 means mhe.")
    parser.add_argument("-b", "--if_use_ang_acc", type=int, default=1, help="Whether to use angular acceleration.")
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    args = parser.parse_args()

    # ========== init ==========
    # ---------- Controller ----------
    nmpc = NMPCTiltQdServoThrustImpedance()
    t_servo_ctrl = getattr(nmpc, "t_servo", 0.0)
    ts_ctrl = nmpc.ts_ctrl

    # ocp solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np  # np has been used for numpy

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # --------- Disturb. Rej. ---------
    ts_sensor = 0.0025
    disturb_estimated = np.zeros(6)  # f_d_i, tau_d_b. Note that they are in different frames.

    # ---------- Simulator ----------
    sim_nmpc = NMPCTiltQdServoThrustDist()
    t_servo_sim = getattr(sim_nmpc, "t_servo", 0.0)
    t_rotor_sim = getattr(sim_nmpc, "t_rotor", 0.0)

    ts_sim = 0.001

    disturb_init = np.zeros(6)

    t_total_sim = 8.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # sim solver
    sim_nmpc.get_ocp_model()
    sim_solver = sim_nmpc.create_acados_sim_solver(sim_nmpc.get_ocp_model(), ts_sim, True)
    nx_sim = sim_solver.acados_sim.dims.nx

    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw
    x_init_sim[-6:] = disturb_init

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()
    viz = Visualizer(N_sim, nx_sim, nu, x_init_sim, is_record_est_disturb=True)

    fir_param = [-0.5, 0, 0.5]  # central difference
    gyro_differentiator = [FIRDifferentiator(fir_param, ts_sensor), FIRDifferentiator(fir_param, ts_sensor),
                           FIRDifferentiator(fir_param, ts_sensor)]  # for gyro differentiation

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== update ==========
    u_cmd = u_init
    u_mpc = u_init
    t_ctl = 0.0
    t_sensor = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim
        t_sensor += ts_sim

        # --------- update state estimation ---------
        assert isinstance(nmpc, NMPCTiltQdServoThrustImpedance) or isinstance(nmpc, NMPCTiltQdServoThrustDist)
        x_now = np.zeros(nx)
        x_now[:nx - 6] = x_now_sim[:nx - 6]  # copy elements except the last 6 elements, which are the disturbance
        x_now[-6:] = disturb_estimated

        # -------- update control target --------
        target_xyz = np.array([[0.0, 0.5, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        # if t_total_sim > 2.0:
        #     if 2.0 <= t_now < 6:
        #         target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        #
        #         roll = 30.0 / 180.0 * np.pi
        #         pitch = 60.0 / 180.0 * np.pi
        #         yaw = 90.0 / 180.0 * np.pi
        #         target_rpy = np.array([[roll, pitch, yaw]]).T
        #
        #     if t_now >= 6:
        #         assert t_sqp_end <= 3.0
        #         target_xyz = np.array([[1.0, 1.0, 1.0]]).T
        #         target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        xr, ur = xr_ur_converter.pose_point_2_xr_ur(target_xyz, target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            ur[:, 4:] = 0.0

        # -------- sqp mode --------
        if is_sqp_change and t_sqp_start > t_sqp_end:
            if t_now >= t_sqp_start:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP"

            if t_now >= t_sqp_end:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- update solver --------
        comp_time_start = time.time()

        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]

                params = np.zeros(n_param)
                params[0:4] = quaternion_r
                if isinstance(nmpc, NMPCTiltQdServoThrustImpedance):
                    # for impedance control
                    W = nmpc.get_ocp_solver().acados_ocp.cost.W
                    # pMxy, pMxy, pMz, oMxy, oMxy, oMz
                    params[4:] = np.sqrt(np.array([W[21, 21], W[22, 22], W[23, 23], W[24, 24], W[25, 25], W[26, 26]]))

                ocp_solver.set(j, "p", params)  # for nonlinear quaternion error

            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]

            params = np.zeros(n_param)
            params[0:4] = quaternion_r
            if isinstance(nmpc, NMPCTiltQdServoThrustImpedance):
                # for impedance control
                W_e = nmpc.get_ocp_solver().acados_ocp.cost.W_e
                # pMxy, pMxy, pMz, oMxy, oMxy, oMz
                params[4:] = np.sqrt(
                    np.array([W_e[21, 21], W_e[22, 22], W_e[23, 23], W_e[24, 24], W_e[25, 25], W_e[26, 26]]))

            ocp_solver.set(ocp_solver.N, "p", params)  # for nonlinear quaternion error

            # feedback, take the first action
            try:
                u_mpc = ocp_solver.solve_for_x0(x_now)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        # by default, the u_cmd is the mpc command
        u_cmd = copy.deepcopy(u_mpc)

        # disturb est. is related to the sensor update frequency
        if t_sensor >= ts_sensor and args.est_dist_type != 0:
            t_sensor = 0.0

            # wrench_u_imu_b
            # - the wrench calculated from imu info
            sf_b, ang_acc_b, rot_ib = sim_nmpc.fake_sensor.update_acc(x_now_sim)

            w = x_now_sim[10:13]
            mass = sim_nmpc.fake_sensor.mass
            gravity = sim_nmpc.fake_sensor.gravity
            iv = sim_nmpc.fake_sensor.iv

            sf_b_imu = sf_b + np.random.normal(0.0, 0.1, 3)  # add noise. real: scale = 0.00727 * gravity
            w_imu = w + np.random.normal(0.0, 0.001, 3)  # add noise. real: scale = 0.0008 rad/s

            ang_acc_b_imu = np.zeros(3)
            if args.if_use_ang_acc == 0:
                ang_acc_b_imu[0] = gyro_differentiator[0].apply_single(w_imu[0])
                ang_acc_b_imu[1] = gyro_differentiator[1].apply_single(w_imu[1])
                ang_acc_b_imu[2] = gyro_differentiator[2].apply_single(w_imu[2])
            else:
                ang_acc_b_imu = ang_acc_b

            wrench_u_imu_b = np.zeros(6)
            wrench_u_imu_b[0:3] = mass * sf_b_imu
            wrench_u_imu_b[3:6] = np.dot(iv, ang_acc_b_imu) + np.cross(w, np.dot(iv, w))

            # wrench_u_sensor_b
            # - the wrench calculated from actuator sensor
            ft_sensor = x_now_sim[17:21]
            a_sensor = x_now_sim[13:17]

            z_sensor = np.zeros(8)
            z_sensor[0] = ft_sensor[0] * np.sin(a_sensor[0])
            z_sensor[1] = ft_sensor[0] * np.cos(a_sensor[0])
            z_sensor[2] = ft_sensor[1] * np.sin(a_sensor[1])
            z_sensor[3] = ft_sensor[1] * np.cos(a_sensor[1])
            z_sensor[4] = ft_sensor[2] * np.sin(a_sensor[2])
            z_sensor[5] = ft_sensor[2] * np.cos(a_sensor[2])
            z_sensor[6] = ft_sensor[3] * np.sin(a_sensor[3])
            z_sensor[7] = ft_sensor[3] * np.cos(a_sensor[3])

            wrench_u_sensor_b = np.dot(xr_ur_converter.alloc_mat, z_sensor)

            u_meas = np.zeros(8)
            u_meas[0:4] = ft_sensor
            u_meas[4:] = a_sensor

            # wrench_u_mpc_b
            # the wrench calculated from mpc command. the current command or the shifted command
            ft_mpc = u_mpc[0:4]
            a_mpc = u_mpc[4:]

            z_mpc = np.zeros(8)
            z_mpc[0] = ft_mpc[0] * np.sin(a_mpc[0])
            z_mpc[1] = ft_mpc[0] * np.cos(a_mpc[0])
            z_mpc[2] = ft_mpc[1] * np.sin(a_mpc[1])
            z_mpc[3] = ft_mpc[1] * np.cos(a_mpc[1])
            z_mpc[4] = ft_mpc[2] * np.sin(a_mpc[2])
            z_mpc[5] = ft_mpc[2] * np.cos(a_mpc[2])
            z_mpc[6] = ft_mpc[3] * np.sin(a_mpc[3])
            z_mpc[7] = ft_mpc[3] * np.cos(a_mpc[3])

            wrench_u_mpc_b = np.dot(xr_ur_converter.alloc_mat, z_mpc)

            # update disturbance estimation
            # # only use the wrench difference between the imu and the actuator sensor, no u_mpc
            if args.est_dist_type == 1:
                alpha = 0.01
                disturb_estimated[0:3] = (1 - alpha) * disturb_estimated[0:3] + alpha * np.dot(rot_ib, (
                        wrench_u_imu_b[0:3] - wrench_u_sensor_b[0:3]))  # world frame
                disturb_estimated[3:6] = (1 - alpha) * disturb_estimated[3:6] + alpha * (
                        wrench_u_imu_b[3:6] - wrench_u_sensor_b[3:6])  # body frame

        # --------- update simulation ----------
        disturb = copy.deepcopy(disturb_init)

        # random disturbance
        # disturb[2] = np.random.normal(1.0, 3.0)  # N, fz

        if 2.0 <= t_now < 3.0:
            disturb[0] = 3.0
            disturb[1] = -5.0
            disturb[2] = -2.0

        if 5.0 <= t_now < 6.0:
            disturb[3] = 0.3
            disturb[4] = -0.5
            disturb[5] = 0.2

        x_now_sim[-6:] = disturb

        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # note that the recording frequency of u_cmd is the same as ts_sim
        viz.update_est_disturb(i, disturb_estimated[0:3], disturb_estimated[3:6])

    # ========== visualize ==========
    if args.plot_type == 0:
        viz.visualize_w_disturb(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                                t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 1:
        viz.visualize_less(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                           t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                          t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
