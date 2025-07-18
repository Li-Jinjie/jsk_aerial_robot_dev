import unittest
import numpy as np
import sys
import os

sys.path.append("../scripts/nmpc/")
from sim_nmpc import main

current_path = os.path.abspath(os.path.dirname(__file__))


class SimArgs:
    def __init__(self):
        self.model = 0
        self.sim_model = 0
        self.plot_type = 1
        self.arch = "qd"
        self.no_viz = True
        self.save_data = False
        self.file_path = "../../../../test/data/"


def print_error(x, x_true, u, u_true):
    print(f"max error: {np.max(np.abs(x - x_true))}, {np.max(np.abs(u - u_true))}")
    print(f"mean error: {np.mean(np.abs(x - x_true))}, {np.mean(np.abs(u - u_true))}")
    print(f"accumulated error: {np.sum(np.abs(x - x_true))}, {np.sum(np.abs(u - u_true))}")


def run_simulation_test(npz_filename, nmpc_model_id, sim_model_id, atol, plot_type=1, arch="qd", no_viz=True):
    """
    Run simulation and compare against reference data.

    Args:
      npz_filename (str): Path to the npz file containing reference 'x' and 'u' data.
      nmpc_model_id (int): Model id for the NMPC controller.
      sim_model_id (int): Model id for the simulator.
      atol (float): Absolute tolerance for np.allclose.
      plot_type (int): Plot type to use in simulation.
      arch (str): Architecture to use in simulation, 'qd' or 'tri' or 'bi.
      no_viz (bool): Disable visualization if True.

    Raises:
      AssertionError: If the simulated data does not match the reference data within tolerance.
    """
    os.chdir(current_path)
    npzfile = np.load(npz_filename)
    x_true, u_true = npzfile["x"], npzfile["u"]

    sim_args = SimArgs()
    sim_args.model = nmpc_model_id
    sim_args.sim_model = sim_model_id
    sim_args.plot_type = plot_type
    sim_args.arch = arch
    sim_args.no_viz = no_viz

    x, u = main(sim_args)
    print_error(x, x_true, u, u_true)
    assert np.allclose(x, x_true, atol=atol) and np.allclose(u, u_true, atol=atol), "Test failed!"


class TestTiltQd(unittest.TestCase):
    def test_tilt_qd_no_servo(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdNoServo_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=0,
            sim_model_id=0,
            atol=1e-1,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_servo(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdServo_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=1,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_thrust(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdThrust_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=2,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_servo_thrust(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdServoThrust_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=3,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_servo_dist(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdServoDist_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=21,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_servo_thrust_dist(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdServoThrustDist_sim_NMPCTiltQdServoThrust.npz",
            nmpc_model_id=22,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )

    def test_tilt_qd_servo_thrust_drag_sim_model(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltQdServo_sim_NMPCTiltQdServoThrustDrag.npz",
            nmpc_model_id=1,
            sim_model_id=1,
            atol=1e-3,
            plot_type=1,
            no_viz=True,
        )


class TestTiltTri(unittest.TestCase):
    def test_tilt_tri_servo(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltTriServo_sim_NMPCTiltTriServo.npz",
            nmpc_model_id=0,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            arch="tri",
            no_viz=True,
        )


class TestTiltBi(unittest.TestCase):
    def test_tilt_bi_servo(self):
        run_simulation_test(
            npz_filename="data/nmpc_NMPCTiltBiServo_sim_NMPCTiltBiServo.npz",
            nmpc_model_id=0,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            arch="bi",
            no_viz=True,
        )


if __name__ == "__main__":
    unittest.main()
