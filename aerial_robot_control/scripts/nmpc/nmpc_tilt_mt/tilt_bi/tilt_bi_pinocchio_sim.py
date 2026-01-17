#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Pinocchio-based multi-body simulator for tiltable birotor
This module provides a high-fidelity simulation using Pinocchio's multi-body dynamics
as an alternative to the Acados numerical integrator.
"""

import os
import numpy as np
import pinocchio as pin
import subprocess
import tempfile
from typing import Tuple


class TiltBirotorPinocchioSimulator:
    """
    Multi-body simulator for tiltable birotor using Pinocchio.

    State vector (nx=15):
        [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, a1, a2]

    Control vector (nu=4):
        [ft1c, ft2c, a1c, a2c]
    """

    def __init__(self, urdf_path: str, dt: float = 0.005):
        """
        Initialize the Pinocchio simulator.

        Args:
            urdf_path: Path to the URDF file (can be .xacro)
            dt: Integration timestep
        """
        self.dt = dt
        self.nx = 15  # State dimension
        self.nu = 4  # Control dimension

        # Convert xacro to URDF if needed
        self.urdf_path = self._process_urdf(urdf_path)

        # Load robot model
        package_dirs = [self._get_package_dir()]
        self.model = pin.buildModelFromUrdf(self.urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        # Get joint indices for gimbal joints
        self.gimbal1_id = self.model.getJointId("gimbal1")
        self.gimbal2_id = self.model.getJointId("gimbal2")

        # Get frame indices for thrust application points
        self.thrust1_frame_id = self.model.getFrameId("thrust1")
        self.thrust2_frame_id = self.model.getFrameId("thrust2")

        # Physical parameters (from phys_param_birotor.py)
        self.mass = 1.44441
        self.gravity = 9.798
        self.kq_d_kt = 0.0172
        self.t_servo = 0.15858

        # Rotor directions and positions
        self.dr1 = 1
        self.dr2 = -1
        self.p1_b = np.array([0.0, 0.2375, 0.12098])
        self.p2_b = np.array([0.0, -0.2375, 0.12098])

        # State storage
        self.q = None  # Generalized positions (7 for floating base + 2 for joints)
        self.v = None  # Generalized velocities (6 for floating base + 2 for joints)
        self.x_state = None  # NMPC state format
        self.u_control = None  # NMPC control input

        print(f"[Pinocchio Sim] Loaded robot model with {self.model.nq} generalized positions")
        print(f"[Pinocchio Sim] and {self.model.nv} generalized velocities")
        print(f"[Pinocchio Sim] Gimbal joint IDs: {self.gimbal1_id}, {self.gimbal2_id}")

    def _get_package_dir(self) -> str:
        """Get the ROS package directory for gimbalrotor."""
        # Assume we're in jsk_aerial_robot_dev workspace
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Navigate up to workspace root and find robots/gimbalrotor
        workspace_root = current_dir
        for _ in range(10):  # Search up to 10 levels
            if os.path.exists(os.path.join(workspace_root, "robots", "gimbalrotor")):
                return os.path.join(workspace_root, "robots")
            workspace_root = os.path.dirname(workspace_root)

        raise RuntimeError("Could not find gimbalrotor package directory")

    def _process_urdf(self, urdf_path: str) -> str:
        """
        Process URDF file. If it's a xacro file, convert it to URDF.

        Args:
            urdf_path: Path to URDF or xacro file

        Returns:
            Path to processed URDF file
        """
        if urdf_path.endswith(".xacro"):
            # Create temporary URDF file
            temp_urdf = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
            temp_urdf_path = temp_urdf.name
            temp_urdf.close()

            # Convert xacro to URDF using xacro command
            try:
                subprocess.run(["xacro", urdf_path, "-o", temp_urdf_path], check=True, capture_output=True)
                print(f"[Pinocchio Sim] Converted xacro to URDF: {temp_urdf_path}")
                return temp_urdf_path
            except subprocess.CalledProcessError as e:
                print(f"[Pinocchio Sim] Error converting xacro: {e.stderr.decode()}")
                raise
        else:
            return urdf_path

    def _nmpc_state_to_pinocchio(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert NMPC state to Pinocchio generalized coordinates.

        NMPC state (15): [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, a1, a2]
        Pinocchio q (9): [px, py, pz, qx, qy, qz, qw, a1, a2]
        Pinocchio v (8): [vx, vy, vz, wx, wy, wz, da1, da2]

        Args:
            x: NMPC state vector

        Returns:
            (q, v): Pinocchio generalized positions and velocities
        """
        q = np.zeros(self.model.nq)
        v = np.zeros(self.model.nv)

        # Floating base position
        q[0:3] = x[0:3]  # px, py, pz

        # Floating base orientation (quaternion)
        # NMPC uses [qw, qx, qy, qz], Pinocchio uses [qx, qy, qz, qw]
        q[3] = x[7]  # qx
        q[4] = x[8]  # qy
        q[5] = x[9]  # qz
        q[6] = x[6]  # qw

        # Joint positions (gimbal angles)
        q[7] = x[13]  # a1
        q[8] = x[14]  # a2

        # Floating base linear velocity
        v[0:3] = x[3:6]  # vx, vy, vz

        # Floating base angular velocity
        v[3:6] = x[10:13]  # wx, wy, wz

        # Joint velocities (will be computed from servo dynamics)
        # For now, assume zero joint velocities (will be updated by servo model)
        v[6] = 0.0  # da1
        v[7] = 0.0  # da2

        return q, v

    def _pinocchio_to_nmpc_state(self, q: np.ndarray, v: np.ndarray, servo_angles: np.ndarray) -> np.ndarray:
        """
        Convert Pinocchio state to NMPC state format.

        Args:
            q: Pinocchio generalized positions
            v: Pinocchio generalized velocities
            servo_angles: Current servo angles [a1, a2]

        Returns:
            NMPC state vector (15)
        """
        x = np.zeros(self.nx)

        # Position
        x[0:3] = q[0:3]

        # Linear velocity
        x[3:6] = v[0:3]

        # Quaternion: Pinocchio [qx, qy, qz, qw] -> NMPC [qw, qx, qy, qz]
        x[6] = q[6]  # qw
        x[7] = q[3]  # qx
        x[8] = q[4]  # qy
        x[9] = q[5]  # qz

        # Angular velocity
        x[10:13] = v[3:6]

        # Servo angles
        x[13:15] = servo_angles

        return x

    def _compute_external_forces(self, u: np.ndarray, servo_angles: np.ndarray) -> pin.StdVec_Force:
        """
        Compute external forces from thrust commands.

        Args:
            u: Control input [ft1c, ft2c, a1c, a2c]
            servo_angles: Current servo angles [a1, a2]

        Returns:
            External forces in Pinocchio format
        """
        # Update frame placements
        pin.framesForwardKinematics(self.model, self.data, self.q)

        # Initialize force vector for all joints
        f_ext = pin.StdVec_Force()
        for _ in range(self.model.njoints):
            f_ext.append(pin.Force.Zero())

        # Thrust commands
        ft1 = u[0]
        ft2 = u[1]

        # Get thrust frame placements
        thrust1_placement = self.data.oMf[self.thrust1_frame_id]
        thrust2_placement = self.data.oMf[self.thrust2_frame_id]

        # Compute thrust forces in local rotor frames
        # Thrust is along z-axis of rotor frame
        f1_local = np.array([0.0, 0.0, ft1])
        f2_local = np.array([0.0, 0.0, ft2])

        # Compute drag torques
        tau1_local = np.array([0.0, 0.0, -self.dr1 * ft1 * self.kq_d_kt])
        tau2_local = np.array([0.0, 0.0, -self.dr2 * ft2 * self.kq_d_kt])

        # Transform to world frame
        f1_world = thrust1_placement.rotation @ f1_local
        f2_world = thrust2_placement.rotation @ f2_local
        tau1_world = thrust1_placement.rotation @ tau1_local
        tau2_world = thrust2_placement.rotation @ tau2_local

        # Create spatial forces at thrust frames
        force1 = pin.Force(f1_world, tau1_world)
        force2 = pin.Force(f2_world, tau2_world)

        # Get joint indices for the frames
        thrust1_joint = self.model.frames[self.thrust1_frame_id].parent
        thrust2_joint = self.model.frames[self.thrust2_frame_id].parent

        # Apply forces at corresponding joints
        f_ext[thrust1_joint] = force1
        f_ext[thrust2_joint] = force2

        return f_ext

    def _compute_servo_torques(self, u: np.ndarray, servo_angles: np.ndarray) -> np.ndarray:
        """
        Compute servo joint torques based on 1st order servo model.

        Args:
            u: Control input [ft1c, ft2c, a1c, a2c]
            servo_angles: Current servo angles [a1, a2]

        Returns:
            Joint torques for gimbal joints
        """
        # Servo commands
        a1c = u[2]
        a2c = u[3]

        # Current angles
        a1 = servo_angles[0]
        a2 = servo_angles[1]

        # 1st order servo model: tau = K_p * (a_cmd - a_current)
        # We use a PD controller to track the desired angle
        # The time constant is modeled through the dynamics

        # For simplicity, we'll use a proportional gain derived from time constant
        # tau_servo = (1/t_servo) * I_servo * (a_cmd - a_current)
        # Approximate servo inertia
        I_servo = 0.0036697  # from phys_param_birotor.py
        K_p = I_servo / self.t_servo

        tau_gimbal1 = K_p * (a1c - a1)
        tau_gimbal2 = K_p * (a2c - a2)

        return np.array([tau_gimbal1, tau_gimbal2])

    def set(self, key: str, value: np.ndarray):
        """
        Set simulation state or input.

        Args:
            key: 'x' for state, 'u' for control input
            value: Value to set
        """
        if key == "x":
            self.x_state = value.copy()
            self.q, self.v = self._nmpc_state_to_pinocchio(value)
        elif key == "u":
            self.u_control = value.copy()
        else:
            raise ValueError(f"Unknown key: {key}")

    def solve(self) -> int:
        """
        Integrate dynamics forward by one timestep.

        Returns:
            Status code (0 for success)
        """
        if self.q is None or self.v is None or self.u_control is None:
            raise RuntimeError("State and control must be set before calling solve()")

        # Current servo angles
        servo_angles = self.x_state[13:15]

        # Compute servo torques
        tau_servo = self._compute_servo_torques(self.u_control, servo_angles)

        # Create full tau vector (6 for floating base + 2 for joints)
        tau = np.zeros(self.model.nv)
        tau[6] = tau_servo[0]  # Gimbal 1
        tau[7] = tau_servo[1]  # Gimbal 2

        # Compute external forces from thrusters
        f_ext = self._compute_external_forces(self.u_control, servo_angles)

        # Compute forward dynamics (ABA algorithm)
        a = pin.aba(self.model, self.data, self.q, self.v, tau, f_ext)

        # Semi-implicit Euler integration
        v_new = self.v + a * self.dt
        q_new = pin.integrate(self.model, self.q, v_new * self.dt)

        # Update servo angles using 1st order model
        # da/dt = (a_cmd - a) / t_servo
        a1_cmd = self.u_control[2]
        a2_cmd = self.u_control[3]
        a1_new = servo_angles[0] + (a1_cmd - servo_angles[0]) / self.t_servo * self.dt
        a2_new = servo_angles[1] + (a2_cmd - servo_angles[1]) / self.t_servo * self.dt

        # Update state
        self.q = q_new
        self.v = v_new
        self.x_state = self._pinocchio_to_nmpc_state(self.q, self.v, np.array([a1_new, a2_new]))

        return 0

    def get(self, key: str) -> np.ndarray:
        """
        Get simulation state.

        Args:
            key: 'x' for state

        Returns:
            State vector
        """
        if key == "x":
            return self.x_state.copy()
        else:
            raise ValueError(f"Unknown key: {key}")


class PinocchioSimWrapper:
    """
    Wrapper class to make Pinocchio simulator compatible with Acados sim solver interface.
    This allows seamless integration with existing sim_nmpc.py code.
    """

    def __init__(self, urdf_path: str, dt: float = 0.005):
        """
        Initialize wrapper.

        Args:
            urdf_path: Path to URDF file
            dt: Integration timestep
        """
        self.simulator = TiltBirotorPinocchioSimulator(urdf_path, dt)
        self.model_name = "tilt_bi_pinocchio_sim"

        # Create mock acados_sim object for compatibility
        class MockAcadosSim:
            class Dims:
                def __init__(self, nx):
                    self.nx = nx

            def __init__(self, nx):
                self.dims = self.Dims(nx)

        self.acados_sim = MockAcadosSim(self.simulator.nx)

    def set(self, key: str, value: np.ndarray):
        """Set state or control input."""
        self.simulator.set(key, value)

    def solve(self) -> int:
        """Integrate dynamics forward."""
        return self.simulator.solve()

    def get(self, key: str) -> np.ndarray:
        """Get state."""
        return self.simulator.get(key)


def create_pinocchio_sim_solver(dt: float = 0.005) -> PinocchioSimWrapper:
    """
    Create a Pinocchio simulator for tiltable birotor.

    Args:
        dt: Integration timestep

    Returns:
        PinocchioSimWrapper instance
    """
    # Get path to URDF file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = current_dir
    for _ in range(10):
        test_path = os.path.join(workspace_root, "robots", "gimbalrotor", "robots", "bi", "gimbalrotor.urdf.xacro")
        if os.path.exists(test_path):
            urdf_path = test_path
            break
        workspace_root = os.path.dirname(workspace_root)
    else:
        raise RuntimeError("Could not find gimbalrotor URDF file")

    print(f"[Pinocchio Sim] Using URDF: {urdf_path}")
    return PinocchioSimWrapper(urdf_path, dt)


if __name__ == "__main__":
    # Test the simulator
    print("Testing Pinocchio simulator...")
    sim = create_pinocchio_sim_solver(dt=0.005)

    # Initialize state
    x0 = np.zeros(15)
    x0[6] = 1.0  # qw

    # Set state
    sim.set("x", x0)

    # Set control (hover thrust)
    u0 = np.array([1.44441 * 9.798 / 2, 1.44441 * 9.798 / 2, 0.0, 0.0])
    sim.set("u", u0)

    # Simulate
    print("Running test simulation...")
    for i in range(100):
        status = sim.solve()
        if status != 0:
            print(f"Error at step {i}")
            break
        x = sim.get("x")
        if i % 20 == 0:
            print(f"Step {i}: pos={x[0:3]}, quat={x[6:10]}")

    print("Test complete!")
