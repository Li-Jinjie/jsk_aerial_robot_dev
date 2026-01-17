#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pinocchio 3D visualization demo for tiltable birotor
使用 MeshCat 实时可视化机器人运动

Requirements:
    pip install meshcat

Usage:
    python3 visualize_pinocchio_3d.py
"""

import sys
import os
import numpy as np
import time

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tilt_bi_pinocchio_sim import create_pinocchio_sim_solver

# Try to import Pinocchio and MeshCat
try:
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer
    import hppfcl

    print("✓ Pinocchio and MeshCat visualization available")
except ImportError as e:
    print(f"✗ Error: {e}")
    print("\nPlease install MeshCat: pip3 install meshcat")
    sys.exit(1)


class PinocchioBirotorVisualizer:
    """
    3D visualization wrapper for Pinocchio birotor simulator using MeshCat.

    This class provides a browser-based 3D visualization of the robot's motion
    using Pinocchio's built-in MeshCat visualizer.
    """

    def __init__(self, simulator):
        """
        Initialize visualizer.

        Args:
            simulator: TiltBirotorPinocchioSimulator instance
        """
        self.simulator = simulator
        self.model = simulator.model
        self.data = simulator.data
        self._init_meshcat()

    def _init_meshcat(self):
        """Initialize MeshCat visualizer with geometry models."""
        print("\n[MeshCat] Initializing 3D visualizer...")

        # Get URDF path
        urdf_path = self.simulator.urdf_path
        print(f"  URDF path: {urdf_path}")

        # Try to load geometry models
        try:
            package_dirs = [os.path.dirname(urdf_path)]

            # Build collision and visual models
            collision_model = pin.buildGeomFromUrdf(
                self.model, urdf_path, pin.GeometryType.COLLISION, package_dirs=package_dirs
            )
            visual_model = pin.buildGeomFromUrdf(
                self.model, urdf_path, pin.GeometryType.VISUAL, package_dirs=package_dirs
            )

            print(f"  ✓ Loaded {len(visual_model.geometryObjects)} visual geometries")

        except Exception as e:
            print(f"  ⚠ Warning: Could not load full geometry: {e}")
            print("  Using simplified visualization (spheres for each link)...")

            # Create empty models
            collision_model = pin.GeometryModel()
            visual_model = pin.GeometryModel()

            # Add sphere for each joint/link
            for i in range(1, self.model.njoints):  # Skip universe
                joint_name = self.model.names[i]
                geom = pin.GeometryObject(
                    f"sphere_{joint_name}", i, hppfcl.Sphere(0.03), pin.SE3.Identity()  # 3cm radius
                )
                visual_model.addGeometryObject(geom)

            print(f"  ✓ Created {len(visual_model.geometryObjects)} simplified geometries")

        # Create visualizer data
        self.collision_data = pin.GeometryData(collision_model)
        self.visual_data = pin.GeometryData(visual_model)

        # Create MeshCat visualizer
        self.viz = MeshcatVisualizer(self.model, collision_model, visual_model)

        # Initialize viewer
        try:
            self.viz.initViewer(open=True)
        except Exception as err:
            print(f"  Note: {err}")
            self.viz.initViewer(loadModel=False, open=True)

        # Load model
        try:
            self.viz.loadViewerModel()
        except Exception as e:
            print(f"  Warning during model loading: {e}")

        print(f"\n✓ MeshCat server started!")
        print(f"  📱 Open your browser at: {self.viz.viewer.url()}")
        print(f"     (Usually: http://127.0.0.1:7000/static/)")
        print()

    def display(self, q):
        """
        Display robot at configuration q.

        Args:
            q: Generalized positions (Pinocchio format)
        """
        self.viz.display(q)

    def update_from_state(self, x_state):
        """
        Update visualization from NMPC state.

        Args:
            x_state: NMPC state vector (15D)
        """
        # Convert to Pinocchio q
        q, _ = self.simulator._nmpc_state_to_pinocchio(x_state)
        self.display(q)


def demo_hover_simulation():
    """
    Demo 1: Hover simulation with small disturbances.
    The robot will hover at 1m altitude with periodic thrust variations.
    """
    print("\n" + "=" * 70)
    print(" 🚁 Pinocchio Birotor 3D Visualization Demo")
    print(" Demo 1: Hover with Disturbances")
    print("=" * 70)

    # Create simulator
    print("\n[Step 1/5] Creating Pinocchio simulator...")
    sim = create_pinocchio_sim_solver(dt=0.001)

    # Create visualizer
    print("\n[Step 2/5] Creating 3D visualizer...")
    viz = PinocchioBirotorVisualizer(sim.simulator)

    # Initial state (hovering at z=1m)
    print("\n[Step 3/5] Setting initial state...")
    x0 = np.zeros(15)
    x0[2] = 1.0  # z = 1m
    x0[6] = 1.0  # qw = 1 (identity quaternion)
    sim.set("x", x0)

    # Hover thrust (split gravity equally)
    mass = sim.simulator.mass
    g = sim.simulator.gravity
    hover_thrust = mass * g / 2.0

    print(f"  Mass: {mass:.3f} kg")
    print(f"  Gravity: {g:.3f} m/s²")
    print(f"  Hover thrust per rotor: {hover_thrust:.3f} N")

    # Display initial configuration
    viz.update_from_state(x0)

    input("\n  Press Enter to start simulation (make sure browser is ready)...")

    print("\n[Step 4/5] Running simulation (10 seconds)...")
    print("  👀 Watch the 3D visualization in your browser!")
    print("  The robot will hover with small periodic disturbances.\n")

    # Simulation parameters
    t_total = 10.0
    dt = 0.01
    n_steps = int(t_total / dt)

    # Run simulation
    for i in range(n_steps):
        t = i * dt

        # Add small periodic disturbances for interesting motion
        thrust1 = hover_thrust
        thrust2 = hover_thrust

        # Gimbal angles - small oscillations
        alpha1 = 0.1
        alpha2 = 0.1

        u = np.array([thrust1, thrust2, alpha1, alpha2])
        sim.set("u", u)

        # Step simulation
        status = sim.solve()
        if status != 0:
            print(f"[ERROR] Simulation failed at step {i}")
            break

        # Update visualization
        if i % 10 == 0:
            x = sim.get("x")
            viz.update_from_state(x)

        # Print progress
        if i % 200 == 0:
            x = sim.get("x")
            print(
                f"  ⏱  t={t:5.2f}s  |  pos=[{x[0]:6.3f}, {x[1]:6.3f}, {x[2]:6.3f}]  |  "
                f"gimbal=[{x[13]:6.3f}, {x[14]:6.3f}] rad"
            )

        # Small delay for smooth visualization
        time.sleep(0.005)

    print("\n[Step 5/5] ✓ Simulation complete!")
    print("  Keep the browser window open to inspect the robot.")

    try:
        input("\n  Press Enter to exit...")
    except KeyboardInterrupt:
        print("\nExiting...")


def demo_gimbal_actuation():
    """
    Demo 2: Demonstrate gimbal joint actuation patterns.
    Shows three different gimbal movement patterns.
    """
    print("\n" + "=" * 70)
    print(" 🚁 Pinocchio Birotor 3D Visualization Demo")
    print(" Demo 2: Gimbal Actuation Patterns")
    print("=" * 70)

    # Create simulator
    print("\n[Step 1/4] Creating simulator...")
    sim = create_pinocchio_sim_solver(dt=0.01)

    # Create visualizer
    print("\n[Step 2/4] Creating visualizer...")
    viz = PinocchioBirotorVisualizer(sim.simulator)

    # Initial state
    print("\n[Step 3/4] Setting initial state...")
    x0 = np.zeros(15)
    x0[2] = 1.0  # z = 1m
    x0[6] = 1.0  # qw = 1
    sim.set("x", x0)

    mass = sim.simulator.mass
    g = sim.simulator.gravity
    hover_thrust = mass * g / 2.0

    viz.update_from_state(x0)

    input("\n  Press Enter to start gimbal actuation demo...")

    print("\n[Step 4/4] Running gimbal actuation patterns (12 seconds)...")
    print("  Phase 1 (0-4s):  Both gimbals move together")
    print("  Phase 2 (4-8s):  Gimbals move in opposite directions")
    print("  Phase 3 (8-12s): Gimbals move at different frequencies\n")

    t_total = 12.0
    dt = 0.01
    n_steps = int(t_total / dt)

    for i in range(n_steps):
        t = i * dt

        # Different patterns for each phase
        if t < 4.0:
            # Phase 1: Together
            alpha1 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
            alpha2 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
            phase_name = "Together      "
        elif t < 8.0:
            # Phase 2: Opposite
            alpha1 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
            alpha2 = -0.3 * np.sin(2 * np.pi * 0.5 * t)
            phase_name = "Opposite      "
        else:
            # Phase 3: Different frequencies
            alpha1 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
            alpha2 = 0.3 * np.sin(2 * np.pi * 0.7 * t)
            phase_name = "Different Freq"

        u = np.array([hover_thrust, hover_thrust, alpha1, alpha2])
        sim.set("u", u)

        status = sim.solve()
        if status != 0:
            print(f"[ERROR] Failed at step {i}")
            break

        if i % 2 == 0:
            x = sim.get("x")
            viz.update_from_state(x)

        if i % 100 == 0:
            x = sim.get("x")
            print(f"  ⏱  t={t:5.2f}s  [{phase_name}]  |  " f"gimbal=[{x[13]:6.3f}, {x[14]:6.3f}] rad")

        time.sleep(0.005)

    print("\n✓ Demo complete!\n")

    try:
        input("  Press Enter to exit...")
    except KeyboardInterrupt:
        print("\nExiting...")


def demo_trajectory_following():
    """
    Demo 3: Follow a circular trajectory with simple P control.
    """
    print("\n" + "=" * 70)
    print(" 🚁 Pinocchio Birotor 3D Visualization Demo")
    print(" Demo 3: Circular Trajectory Following")
    print("=" * 70)

    # Create simulator
    print("\n[Step 1/4] Creating simulator...")
    sim = create_pinocchio_sim_solver(dt=0.01)

    # Create visualizer
    print("\n[Step 2/4] Creating visualizer...")
    viz = PinocchioBirotorVisualizer(sim.simulator)

    # Initial state
    print("\n[Step 3/4] Setting initial state...")
    x0 = np.zeros(15)
    x0[2] = 1.0  # z = 1m
    x0[6] = 1.0  # qw = 1
    sim.set("x", x0)

    mass = sim.simulator.mass
    g = sim.simulator.gravity
    hover_thrust = mass * g / 2.0

    viz.update_from_state(x0)

    input("\n  Press Enter to start trajectory following...")

    print("\n[Step 4/4] Following circular trajectory...")
    print("  Radius: 0.5m, Height: 1m, Period: 8s")
    print("  Watch the robot fly in a circle!\n")

    # Simulation parameters
    t_total = 16.0  # 2 full circles
    dt = 0.01
    n_steps = int(t_total / dt)

    # Trajectory parameters
    radius = 0.5
    height = 1.0
    period = 8.0
    omega = 2 * np.pi / period

    for i in range(n_steps):
        t = i * dt

        # Circular trajectory
        x_target = radius * np.cos(omega * t)
        y_target = radius * np.sin(omega * t)

        # Simple proportional control
        x_current = sim.get("x")
        px, py, pz = x_current[0:3]

        error_x = x_target - px
        error_y = y_target - py

        # P controller for gimbal angles
        k_p = 0.5
        alpha1_cmd = np.clip(k_p * error_y, -0.3, 0.3)
        alpha2_cmd = np.clip(-k_p * error_x, -0.3, 0.3)

        # Thrust to maintain altitude
        error_z = height - pz
        thrust_total = hover_thrust * 2 + 2.0 * error_z
        thrust1 = thrust_total / 2
        thrust2 = thrust_total / 2

        u = np.array([thrust1, thrust2, alpha1_cmd, alpha2_cmd])
        sim.set("u", u)

        status = sim.solve()
        if status != 0:
            print(f"[ERROR] Simulation failed at step {i}")
            break

        if i % 2 == 0:
            x = sim.get("x")
            viz.update_from_state(x)

        if i % 100 == 0:
            print(
                f"  ⏱  t={t:5.2f}s  |  pos=[{px:6.3f}, {py:6.3f}, {pz:6.3f}]  |  "
                f"target=[{x_target:6.3f}, {y_target:6.3f}, {height:6.3f}]"
            )

        time.sleep(0.005)

    print("\n✓ Trajectory complete!\n")

    try:
        input("  Press Enter to exit...")
    except KeyboardInterrupt:
        print("\nExiting...")


def main():
    """Main function - choose demo."""
    print("\n" + "=" * 70)
    print(" 🚁 Pinocchio Birotor 3D Visualization Demos")
    print("=" * 70)
    print("\n Available demos:")
    print("   1. Hover with disturbances (recommended for first-time)")
    print("   2. Gimbal actuation patterns")
    print("   3. Circular trajectory following")
    print("   4. Run all demos sequentially")
    print()

    try:
        choice = input(" Select demo [1-4, default=1]: ").strip()
        if not choice:
            choice = "1"

        if choice == "1":
            demo_hover_simulation()
        elif choice == "2":
            demo_gimbal_actuation()
        elif choice == "3":
            demo_trajectory_following()
        elif choice == "4":
            print("\n Running all demos...")
            demo_hover_simulation()
            print("\n" + "-" * 70 + "\n")
            demo_gimbal_actuation()
            print("\n" + "-" * 70 + "\n")
            demo_trajectory_following()
        else:
            print(f" Invalid choice: {choice}")
            return

    except KeyboardInterrupt:
        print("\n\n Interrupted by user.")
    except Exception as e:
        print(f"\n [ERROR] {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
