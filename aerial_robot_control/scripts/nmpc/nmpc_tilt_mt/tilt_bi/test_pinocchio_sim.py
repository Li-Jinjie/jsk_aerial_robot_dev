#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test script for Pinocchio simulator
Tests basic functionality before integration with full NMPC system
"""

import numpy as np
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from tilt_bi_pinocchio_sim import create_pinocchio_sim_solver

    print("=" * 60)
    print("Testing Pinocchio Simulator for Tiltable Birotor")
    print("=" * 60)

    # Create simulator
    print("\n[1] Creating simulator...")
    sim = create_pinocchio_sim_solver(dt=0.005)
    print("[OK] Simulator created successfully")

    # Initialize state
    print("\n[2] Initializing state...")
    x0 = np.zeros(15)
    x0[6] = 1.0  # qw (quaternion scalar)
    sim.set("x", x0)
    print(f"[OK] Initial state set: pos={x0[0:3]}, quat={x0[6:10]}")

    # Set hover control (split gravity equally between two rotors)
    print("\n[3] Setting hover thrust commands...")
    mass = 1.44441
    g = 9.798
    hover_thrust = mass * g / 2.0
    u0 = np.array([hover_thrust, hover_thrust, 0.0, 0.0])
    sim.set("u", u0)
    print(f"[OK] Control set: thrust1={u0[0]:.2f}N, thrust2={u0[1]:.2f}N, angles={u0[2:4]}")

    # Run simulation
    print("\n[4] Running simulation for 2 seconds...")
    n_steps = 400  # 2 seconds at 0.005s timestep
    for i in range(n_steps):
        status = sim.solve()
        if status != 0:
            print(f"[ERROR] Error at step {i}, status={status}")
            sys.exit(1)

        if i % 100 == 0:
            x = sim.get("x")
            print(f"  Step {i:3d}: z={x[2]:6.3f}m, qw={x[6]:6.3f}, servo=[{x[13]:5.2f}, {x[14]:5.2f}]")

    # Final state
    x_final = sim.get("x")
    print(f"\n[OK] Simulation completed successfully")
    print(f"  Final position: [{x_final[0]:.4f}, {x_final[1]:.4f}, {x_final[2]:.4f}]")
    print(f"  Final velocity: [{x_final[3]:.4f}, {x_final[4]:.4f}, {x_final[5]:.4f}]")
    print(f"  Final quaternion: [{x_final[6]:.4f}, {x_final[7]:.4f}, {x_final[8]:.4f}, {x_final[9]:.4f}]")

    # Test with servo commands
    print("\n[5] Testing servo actuation...")
    x0 = np.zeros(15)
    x0[6] = 1.0
    sim.set("x", x0)

    # Command 30 degree tilt
    u_tilt = np.array([hover_thrust, hover_thrust, 0.5, -0.5])  # ~30 degrees
    sim.set("u", u_tilt)

    for i in range(200):  # 1 second
        status = sim.solve()
        if status != 0:
            print(f"[ERROR] Error at step {i}, status={status}")
            sys.exit(1)

    x_final = sim.get("x")
    print(f"[OK] Servo test completed")
    print(f"  Final servo angles: [{x_final[13]:.3f}, {x_final[14]:.3f}] rad")
    print(f"  (Target was [0.500, -0.500] rad)")

    print("\n" + "=" * 60)
    print("All tests passed! [OK]")
    print("=" * 60)
    print("\nYou can now run the full simulation with:")
    print("  python sim_nmpc.py 0 -a bi --sim_model 2")

except ImportError as e:
    print(f"[ERROR] Import error: {e}")
    print("\nMake sure Pinocchio is installed:")
    print("  pip install pin")
    print("  OR")
    print("  conda install -c conda-forge pinocchio")
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] Error: {e}")
    import traceback

    traceback.print_exc()
    sys.exit(1)
