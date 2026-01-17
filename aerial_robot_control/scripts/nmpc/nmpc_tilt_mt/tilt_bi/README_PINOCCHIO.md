# Pinocchio Multi-Body Simulator for Tiltable Birotor

This module provides a high-fidelity multi-body dynamics simulator for the tiltable birotor using the Pinocchio library, as an alternative to the Acados numerical integrator.

## Features

- **Multi-body dynamics**: Uses Pinocchio's Articulated-Body Algorithm (ABA) for accurate computation of dynamics with gimbal joints
- **URDF-based**: Loads the robot model directly from the Gazebo URDF file
- **Compatible interface**: Drop-in replacement for Acados sim solver with the same API
- **Servo dynamics**: Includes 1st-order servo model for gimbal actuation
- **External forces**: Properly applies rotor thrust and drag torques at appropriate frames

## Architecture

The simulator consists of two main classes:

### TiltBirotorPinocchioSimulator
Core simulator that:
- Loads URDF and builds Pinocchio model with free-flyer base
- Converts between NMPC state format (15D) and Pinocchio generalized coordinates
- Computes forward dynamics using ABA with external forces
- Integrates using semi-implicit Euler method

### PinocchioSimWrapper
Wrapper class that provides Acados-compatible interface:
- `set(key, value)`: Set state ('x') or control ('u')
- `solve()`: Integrate dynamics forward by one timestep
- `get(key)`: Get current state
- `acados_sim.dims.nx`: State dimension for compatibility

## State and Control Format

**NMPC State (15D):**
```
[px, py, pz,           # Position (m)
 vx, vy, vz,           # Linear velocity (m/s)
 qw, qx, qy, qz,       # Quaternion (scalar-first)
 wx, wy, wz,           # Angular velocity (rad/s)
 a1, a2]               # Servo angles (rad)
```

**Control Input (4D):**
```
[ft1c, ft2c,           # Thrust commands (N)
 a1c, a2c]             # Servo angle commands (rad)
```

**Pinocchio Generalized Coordinates:**
- q (9D): [px, py, pz, qx, qy, qz, qw, a1, a2]
- v (8D): [vx, vy, vz, wx, wy, wz, da1, da2]

## Installation

### Prerequisites

1. **Pinocchio** (Python bindings):
   ```bash
   # Using pip
   pip install pin

   # OR using conda (recommended)
   conda install -c conda-forge pinocchio
   ```

   **System / ROS package (recommended on ROS systems):**
   If you installed Pinocchio using the ROS packaged build (apt), install with:
   ```bash
   sudo apt update
   sudo apt install ros-${ROS_DISTRO}-pinocchio
   ```

   Note: On some ROS/Ubuntu setups the apt-installed Python bindings may not be on the default
   Python search path. If `import pinocchio` fails after apt install, add the ROS Python site-packages
   to your shell startup file (example for ROS Noetic / Python 3.8):
   ```bash
   # Add to ~/.bashrc (or ~/.zshrc if you use zsh)
   export PYTHONPATH=/opt/ros/noetic/lib/python3.8/site-packages:$PYTHONPATH
   # Then reload the shell or source the file
   source ~/.bashrc
   ```

2. **xacro** (for URDF conversion):
   ```bash
   sudo apt-get install ros-${ROS_DISTRO}-xacro
   ```

### Verification

Test the simulator independently:
```bash
cd aerial_robot_control/scripts/nmpc/nmpc_tilt_mt/tilt_bi
python test_pinocchio_sim.py
```

## Usage

### Command Line

Run simulation with Pinocchio multi-body simulator:
```bash
cd aerial_robot_control/scripts/nmpc
python sim_nmpc.py 0 -a bi --sim_model 2
```

Arguments:
- `0`: NMPC model (0 = basic servo model)
- `-a bi`: Birotor architecture
- `--sim_model 2`: Use Pinocchio simulator (0=Acados servo+thrust, 2=Pinocchio)

### Comparison with Acados

Run both simulators and compare:
```bash
# Acados simulator
python sim_nmpc.py 0 -a bi --sim_model 0 -s --file_path ./results/

# Pinocchio simulator
python sim_nmpc.py 0 -a bi --sim_model 2 -s --file_path ./results/
```

### Python API

```python
import numpy as np
from nmpc_tilt_mt.tilt_bi.tilt_bi_pinocchio_sim import create_pinocchio_sim_solver

# Create simulator
sim = create_pinocchio_sim_solver(dt=0.005)

# Initialize state
x0 = np.zeros(15)
x0[6] = 1.0  # qw
sim.set('x', x0)

# Set control
u = np.array([7.08, 7.08, 0.0, 0.0])  # Hover thrust
sim.set('u', u)

# Step simulation
status = sim.solve()
x_new = sim.get('x')
```

## Implementation Details

### URDF Loading

The simulator automatically:
1. Locates the URDF file: `robots/gimbalrotor/robots/bi/gimbalrotor.urdf.xacro`
2. Converts xacro to URDF using `xacro` command
3. Loads the URDF with Pinocchio's `buildModelFromUrdf()`
4. Adds a free-flyer joint for the floating base

### Dynamics Computation

For each timestep:
1. **Servo torques**: Computed using 1st-order model `τ = (I_servo/t_servo) × (α_cmd - α)`
2. **External forces**: Rotor thrust and drag applied at thrust frames
3. **Forward dynamics**: ABA algorithm computes accelerations
4. **Integration**: Semi-implicit Euler (v_new = v + a×dt, q_new = integrate(q, v_new×dt))
5. **Servo integration**: Separate 1st-order ODE for servo angles

### Coordinate Transformations

**Quaternion conversion:**
- NMPC uses scalar-first: [qw, qx, qy, qz]
- Pinocchio uses scalar-last: [qx, qy, qz, qw]

**Joint indexing:**
- Floating base: 7 DOF position (3 translation + 4 quaternion)
- Gimbal joints: 2 DOF (revolute joints)

## Limitations and Future Work

### Current Limitations
1. **Rotor dynamics**: Does not include rotor inertia effects or blade flapping
2. **Aerodynamics**: No aerodynamic drag or ground effects
3. **Flexibility**: Assumes rigid links (no structural deformation)
4. **Collisions**: No collision detection or contact dynamics

### Possible Extensions
1. **Higher-fidelity rotor model**: Include motor dynamics and blade dynamics
2. **Aerodynamic forces**: Add air resistance and wind disturbances
3. **Flexible bodies**: Use Pinocchio's flexible joint models
4. **Contact simulation**: Add ground contact for landing scenarios
5. **Real-time capability**: Optimize for real-time performance with code generation

## Performance Comparison

Expected performance characteristics:

| Metric | Acados | Pinocchio |
|--------|--------|-----------|
| Accuracy | Good | Excellent |
| Computation | ~0.1-0.5ms | ~0.5-2ms |
| Model fidelity | Single rigid body | Multi-body with joints |
| Joint constraints | Approximate | Exact |
| Best for | Fast NMPC prototyping | High-fidelity validation |

## Troubleshooting

### "Cannot find reference 'buildModelFromUrdf'"
- Pinocchio is not installed. Install with `pip install pin` or conda.

### "Could not find gimbalrotor URDF file"
- Ensure you're running from the correct workspace
- Check that `robots/gimbalrotor/robots/bi/gimbalrotor.urdf.xacro` exists

### "Error converting xacro"
- Install xacro: `sudo apt-get install ros-${ROS_DISTRO}-xacro`
- Verify xacro is in PATH: `which xacro`

### Simulation diverges
- Reduce timestep: `ts_sim = 0.001` instead of `0.005`
- Check initial conditions (especially quaternion normalization)
- Verify thrust commands are reasonable (hover ~7N per rotor)

## References

1. Pinocchio: A fast and flexible implementation of Rigid Body Dynamics algorithms
   - Documentation: https://stack-of-tasks.github.io/pinocchio/
   - Paper: Carpentier et al., "Pinocchio: Fast Forward and Inverse Dynamics for Poly-Articulated Systems", 2019

2. Acados: Fast and embedded solvers for nonlinear optimal control
   - Documentation: https://docs.acados.org/

## Authors

- Implementation: GitHub Copilot (2026)
- Original NMPC framework: JSK Aerial Robot Team

## License

Same as the parent project (jsk_aerial_robot)
