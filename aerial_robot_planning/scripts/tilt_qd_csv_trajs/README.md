# Tilt Quadrotor CSV Trajectory Data

This directory contains CSV trajectory files for tilt quadrotor robots with structured data format for efficient processing and hardware deployment.

## Data Format Structure

### Header Format
Each CSV file contains a 3-line header followed by trajectory data:

```
robot,beetle1
frame,cog
time,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,a1,a2,a3,a4,control effort,fc1,fc2,fc3,fc4,ac1,ac2,ac3,ac4
```

**Header Lines:**
1. `robot,<robot_name>` - Specifies the robot model (e.g., beetle1)
2. `frame,<frame_type>` - Reference frame for the trajectory (e.g., cog for center of gravity, ee for end effector)
3. Column headers - Describes each data column

### Data Columns (27 columns total)

| Column | Name | Description |
|--------|------|-------------|
| 1 | time | Time stamp (seconds) |
| 2-4 | px, py, pz | Position coordinates (x, y, z) in meters |
| 5-7 | vx, vy, vz | Velocity components (x, y, z) in m/s |
| 8-11 | qw, qx, qy, qz | Quaternion orientation (w, x, y, z) |
| 12-14 | wx, wy, wz | Angular velocity components (rad/s) |
| 15-18 | a1, a2, a3, a4 | Tilt angles for rotors 1-4 (rad) |
| 19 | control effort | Overall control effort metric |
| 20-23 | fc1, fc2, fc3, fc4 | Force commands for rotors 1-4 (N) |
| 24-27 | ac1, ac2, ac3, ac4 | Additional control for tilt angles |

Note that the previous csv files (e.g., scvx_traj_max_v=1_1.csv) have 28-th column, that is an additional control variable for time. It is not used anymore, we treat the time as a variable directly now.

<!-- ### Example Data Row
```
0.32378369,-2.12093765,-0.16995651,1.07857317,-0.76044131,-1.29754462,0.60497767,0.82664959,-0.15297602,0.39020316,-0.37548673,-1.76566635,3.46582686,-2.80038009,0.32484633,-1.47890692,-1.13291159,1.35429701,180.82157107,11.90997970,17.22617390,13.07789141,16.30333026,0.08029977,-1.66545061,-1.85885896,1.39999946
``` -->

## Why Row-Based Storage?

The trajectory data is stored in row-based format for several important reasons:

### 1. CSV Sequential Reading Efficiency
- **Line-by-line processing**: CSV format allows reading individual trajectory points sequentially
- **Memory efficiency**: Can process large trajectories without loading entire file into memory
- **Real-time streaming**: Enables real-time trajectory following by reading one line at a time

### 2. Hardware Deployment Optimization
- **Row-based hardware storage**: When deploying to embedded systems, Python stores data to hardware in row-based format
- **Memory layout optimization**: Hardware memory controllers are optimized for sequential row access
- **Cache efficiency**: Accessing consecutive memory locations (row data) maximizes CPU cache utilization

### 3. NumPy Array Performance
- **Fast array conversion**: Converting CSV to NumPy array preserves row-based memory layout
- **Vectorized operations**: NumPy operations on rows are highly optimized
- **Memory contiguity**: Row-major order (C-style) ensures contiguous memory access patterns

```python
# Example: Fast trajectory loading and processing
import numpy as np
import pandas as pd

# Load trajectory data
traj_data = pd.read_csv('trajectory.csv', skiprows=3).values  # Skip header lines
# Result: NumPy array with shape (n_points, 27)

# Fast row-wise operations
positions = traj_data[:, 1:4]      # Extract position columns (px, py, pz)
velocities = traj_data[:, 4:7]     # Extract velocity columns (vx, vy, vz)
quaternions = traj_data[:, 7:11]   # Extract quaternion columns (qw, qx, qy, qz)
```

## File Types

### Trajectory Files
- `scvx_patraj_gazebo.csv` - Gazebo simulation trajectory for perception-aware flight
- `scvx_traj_max_v=X_Y.csv` - SCVX generated trajectories with different velocity constraints
  - `X` = maximum velocity constraint
  - `Y` = trajectory variant number

<!-- ### Utility Scripts
- `add_header.py` - Adds standardized headers to CSV files
- `reorder_csv.py` - Reorders columns for specific requirements -->

## Usage Examples

### Loading Trajectory Data
```python
import pandas as pd
import numpy as np

# Method 1: Using pandas (recommended for analysis)
df = pd.read_csv('scvx_patraj_gazebo.csv', skiprows=3)
positions = df[['px', 'py', 'pz']].values

# Method 2: Direct NumPy loading (fastest for computation)
data = np.loadtxt('scvx_patraj_gazebo.csv', delimiter=',', skiprows=3)
time = data[:, 0]
positions = data[:, 1:4]
```

<!-- ### Real-time Trajectory Following
```python
import csv

def follow_trajectory(filename):
    with open(filename, 'r') as file:
        # Skip headers
        for _ in range(3):
            next(file)

        reader = csv.reader(file)
        for row in reader:
            time, px, py, pz = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            # Send position command to robot
            send_position_command(px, py, pz)
            wait_until_time(time)
``` -->

<!-- ## Data Generation -->

<!-- These trajectories are generated using:
- **SCVX (Sequential Convex Programming)** - For optimal trajectory generation
- **Gazebo simulation** - For physics-based trajectory validation
- Various velocity and acceleration constraints for different flight profiles

The row-based format ensures optimal performance across the entire pipeline from trajectory generation to hardware execution. -->
