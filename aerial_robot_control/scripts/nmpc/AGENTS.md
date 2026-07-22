# NMPC workspace guidance

This directory contains the tilt-quadrotor NMPC simulations used to compare a
force-compliant controller with an ideal translational impedance model.  The
detailed development history, equations, commands, and recorded results are in
[`FORCE_IMPEDANCE_EXPERIMENT_LOG_20260722.md`](FORCE_IMPEDANCE_EXPERIMENT_LOG_20260722.md).

## Current experiment architecture

- `sim_ee_force_impedance_nmpc.py` is the main closed-loop simulator.  It was
  formerly named `sim_impedance_no_mhe.py`.
- Controller model selections are:
  - `0`: `NMPCTiltQdServoDist`
  - `1`: `NMPCTiltQdServoImpedance`
  - `2`: `NMPCTiltQdServoForceImpedance`
- Model 2 applies impedance only to translational force.  Attitude and torque
  retain the ordinary tracking formulation; torque is not made compliant.
- `sim_impedance_only.py --sim_model 1` produces the ideal force-impedance
  reference trajectory.
- `plot_force_impedance_comparison.py` compares structured NPZ runs and creates
  the main response plot, rotational diagnostics, and CSV metrics.
- The shared 18 s force schedule and NPZ helpers are in
  `nmpc_tilt_mt/utils/force_impedance_experiment.py`.

## Frame options are independent

Do not collapse these three concepts into one option:

- `--controller-state-frame {cog,ee}`: point used by the controller's
  force-impedance state/cost.
- `--wrench-application-point {cog,ee}`: physical point where the simulated
  environment applies its wrench.
- `--plot-state-frame {cog,ee}`: point saved as `state_plot` and shown in the
  comparison plot.

`--interaction-frame` is only a deprecated shorthand for the latter two.  The
default for all three effective values is `ee`.

For the CoG-controller/EE-contact experiment, zero only the controller copy of
the acados `ee_p` parameter.  Never mutate the shared physical-parameter module:
the plant must retain its real lever arm (`ball_effector_p = [0, 0, 0.264]`).
An EE force is converted to the equivalent CoG wrench using

```text
f_CoG^W = f_EE^W
tau_CoG^B = p_BE^B x (R_WB^T f_EE^W) + tau_EE^B
```

## Critical implementation invariants

- Never hard-code the force-impedance parameter slice as `34:40`.  The acados
  parameter layout is currently quaternion reference `0:4`, physical parameters
  `4:35`, optional predicted CoG disturbance `35:41`, then impedance parameters
  `41:47`.  Derive the start dynamically from
  `len(nmpc.phys.physical_param_list)` and
  `nmpc.include_cog_dist_parameter` as the simulator does.
- Preserve the controller/plant distinction when changing `ee_p`.  Shared-module
  mutation silently makes the plant CoG-centric as well.
- Re-read `BeetleNMPCFullServoForceImp.yaml` before every batch.  The NPZ
  metadata records M/D/K but does not currently record `enlarge_factor`, so put
  the YAML value in the output name/run label at execution time.  The
  configuration changed repeatedly during these experiments.
- Use unique, parameterized output names.  Do not overwrite earlier NPZ, PNG,
  PDF, or CSV results.
- Keep `--save-run` paths normalized before acados can change the process working
  directory.
- Keep the terminal force-impedance residual independent of control-dependent
  acceleration.  It currently uses `-fds_w` at the terminal node intentionally.
- Preserve unrelated local files and generated data.  This workspace may have a
  dirty worktree; never reset or delete them as cleanup.

## Canonical commands

Run from this `nmpc` directory.  Use a writable Matplotlib cache on machines
whose home cache is read-only.

Ideal truth for the currently configured M/D/K:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_impedance_only.py --sim_model 1 -p 4 \
  --scenario force-impedance-compare \
  --save-run sim_data/comparison/nominal_force_impedance_UNIQUE.npz
```

EE-centric baseline:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_ee_force_impedance_nmpc.py 2 -e 0 -p 4 \
  --controller-state-frame ee \
  --wrench-application-point ee \
  --plot-state-frame ee \
  --torque-compensation lever-arm \
  --ee-acceleration full \
  --scenario force-impedance-compare \
  --save-run sim_data/comparison/nmpc_force_impedance_UNIQUE.npz
```

CoG controller, physical load at EE, CoG plot:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_ee_force_impedance_nmpc.py 2 -e 0 -p 4 \
  --controller-state-frame cog \
  --wrench-application-point ee \
  --plot-state-frame cog \
  --torque-compensation lever-arm \
  --ee-acceleration cog \
  --scenario force-impedance-compare \
  --save-run sim_data/comparison/nmpc_force_impedance_cog_controller_UNIQUE.npz
```

Comparison and diagnostics:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 plot_force_impedance_comparison.py \
  --nmpc sim_data/comparison/NMPC_RUN.npz \
  --truth sim_data/comparison/TRUTH_RUN.npz \
  --run-label 'K=..., enlarge_factor=...' \
  --output-prefix experiment_results/force_impedance_comparison_UNIQUE
```

## Validation checklist

After changing frame logic or rerunning an experiment, check all of the
following:

1. `python3 -m py_compile` succeeds for modified Python files and
   `git diff --check` is clean.
2. Saved state and input arrays contain only finite values and have the expected
   `N+1` versus `N` lengths.
3. NPZ metadata reports the intended controller, wrench, and plot frames plus
   the actual M/D/K.  Confirm `enlarge_factor` separately from the YAML and run
   name because it is not currently stored in the bundle.
4. `state_plot` equals `state_cog` or `state_ee` according to
   `plot_state_frame`.
5. For EE loading, the saved CoG torque equals the lever-arm cross product.
6. Comparison truth and NMPC runs have identical scenario and M/D/K metadata.

Communicate experiment conclusions in Chinese unless the user asks otherwise.
