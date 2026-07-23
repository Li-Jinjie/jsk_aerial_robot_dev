# Force-impedance NMPC development and experiment log

Initially summarized: 2026-07-22; updated 2026-07-23 (Asia/Tokyo)

This document records the design decisions, fixes, commands, and experimental
results from the force-impedance NMPC conversation.  It is intended to make the
work reproducible after moving between machines or development platforms.  It
is a technical summary, not a verbatim transcript.

## 1. Goal and final design

The original wrench estimator estimated torque poorly.  The desired controller
therefore became a hybrid:

- translational force follows an impedance objective;
- attitude uses the previous non-impedance controller;
- torque is not included in the compliant objective;
- torque compensation, when desired, is supplied independently from a lever-arm
  calculation or an estimator.

The resulting class is
`nmpc_tilt_mt/tilt_qd/tilt_qd_servo_dist_force_imp.py`:
`NMPCTiltQdServoForceImpedance`.  Its main configuration is
`../../../robots/beetle_omni/config/BeetleNMPCFullServoForceImp.yaml`.

The main simulation script is now `sim_ee_force_impedance_nmpc.py`; it was
renamed from `sim_impedance_no_mhe.py` to make its purpose identifiable.

## 2. Relevant files

| File | Role |
| --- | --- |
| `sim_ee_force_impedance_nmpc.py` | Closed-loop NMPC/plant simulation, frame selection, wrench conversion, structured saving |
| `sim_impedance_only.py` | Ideal second-order impedance truth simulation |
| `plot_force_impedance_comparison.py` | Main comparison, metrics, and rotational diagnostics |
| `nmpc_tilt_mt/tilt_qd/tilt_qd_servo_dist_force_imp.py` | Force-only compliant NMPC |
| `nmpc_tilt_mt/utils/force_impedance_experiment.py` | Shared scenario, steady-state windows, metadata-aware NPZ helpers |
| `../../../robots/beetle_omni/config/BeetleNMPCFullServoForceImp.yaml` | Force-impedance weights and virtual M/D/K |

Controller selection in the main simulator:

| Model | Controller |
| ---: | --- |
| 0 | `NMPCTiltQdServoDist` |
| 1 | `NMPCTiltQdServoImpedance` |
| 2 | `NMPCTiltQdServoForceImpedance` |

The normal plant model remains `NMPCTiltQdServoThrustDist`.

## 3. Force-impedance equation

The translational nonlinear least-squares residual is based on

```text
M a + D v + K p - f = 0
```

where `M`, `D`, and `K` are the virtual mass, damping, and stiffness, and `f` is
the applied/predicted world-frame force.  Position and velocity are expressed at
the controller-selected kinematic point.

For an end effector fixed rigidly to the body at `p_BE`, the full world-frame EE
acceleration is

```text
a_EE^W = a_CoG^W
         + R_WB [alpha_B x p_BE
                 + omega_B x (omega_B x p_BE)]
```

`--ee-acceleration full` uses this expression.  `--ee-acceleration cog` uses
only `a_CoG^W`.  There are no relative-motion Coriolis terms because the end
effector is fixed to the body.

At a true static equilibrium, angular velocity and angular acceleration vanish,
so the two acceleration definitions coincide.  In NMPC simulation they need not
produce the same trajectory or even the same apparent steady state because the
impedance equation is a soft cost competing with attitude and actuator costs,
the horizon is finite, and SQP-RTI follows a local solution.  Small persistent
angular motion keeps the full rigid-body terms nonzero.

The terminal residual intentionally avoids acceleration that depends on the
control input; its force component uses `-fds_w`.

## 4. Frame semantics and physical wrench application

Three options were separated because they describe different physical or
visual concepts:

| Option | Meaning | Default |
| --- | --- | --- |
| `--controller-state-frame {cog,ee}` | Kinematic point used in the force-impedance controller | `ee` |
| `--wrench-application-point {cog,ee}` | Point at which the environment physically applies the simulated wrench | `ee` |
| `--plot-state-frame {cog,ee}` | Kinematic point saved/displayed for position and velocity | `ee` |

`--interaction-frame` is retained only as a deprecated shorthand that sets the
wrench and plot frames.  Conflicting explicit values raise an error.

When a force acts at the real EE but the plant equations consume a CoG wrench,
the equivalent wrench is

```text
f_CoG^W   = f_EE^W
tau_CoG^B = p_BE^B x (R_WB^T f_EE^W) + tau_EE^B
```

Thus a constant world-frame force can create a body-frame lever torque that
changes with attitude.  This explains correlations among position response,
RPY, body angular velocity, and the plotted lever-arm torque.

For a CoG-centric controller, only the controller's acados copy of `ee_p` is set
to zero.  The plant keeps the physical value `[0, 0, 0.264]`.  Mutating the
shared physical-parameter object would incorrectly move the plant contact point
to the CoG too.

## 5. Torque compensation

The independent option is

```text
--torque-compensation {none,lever-arm,estimator}
```

It is available to all controller model types.  Defaults are `estimator` for
models 0 and 1 and `lever-arm` for model 2.  Lever-arm compensation requires an
EE wrench application point.  A proposed separate ground-truth mode was removed
because `--est_dist_type` already supplies similar functionality and duplicating
it would blur the disturbance-source semantics.

## 6. Critical acados parameter bug and fix

An earlier implementation wrote virtual impedance parameters into
`nmpc.acados_init_p[34:40]`.  This was wrong: it corrupted the EE quaternion and
predicted-disturbance fields and left the intended virtual mass at zero.  Early
plots in which “with EE acceleration” and “without EE acceleration” appeared
identical were therefore invalid.

The current parameter layout is:

| Slice | Contents |
| --- | --- |
| `0:4` | reference quaternion |
| `4:35` | 31 physical parameters, including EE pose |
| `35:41` | predicted CoG disturbance, when enabled |
| `41:47` | impedance virtual mass/inertia |

The correct code derives the impedance start dynamically:

```python
impedance_param_start = 4 + len(nmpc.phys.physical_param_list)
if nmpc.include_cog_dist_parameter:
    impedance_param_start += 6
```

Do not replace this with a literal slice.

## 7. Shared comparison scenario and saved data

Scenario name: `force-impedance-compare`; current duration: 20 s.  Historical
runs made before 2026-07-23 used 18 s.

| Time (s) | World force (N) | EE contact torque |
| --- | --- | --- |
| 0–2 | `[0, 0, 0]` | zero |
| 2–7 | `[5, 0, 0]` | zero |
| 7–12 | `[5, -5, 0]` | zero |
| 12–17 | `[5, -5, -5]` | zero |
| 17–20 | `[0, 0, 0]` | zero |

Steady-state metric windows are 6–7 s, 11–12 s, 16–17 s, and the post-release
window 19–20 s.

Structured compressed NPZ bundles store time, controller/plant states, applied
wrenches, controls, and JSON metadata.  Important arrays include `state_cog`,
`state_ee`, `state_plot`, `applied_wrench_at_point`, and
`applied_wrench_cog`.  `applied_wrench_ee` remains as a legacy-compatible alias.
Metadata records scenario, controller/wrench/plot frames, controller and plant
EE positions, reference transform, acceleration mode, duration, virtual M/D/K,
and `enlarge_factor`.  Preserve the parameters in the batch name as well.

`plot_force_impedance_comparison.py` checks that the scenario and M/D/K match,
baseline-subtracts position using 1.5–2.0 s, and emits:

- `<prefix>.png` and `<prefix>.pdf`: force plus XYZ displacement/velocity;
- `<prefix>_metrics.csv`: RMSE and maximum errors;
- `<prefix>_rotational_diagnostics.png/.pdf`: force, body RPY, body angular
  velocity, and lever-arm body torque.

The plotter uses `state_plot`, falling back to the legacy `state_ee` key for old
files.  Labels are selected from metadata.

## 8. Reproduction workflow

Run commands from the `nmpc` directory.  Replace `UNIQUE` with a label containing
the actual K, `enlarge_factor`, acceleration mode, frames, and date.

First inspect the active configuration:

```bash
rg -n 'enlarge_factor|pMxy|pMz|Qv_xy|Qv_z|Qp_xy|Qp_z' \
  ../../../robots/beetle_omni/config/BeetleNMPCFullServoForceImp.yaml
```

Generate ideal truth for those M/D/K values:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_impedance_only.py --sim_model 1 -p 4 \
  --scenario force-impedance-compare \
  --save-run experiment_results/impedance/paper/BATCH/data/nominal_UNIQUE.npz
```

Generate an EE-centric run with full EE acceleration:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_ee_force_impedance_nmpc.py 2 -e 0 -p 4 \
  --controller-state-frame ee \
  --wrench-application-point ee \
  --plot-state-frame ee \
  --torque-compensation lever-arm \
  --ee-acceleration full \
  --scenario force-impedance-compare \
  --save-run experiment_results/impedance/paper/BATCH/data/nmpc_ee_UNIQUE.npz
```

For an otherwise matching run using CoG linear acceleration, change only:

```text
--ee-acceleration cog
```

Generate the current CoG-controller/EE-load/EE-display case.  The simulator
performs the external EE-to-CoG reference conversion automatically:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 sim_ee_force_impedance_nmpc.py 2 -e 0 -p 4 \
  --controller-state-frame cog \
  --wrench-application-point ee \
  --plot-state-frame ee \
  --torque-compensation lever-arm \
  --ee-acceleration cog \
  --scenario force-impedance-compare \
  --save-run experiment_results/impedance/paper/BATCH/data/nmpc_cog_UNIQUE.npz
```

Plot either run:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/nmpc_force_imp_mpl \
python3 plot_force_impedance_comparison.py \
  --nmpc experiment_results/impedance/paper/BATCH/data/NMPC_RUN.npz \
  --truth experiment_results/impedance/paper/BATCH/data/TRUTH_RUN.npz \
  --run-label 'K=..., enlarge_factor=...' \
  --output-prefix experiment_results/impedance/paper/BATCH/figures/COMPARISON \
  --metrics-path experiment_results/impedance/paper/BATCH/metrics/COMPARISON.csv
```

## 9. Recorded experiment results

These metrics are whole-run RMSE.  Values are listed as XYZ.  They describe
particular saved runs, not guaranteed behavior under the current YAML.

| Setup | Position RMSE (m) | Velocity RMSE (m/s) | Observation |
| --- | --- | --- | --- |
| K20, ef2, EE full acceleration | `0.00857, 0.01633, 0.00885` | `0.03336, 0.08621, 0.02124` | Persistent rotational coupling visible |
| K20, ef2, EE controller with CoG acceleration | `0.02144, 0.01183, 0.00386` | `0.03281, 0.03926, 0.01797` | Different local response; not equivalent in motion |
| K20, ef8, EE full acceleration | `0.02209, 0.04318, 0.01938` | `0.06644, 0.22326, 0.05829` | Larger weight was substantially more oscillatory |
| K20, ef8, EE controller with CoG acceleration | `0.01824, 0.00937, 0.00526` | `0.03624, 0.05023, 0.02046` | Better than full mode in this run |
| K6, ef2, EE full acceleration | `0.01214, 0.01487, 0.02078` | `0.03067, 0.05953, 0.01729` | Lower stiffness test |
| K6, ef2, EE controller with CoG acceleration | `0.05633, 0.02834, 0.00705` | `0.03289, 0.03590, 0.01568` | Large translational offset in X/Y |
| K20, ef2, CoG controller, EE load, CoG plot | `0.00659, 0.00389, 0.00279` | `0.01299, 0.01686, 0.01114` | Closest recorded translational match |

The main conclusion is that increasing `enlarge_factor` did not monotonically
move the NMPC closer to ideal impedance.  With full EE acceleration it amplified
oscillation in the recorded ef8 case.  The latest CoG-centric controller with a
physically correct EE load greatly reduced translational error while retaining
the expected attitude response to lever-arm torque.

## 10. Verified run from 2026-07-22 (historical)

At the time of the latest run, the YAML contained:

```text
enlarge_factor = 2
M = [1.5, 1.5, 1.5]
D = [10, 10, 10]
K = [20, 20, 20]
```

Because this file was edited repeatedly, these are historical values and must
not be assumed without re-reading the YAML.

Latest run bundle:

```text
sim_data/comparison/nmpc_force_impedance_cog_controller_ee_load_cog_plot_k20_ef2_20260722.npz
```

Truth bundle:

```text
sim_data/comparison/nominal_force_impedance.npz
```

Outputs:

```text
experiment_results/force_impedance_comparison_cog_controller_ee_load_cog_plot_k20_ef2_20260722.png
experiment_results/force_impedance_comparison_cog_controller_ee_load_cog_plot_k20_ef2_20260722.pdf
experiment_results/force_impedance_comparison_cog_controller_ee_load_cog_plot_k20_ef2_20260722_metrics.csv
experiment_results/force_impedance_comparison_cog_controller_ee_load_cog_plot_k20_ef2_20260722_rotational_diagnostics.png
experiment_results/force_impedance_comparison_cog_controller_ee_load_cog_plot_k20_ef2_20260722_rotational_diagnostics.pdf
```

Maximum position errors were `0.02076, 0.02161, 0.01140 m`; maximum velocity
errors were `0.08699, 0.10315, 0.06667 m/s`.

Validation performed on this run:

- metadata frames were controller `cog`, wrench `ee`, plot `cog`;
- controller `ee_p` was `[0, 0, 0]`, plant `ee_p` was `[0, 0, 0.264]`;
- all 3601 state samples and 3600 input samples were finite;
- `state_plot` exactly equaled `state_cog`;
- force conversion error was zero to numerical precision;
- torque compensation exactly equaled the equivalent applied CoG torque;
- at 2 s the equivalent wrench was approximately
  `[5, 0, 0, 0, 1.32, 0]`;
- Python compilation and `git diff --check` passed.

## 11. Cross-platform notes and pitfalls

- Do not embed the absolute workspace path in scripts or saved metadata logic.
  Clone/source the ROS workspace on the target platform and run from this
  directory.
- Ensure the ROS package paths and acados environment are available.  In
  particular, `ACADOS_SOURCE_DIR` and generated model libraries must correspond
  to the target machine.
- Use `MPLBACKEND=Agg` for headless runs.  Set `MPLCONFIGDIR` to any writable
  temporary directory; `/tmp/...` is Linux-specific and should be replaced on
  platforms without `/tmp`.
- NPZ and CSV are the portable experiment artifacts.  Generated acados binaries
  are platform-specific and should be regenerated rather than copied blindly.
- Never compare a truth run and NMPC run merely by filename.  The plotting script
  checks M/D/K metadata for this reason.
- A run initially intended as K6 was discovered to have used the then-current
  K20 YAML and was renamed before plotting.  Always inspect metadata before
  publishing a result.
- Keep unique names.  The user explicitly wants earlier figures preserved while
  parameters are varied.

## 12. Suggested next experiments

For a clean ablation, hold the scenario, plant, M/D/K, horizon, solver settings,
and torque compensation constant, then vary exactly one of:

1. controller state frame (`ee` versus `cog`);
2. acceleration expression (`full` versus `cog`) for an EE controller;
3. `enlarge_factor`;
4. stiffness K.

Record each run's metadata and use the same truth bundle only when M/D/K match.
Inspect both the translational plot and rotational diagnostics before assigning
a cause to a steady-state error or oscillation.

## 13. Paper workflow update (2026-07-23)

The formal comparison workflow now uses a 20 s scenario and stores artifacts
under `experiment_results/impedance/paper/<batch>/` with separate `data`,
`figures`, and `metrics` directories.  When `--save-run` is omitted for the
comparison scenario, both simulators generate a unique parameterized NPZ name
under the common paper data directory.

The main plot uses SciencePlots with all configured fonts at least 14 pt.  Its
4x2 layout is:

```text
EE applied force       | CoG lever-arm torque
EE x position          | EE x velocity
EE y position          | EE y velocity
EE z position          | EE z velocity
```

The current LaTeX labels use left superscript `T` for EE quantities and `B` for
CoG/body quantities.  The old rotational-diagnostics plot is optional via
`--rotational-diagnostics`.

For the CoG-controller case, the planning interface remains EE-centric and the
simulator converts the pose externally:

```text
R_WB = R_WT R_BE^T
p_WB = p_WT - R_WB p_BE
```

Both the CoG- and EE-controller plant states now initialize the CoG at
`-p_BE`, making initial EE position and velocity exactly zero.  The first paper
batch is documented in
`experiment_results/impedance/paper/m1p5_d10_k20_ef2_20260723/README.md`.
