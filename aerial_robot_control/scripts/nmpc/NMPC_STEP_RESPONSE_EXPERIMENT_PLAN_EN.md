# NMPC Step-Response Experiment Plan for an Omnidirectional Tiltable Quadrotor

## 1. Experimental Objective

This simulation study evaluates the six-degree-of-freedom step-response
performance of the NMPC controller at the zero-pose operating point. The
formal matrix isolates the commanded axis and command amplitude while keeping
the position and attitude tracking weights identical across all three axes.
The evaluated quantities include tracking performance, cross-axis coupling,
constraint handling, actuator usage, and real-time solver performance.

Experiments at nonzero attitude operating points remain available as an
extension, but they are not included in the formal 30-case result set reported
in this document.

## 2. Controller, Plant, and Common Configuration

The formal experiments use `model=1` in `sim_nmpc.py`, corresponding to the
`NMPCTiltQdServo` controller. The simulated plant uses `sim_model=0`,
corresponding to `NMPCTiltQdServoThrust`, which includes first-order servo and
rotor-thrust dynamics. The plant therefore contains rotor dynamics that are
not explicitly represented in the controller model.

The common experimental parameters are:

| Parameter | Value |
|:--|:--|
| Attitude workpoint, roll/pitch/yaw | $(0^\circ,0^\circ,0^\circ)$ |
| Step application time | $2~\mathrm{s}$ |
| Total simulation duration | $10~\mathrm{s}$ |
| NMPC control period | $0.01~\mathrm{s}$ |
| Simulation integration period | $0.001~\mathrm{s}$ |
| Steady-state evaluation window | $9$--$10~\mathrm{s}$ |
| Dynamic evaluation window | $2$--$10~\mathrm{s}$ |
| Linear-velocity bounds | $[-5,5]~\mathrm{m/s}$ |
| Position weights | $Q_{p,x}=Q_{p,y}=Q_{p,z}=300$ |
| Attitude weights | $Q_{q,x}=Q_{q,y}=Q_{q,z}=300$ |
| Velocity weights | $Q_{v,x}=Q_{v,y}=Q_{v,z}=10$ |
| Angular-velocity weights | $Q_{\omega,x}=Q_{\omega,y}=Q_{\omega,z}=5$ |
| Repetitions | One deterministic run per condition |

Each experiment follows the same procedure:

1. Initialize the plant and controller at the zero-pose equilibrium.
2. Hold the reference constant for the first $2~\mathrm{s}$ and verify
   pre-step stability.
3. Apply a positive single-axis step at $t=2~\mathrm{s}$.
4. Hold the post-step reference until $t=10~\mathrm{s}$.
5. Save the plant and controller states, references, control inputs,
   constraints, and solver timing information.

## 3. Formal Base30 Experimental Matrix

The formal matrix is produced by `base30_cases()` in
`nmpc_tilt_mt/utils/step_response_experiment.py`. It contains:

* Position steps along $x$, $y$, and $z$, with amplitudes of
  $0.2$, $0.4$, $0.6$, $0.8$, and $1.0~\mathrm{m}$.
* Attitude steps about roll, pitch, and yaw, with amplitudes of
  $10^\circ$, $30^\circ$, $50^\circ$, $70^\circ$, and $90^\circ$.

The total number of cases is

$$
3\ \text{position axes}\times5\ \text{amplitudes}
+
3\ \text{attitude axes}\times5\ \text{amplitudes}
=30.
$$

Attitude increments are implemented through rotation composition rather than
direct addition to an Euler-angle component:

$$
R_{\mathrm{ref},+}
=R_{\mathrm{wp}}
\exp\!\left(\Delta\theta[\mathbf e_i]_\times\right),
\qquad i\in\{x,y,z\}.
$$

At the zero-pose workpoint this is equivalent to a rotation of
$\Delta\theta$ about the selected roll, pitch, or yaw axis. This formulation
also remains well defined when the same implementation is used at nonzero
attitude workpoints. Euler angles are used for case definition and plotting;
the saved attitude command is quaternion based.

## 4. Evaluation Metrics

For the commanded axis, the implementation reports rise time, settling time,
percentage overshoot, steady-state error, RMSE, and IAE. Let $y_0$ be the mean
response over the final, at most, 100 pre-step samples, and let
$\Delta r=r_1-r_0$ be the commanded step amplitude. The normalized progress
used by the current implementation is

$$
p(t)=\frac{y(t)-y_0}{\Delta r}.
$$

The 10--90% rise time is

$$
t_r=t_{90}-t_{10},
\qquad
t_\alpha=\min\{t\mid p(t)\geq\alpha\},
\quad \alpha\in\{0.1,0.9\},
$$

where threshold-crossing times are linearly interpolated between samples.
The 2% settling time is the earliest time after the command at which the
response enters and remains inside

$$
|y(t)-r_1|\leq0.02|\Delta r|
$$

until the end of the recorded interval. If a required threshold is not
reached, the corresponding time is reported as `NaN`.

For attitude steps, the scalar response is obtained from the commanded-axis
component of

$$
\operatorname{Log}\!\left(R_{\mathrm{wp}}^\mathsf{T}R(t)\right).
$$

The remaining reported metrics are:

* maximum deviation and RMSE over the five noncommanded axes;
* geodesic attitude error based on the shortest quaternion path;
* peak and RMS control effort, control-input slew rate, and saturation ratio;
* minimum state and input constraint margins and violation counts;
* mean, 95th-percentile, and worst-case NMPC solution times;
* solver failures and control-deadline misses.

## 5. Completed Base30 Results

The completed result set is stored in:

`experiment_results/nmpc_step_response_base30_wp000_v5_Qp300_Qq300_20260723_130318`

All 30 cases completed successfully and passed the pre-step stability check.
There were no solver failures, state-constraint violations, or input-constraint
violations. One solver-reported control-deadline miss occurred in the
$50^\circ$ yaw case: the worst solution time was $13.008~\mathrm{ms}$ versus
the $10~\mathrm{ms}$ control period.

The following values are arithmetic means over the five amplitudes of each
commanded axis:

| Axis | Cases | Mean rise time [s] | Mean settling time [s] | Mean overshoot [%] |
|:--|--:|--:|--:|--:|
| $x$ | 5 | 0.712230 | 1.6774 | 2.555761 |
| $y$ | 5 | 0.699565 | 1.7282 | 2.830680 |
| $z$ | 5 | 0.579996 | 1.3570 | 2.535416 |
| Roll | 5 | 0.911618 | 1.8812 | 0.000034 |
| Pitch | 5 | 0.885894 | 1.8908 | 0.000470 |
| Yaw | 5 | 0.682945 | 1.1534 | 0.258546 |

Roll and pitch show essentially zero overshoot. The mean yaw overshoot is
$0.2585\%$, which is also small. Among the attitude axes, yaw has the shortest
mean rise and settling times. Among the position axes, $z$ has the shortest
mean rise and settling times.

The result directory contains:

* `runs/*.npz`: complete time-series data and metadata for all 30 cases;
* `logs/*.log`: per-case simulation logs;
* `round_times/*.csv`: per-control-update solver timings;
* `manifest.json`: the Base30 run manifest;
* `analysis/cases/*.{png,pdf}`: per-case response figures;
* `analysis/position_metric_summary.{png,pdf}` and
  `analysis/attitude_metric_summary.{png,pdf}`: axis-wise summary figures;
* `analysis/step_response_metrics.{csv,json}`: detailed per-case metrics;
* `analysis/axis_average_metrics.csv`: the axis-wise averages reported above.

## 6. Potential Extensions

The expanded `base90_cases()` matrix additionally contains the same 30 steps
at each of the nonzero attitude workpoints
$(30^\circ,0^\circ,0^\circ)$ and $(30^\circ,30^\circ,0^\circ)$. Those 60
cases can be used to evaluate attitude-dependent coupling and available
actuator authority, but they must use the same velocity limits and weighting
configuration before being compared directly with the Base30 results.

Further extensions may include negative-direction steps, model uncertainty,
external disturbances, and repeated trials with stochastic perturbations.
