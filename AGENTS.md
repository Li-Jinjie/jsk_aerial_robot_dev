# Repository Working Notes

## Build and validation

- Validate C++ changes from the workspace root (`/home/jinjie/ros1/dragon_ws`) with `catkin build`.
- Prefer building the affected packages explicitly, for example:
  `catkin build aerial_robot_model aerial_robot_estimation aerial_robot_control --no-status`.
- A direct `cmake --build build/<package>` uses the same catkin build/devel space, but it is only a target-level
  incremental check and is not the preferred final validation.
- Commits run the repository's pre-commit hooks, which may format staged files. Do not bypass them with
  `--no-verify`. If a hook rewrites files, inspect the formatting changes, rebuild when relevant, stage the updated
  files, and rerun the commit.
- After committing, inspect `git status` and `git show --stat` to confirm that only the intended files were committed.
- `aerial_robot_control` may build successfully with existing warnings from the deprecated `register` keyword in
  `aerial_robot_estimation/sensor/gps.h` and the wrench-estimator enum/`%d` format mismatch. Do not treat these as
  regressions from EE/CoG changes or fix them unless explicitly requested.

## Scope and worktree hygiene

- The worktree may contain unrelated simulation-model changes, experiment results, CSV data, plots, PDFs, and notes.
  Preserve them and never include them in an NMPC implementation commit unless explicitly requested.
- Keep functional NMPC changes in small, focused commits. Plotting and experiment-analysis code is unrelated to the
  EE/CoG controller functionality and should be kept separate.
- Do not modify `robots/beetle_omni/launch/bringup_nmpc_omni.launch` unless the user explicitly asks for the launch
  integration step.

## NMPC controller context

- The existing controller supports an EE-centric NMPC formulation: an EE trajectory remains expressed at the EE, and
  the CoG-to-EE contact transform is passed into the NMPC model.
- The new formulation converts an EE trajectory to a CoG trajectory before it enters NMPC, so the NMPC itself always
  flies in the CoG frame.
- The eventual merge target is `develop/MPC_tilt_mt`. Its default Beetle launch behavior
  (`nmpc_mode == 0`, `BeetleNMPCFullDist.yaml`) must remain EE-centric.
- Switching between the two formulations should be explicit and backward-compatible. Avoid unconditional behavior
  changes in shared NMPC base classes.
- In trajectory handling, keep the input frame and the frame actually passed to NMPC semantically consistent. After an
  EE reference has been converted to CoG, its internal reference frame must be treated as `cog`.
- The controller parameter is `controller/nmpc/is_convert_ee_traj_to_cog`. It defaults to `false` so existing
  configurations retain the EE-centric formulation.
- When `is_convert_ee_traj_to_cog` is enabled, convert only trajectories whose input frame is `ee`. After conversion,
  set the internal reference `child_frame_id` to `cog`. For other input frames, preserve the original frame and data.
- With conversion disabled and an `ee` trajectory, continue passing the real `ee_contact` offset to the NMPC physical
  parameters. With a converted CoG trajectory, the `cog` metadata naturally selects a zero contact offset.

## Naming conventions

- Use `traj` rather than `trajectory` in identifiers and ROS parameter names for this feature.
- The public ROS boolean parameter name is `is_convert_ee_traj_to_cog`; keep this exact spelling for launch and YAML
  integration.
- The corresponding C++ member is `is_ee_traj_to_cog_conversion_enabled_`.
- Name a per-message decision as `should_...`. The current derived condition is `should_convert_ee_traj_to_cog`,
  combining the persistent configuration state with the incoming trajectory frame.

## EE/CoG conversion API

- `RobotModel::convertFromCoGToEEContactNoAcc()` is the lightweight state conversion API. Use it when only position,
  velocity, orientation, and angular velocity are available.
- `RobotModel::convertFromCoGToEEContact()` is the full conversion API. It is built on
  `convertFromCoGToEEContactNoAcc()` and additionally converts linear and angular acceleration.
- `RobotModel::convertFromEEContactToCoG()` performs the full inverse conversion, including linear and angular
  acceleration.
- Keep `aerial_robot_estimation/src/state_estimation.cpp` changes minimal. It does not have acceleration information
  for this conversion and should use `convertFromCoGToEEContactNoAcc()` instead of manufacturing zero-acceleration
  arguments.
- NMPC measurement conversion also uses the no-acceleration API when acceleration outputs are not needed.
- An EE reference trajectory contains acceleration information, so its EE-to-CoG conversion should use the full API
  and pass the converted CoG acceleration and angular acceleration to `setXrUrRef()`.
- Both conversion directions should handle a missing `ee_contact` frame safely by warning and treating the two frames
  as coincident.
