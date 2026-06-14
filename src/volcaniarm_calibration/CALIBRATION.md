# Camera-pose calibration workflow (Phase 4)

Human-in-the-loop loop: **solve → review a live preview → Save & apply → relaunch**.
A solve never writes anything; you review it, then explicitly persist it.

## Launch

```bash
# stand camera (tests) or on-robot camera (work); calibration:=true loads the dashboard
ros2 launch volcaniarm_bringup sim_bringup.launch.py  mode:=tests calibration:=true   # sim
ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=work  calibration:=true   # real
```

Mode (stand vs on-robot) is auto-detected from the URDF (parent of `camera_link`).

## The loop

1. **Preflight** — the dashboard checks the visible AprilTag set against the
   detected camera placement and refuses to start on a mismatch (e.g. the base
   tag visible while in on-robot mode → camera is probably on the stand).
2. **Calibrate camera** — sweeps the EE tag through `calibration_poses.yaml` and
   solves the camera pose. This only PREVIEWS:
   - `camera_link_calibrated` TF (orange) appears next to the current `camera_link`
     in RViz — the correction, literally, side by side.
   - `/calibration_overlay` image shows DETECTED tag corners (green) vs
     FK-PREDICTED corners (red) under the candidate calibration, with per-corner
     pixel error + RMS. Overlap within ~1-2 px across the FoV = good.
   - The review label shows RMS / max / pose count and the delta vs the applied
     pose, and warns if RMS is worse than the last saved run.
3. **Save & apply** — writes `config/camera_pose.yaml` and appends
   `config/camera_pose_history.yaml` (timestamp, git SHA, mode, residuals).
   Takes effect on the next launch (the existing camera_pose → xacro pipeline).

## Work mode: rail decomposition

In on-robot mode the solve also reports a suggested **`camera_mount_x`** (the rail
projection) and the **off-axis mount tolerance** (the Y-Z mounting error). Set the
rail with `camera_mount_x:=<suggested>` on the next launch; the residual stays in
`camera_*` (`camera_joint`). (Informational this iteration — validate on hardware
before relying on the number.)

## Accuracy hygiene (real hardware)

- Use the 200 mm tag when standoff allows; view obliquely (15-40°); reject
  ambiguous PnP flips.
- **Measure** the printed tag's black square with calipers and pass it as
  `tag_size:=` (depth error scales ~linearly with size error).
- Replace the base-tag bracket offset placeholder in
  `volcaniarm_description/urdf/volcaniarm_apriltag.xacro` with the measured value.
- Calibrate the RealSense color intrinsics and ensure apriltag_ros uses the same
  `camera_info`; let the camera warm up before a run.

## tests-mode cross-check (TODO, hardware-dependent)

With the stand camera, also locate a tag on the camera mount relative to the arm
and compare to the work-mode `camera_mount_x` (agreement within a few mm =
validated). Requires a physical camera-mount tag + a URDF frame for it.

## Validate-tomorrow checklist (real robot)

- Overlay corner ordering/sign (predicted vs detected) renders correctly.
- `camera_link_calibrated` lands where expected; Save & apply → relaunch matches.
- Rail `camera_mount_x` suggestion is sane and improves the fit.
- Measured tag size + bracket offset filled in.
