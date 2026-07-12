# Calibration test protocol

This document is the operating procedure for producing thesis-grade
accuracy and repeatability numbers with the `volcaniarm_calibration`
package. It covers the common setup, the per-test protocol (cycle
counts, run counts, and their justification), the tag mount bias,
and the aggregation rules the notebooks follow.

Measurement principle: an external camera observes two AprilTags, one
on `volcaniarm_base_link` and one on `right_arm_tip_link`. The
detected base-to-tool Y-Z distance is compared against the same
quantity predicted by the URDF / analytic kinematics, which serves as
the ground truth. The headline scalar per capture is
`d_error = d_detected - d_urdf` (metres, signed).

## Common setup (every session)

1. Launch the robot with the detector, then the GUI in a second
   terminal:

   ```bash
   # Terminal 1
   ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=tests calibration:=true
   # Terminal 2
   ros2 launch volcaniarm_calibration calibration_gui.launch.py
   ```

2. **Home the robot** (Start page) so the joint zeros come from the
   limit switches, not from wherever the arm was powered on.
3. **Camera localization** (Camera Localization page) whenever the
   camera or either tag bracket has moved since the last saved
   localization. The solve previews before it writes: check the RMS in
   the review label and the overlay (detected corners vs predicted
   corners) before Save & apply. Re-launch afterwards so the new
   camera pose loads.
4. Sanity check: both tags detected (the detection label on any test
   page goes green when the base-to-ee tag transform is fresh).

Accuracy hygiene on real hardware: measure the printed tag's black
square with calipers and pass it as `tag_size:=`; view the tags
obliquely (15 to 40 degrees); let the camera warm up before a run.

## Per-test protocol

Every test is standalone. The GUI defaults implement the protocol
below; the spinboxes stay adjustable for quick checks. Between
independent runs of the same test: re-home the robot and let the run
start from a fresh session, so the runs actually sample
session-to-session variation instead of being one long run split in
three.

### Static accuracy

- **30 cycles per run.** ISO 9283:1998 specifies 30 cycles per pose
  for the pose-accuracy statistic; using the same count makes the
  numbers directly comparable to the robotics literature.
- **3 runs minimum, 5 preferred.** The notebook quotes the across-run
  mean with a t-based 95 % confidence interval; at 3 runs the t factor
  is 4.30 (wide), at 5 runs 2.78. More runs tighten the headline
  faster than more cycles.
- Notebook: `notebooks/static_accuracy.ipynb`.
- While the URDF tag mount values remain placeholders (see "Mount
  bias" below), the mean residual is a fiducial modelling artefact
  and is reported as such, with the standard deviation quoted as the
  positioning precision. Once the mount values are measured and fixed
  in the xacro, the mean residual is the absolute accuracy.

### Repeatability

- **30 cycles per run** (ISO 9283 defines RP over a 30-point
  cluster), **3 runs or more**.
- The headline is the mean **within-run RP** across runs with a
  t-based CI; the pooled cross-session RP is reported alongside,
  clearly labelled (it additionally contains re-homing and camera
  relock effects).
- Home-confirm gate: enable it once the mount bias is removed and
  drop the tolerance from the 80 mm default toward 10 mm. While the
  mounts carry the placeholder bias, the gate can only pass with the
  tolerance above that bias (whatever mean `d_error` static accuracy
  reports is the floor).
- Notebook: `notebooks/repeatability.ipynb`.

### Workspace coverage

- **Grid of 9 points, 3 cycles (full sweeps) per run, 3 runs or
  more.** Paste-able goals block for the dashboard:

  ```
  -0.30, 0.40
  -0.30, 0.525
  -0.30, 0.65
   0.00, 0.40
   0.00, 0.525
   0.00, 0.65
   0.30, 0.40
   0.30, 0.525
   0.30, 0.65
  ```

- Produces the Y-Z accuracy map, the repeatability map, and the
  commanded-vs-attained error-vector plot, all aggregated per grid
  point across runs.
- This deviates deliberately from ISO 9283's five poses on a cube
  diagonal: the arm is planar, so a plane-filling grid is the
  meaningful envelope. State the deviation in the thesis.
- Notebook: `notebooks/workspace_coverage.ipynb`.

## Mount bias

The URDF tag mount translations in
`volcaniarm_description/urdf/volcaniarm_apriltag.xacro` carry
placeholder values (the base bracket offset is an explicit TODO in
the file); until they match the physical brackets, every accuracy
figure contains a constant fiducial offset that has nothing to do
with the arm. To remove it:

1. Measure the physical tag-center offsets relative to their parent
   links (base tag on `volcaniarm_base_link`, EE tag on
   `right_arm_tip_link`) and update the mount `xyz` values in the
   xacro. Whatever mean `d_error` a static accuracy run reports is
   the current size of the bias.
2. Rebuild
   (`colcon build --symlink-install --packages-select volcaniarm_description`)
   and relaunch, then **re-run camera localization** (the previous
   solution absorbed the old mount values and is stale).
3. Verify with one 30-cycle static accuracy run; the mean residual
   should drop to the few-millimetre level.
4. Update the mirrored mount constants at the top of
   `volcaniarm_calibration/analysis/loader.py` (marked as a manual
   sync) so legacy tooling matches the URDF.

## Aggregation rules (what the notebooks do)

- Runs are grouped by commanded goals (matched to the millimetre) and
  by **mount version** (recorded per run in `config.yaml` under
  `urdf_mounts`; legacy runs fall back to their git SHA). Runs from
  different mount versions are **never averaged together**; merging
  legacy runs known to share mounts requires listing their keys in
  the notebook's `ALLOW_MOUNT_KEYS` parameter.
- Excluded runs are printed with the reason, so the aggregation in a
  thesis figure is auditable. Pin the exact run set with the
  `RUN_DIRS` parameter for the final figures.
- Headline statistics are across-run means with t-based 95 %
  confidence intervals (degrees of freedom = runs - 1); pooled
  per-cycle statistics feed the histograms and worst-case numbers.

## Deviations from ISO 9283 to state in the thesis

- Measurement by fiducial camera tracking instead of a laser tracker;
  the detector noise floor (sub-millimetre at this standoff) bounds
  the resolvable repeatability.
- No rated-load / rated-speed conditioning; tests run unloaded at the
  default trajectory speed.
- Metrics are planar (world Y-Z projection) because the manipulator
  has 2 degrees of freedom.
- Workspace characterisation uses a plane grid instead of the cube
  diagonal.

## Data layout

```
data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/
  config.yaml           run config + git SHA + mount snapshot + status
  tag_observations.csv  one row per capture (world Y-Z origins,
                        d_detected / d_urdf / d_error, tip quaternion)
  fk_poses.csv          analytic FK at each visited goal
```
