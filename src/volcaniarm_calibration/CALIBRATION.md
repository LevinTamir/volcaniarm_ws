# Calibration tests: operator manual and protocol

This document is the full operating procedure for producing
thesis-grade accuracy and repeatability numbers with the
`volcaniarm_calibration` package: what the tests are, how to run them
from the GUI, and how to validate the results afterwards. A new user
should be able to go from a powered robot to quotable numbers using
only this file.

**Experiment 0.** The end-to-end characterization campaign (joint
limit measurement, noise gate, settle probe, grid sweeps, anchor
points, weed-position freeze) has its own step-by-step pipeline in
the experiments repo: `<ws>/experiments/RUNBOOK.md`. This file stays
the reference for what each test does and how to judge the numbers.

**Measurement principle.** An external camera observes two AprilTags,
one on `volcaniarm_base_link` and one on `right_arm_tip_link` (the
tool point). The detected base-to-tool Y-Z distance is compared
against the same quantity predicted by the URDF / analytic
kinematics, which serves as the ground truth. The headline scalar per
capture is `d_error = d_detected - d_urdf` (metres, signed). The arm
is planar, so all metrics live in the Y-Z plane.

## 1. Physical setup

- Camera on its stand roughly 1.5 m in front of the arm, facing the
  working plane. Let it warm up a few minutes before measuring.
- Both AprilTags mounted and fully visible from the camera: base tag
  (ID 5) on its bracket at the base, EE tag (ID 20) on the tool
  bracket, parallel to the Y-Z plane.
- Measure the printed tag's black square with calipers and pass it at
  launch as `tag_size:=<metres>` if it differs from the 64 mm
  default. A tag-size error scales the depth estimate.

## 2. Launch

Two terminals:

```bash
# Terminal 1: robot + camera + AprilTag detector + RViz + TF
ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=tests calibration:=true
```

```bash
# Terminal 2: the calibration GUI
ros2 launch volcaniarm_calibration calibration_gui.launch.py
```

NOTE: calibration is done on real-hardware only.

## 3. GUI tour

The dashboard is laid out like the MoveIt Setup Assistant: a sidebar
on the left selects a step, the right panel shows that step's
controls, and a shared strip at the bottom holds the status line, the
log, and the post-run banner.

Sidebar steps:

| Step                | Purpose                                            |
|---------------------|----------------------------------------------------|
| Start               | Home the robot (limit-switch homing).              |
| Camera Localization | Solve and save the camera pose relative to the arm.|
| Static Accuracy     | Accuracy test at a single goal.                    |
| Repeatability       | Repeatability test at a single goal.               |
| Workspace Coverage  | Grid sweep for the Y-Z performance maps.           |

Each test page contains, top to bottom:

- **Protocol note** (gray text): the recommended cycle and run counts
  for that test.
- **Completed runs saved: N**: how many completed runs of this test
  already exist on disk. Refreshes after every run; use it to track
  progress toward the 3-run protocol target.
- **Test configuration**: the iterations spinbox. On the workspace
  page it is labelled "cycles (full sweeps)".
- **Home-confirm gate** (repeatability page only): checkbox plus
  tolerance / hold / timeout. See the repeatability section.
- **Initial pose (y, z)**: where the arm parks before, between, and
  after visits. The small buttons snap each axis to the arm's home
  FK. **Move to initial** drives the arm there without starting a
  run; use it to confirm the pose is safe.
- **Goal pose** (single goal) or **Goals list** (workspace page; one
  `y, z` per line, `#` starts a comment line).
- **Reachability line**: every pose edit is checked against the
  arm's kinematics after a short debounce. Green means all poses are
  reachable; red lists the offending pose(s). Start refuses to launch
  while anything is red.
- **Capture settings**: settle time (wait after each motion before
  measuring, default 2 s), detection fresh window (maximum accepted
  age of a detection, default 0.5 s), detection timeout (abort budget
  for a fresh detection after settle, default 5 s, capped at 15 s so a
  long timeout cannot mask a marginal detection setup), auto-continue
  and its fresh-hold time (advance automatically once detection has
  stayed fresh that long; uncheck to require a manual Continue click
  at every goal).
- **Run controls**: Start Run / Continue / Reset / Cancel, the
  detection label (green "fresh" or red "not visible"), and the
  progress bar (one tick per capture).

Button semantics:

- **Start Run**: validates, then runs the whole test automatically.
- **Continue**: only needed when auto-continue is off; enabled once
  detection is fresh at the current goal.
- **Reset**: abort the run, then drive the arm back to the typed
  initial pose.
- **Cancel**: emergency stop; the arm halts in place.

After every run a banner shows COMPLETED / CANCELED / FAILED with the
run directory and four actions: **Keep**, **Resume run** (failed runs
only: continues the same run from its first incomplete cycle,
appending to the same data files under the original settings, so the
already-captured cycles are not lost), **Delete run** (removes the
directory, for aborted or junk runs), and **Open notebook** (opens
the matching analysis notebook). A resumed run that finishes counts
as one completed run of the full cycle count; config.yaml records
the resume timestamps. For a multi-goal sweep, an interrupted cycle
is repeated in full on resume, so its already-captured goals gain an
extra sample; harmless for the aggregated metrics.

## 4. Session checklist (before any test)

1. **Home the robot** (Start page) so joint zeros come from the limit
   switches, not from wherever the arm powered on.
2. **Camera localization**, whenever the camera or either tag bracket
   has moved since the last saved localization:
   - On the Camera Localization page, check the detected mode label
     (stand vs on-robot is read from the URDF), then click
     **Calibrate camera**. The arm sweeps the EE tag through a set of
     poses and solves the camera pose.
   - The solve only previews: review the residual RMS in the result
     label (and the overlay of detected vs predicted tag corners in
     RViz) before **Save & apply**. RMS at the few-millimetre level
     is normal; tens of millimetres means something moved mid-sweep.
   - Saving writes `config/camera_pose.yaml`; **relaunch Terminal 1**
     so the new camera pose loads.
3. Sanity check on any test page: the detection label goes green when
   both tags resolve. If it stays red, see Troubleshooting.

## 5. Running the tests

Common to all tests: the GUI defaults implement the protocol; the
spinboxes stay adjustable for quick checks. Between independent runs
of the same test, **re-home the robot** and start fresh, so the runs
genuinely sample session-to-session variation instead of being one
long run split in three. Each run is saved on Keep; delete failed or
interrupted runs from the banner so they never pollute the analysis
(the notebooks only aggregate `status: completed` runs anyway).

### 5.1 Static accuracy

*What it measures:* the residual between measured and predicted
base-to-tool distance at one pose, split into systematic bias and
random precision.

1. Open the Static Accuracy page.
2. Set the initial pose (home-snap buttons are fine) and the goal
   pose. Check the reachability line is green.
3. Iterations: **30** (default). ISO 9283:1998 uses 30 cycles per
   pose, which keeps the numbers comparable to the literature.
4. Leave capture settings at their defaults unless detections are
   slow (then raise the fresh window slightly).
5. Start Run. The arm visits the goal, returns to initial, and
   repeats; with auto-continue on it is fully hands-off
   (a 30-cycle run takes roughly 5 to 10 minutes).
6. Repeat for **3 runs minimum, 5 preferred**, re-homing between
   runs. At 3 runs the t factor on the across-run CI is 4.30; at 5
   runs it is 2.78, so extra runs tighten the headline quickly.

### 5.2 Repeatability

*What it measures:* the scatter of attained positions at one goal
(ISO 9283 RP). Bias-free, so it is meaningful even with placeholder
tag mounts.

1. Open the Repeatability page; set initial pose and the single goal
   (the test refuses multiple goals by design).
2. Iterations: **30** (default). Runs: **3 or more**, re-homed
   between.
3. **Home-confirm gate** (optional): when enabled, after every
   return-to-initial the runner waits until the detected tool
   position agrees with the URDF prediction within the tolerance for
   the configured number of consecutive fresh frames, so every cycle
   provably starts from the same physical state. While the tag
   mounts carry the placeholder bias, the gate only passes with the
   tolerance above that bias (whatever mean `d_error` static accuracy
   reports is the floor; the default 80 mm covers it). After the
   mount bias is removed, drop the tolerance toward 10 mm.
4. Start Run; hands-off as above.

### 5.3 Workspace coverage

*What it measures:* how accuracy and repeatability vary over the Y-Z
working plane; feeds the per-point maps.

1. Open the Workspace Coverage page and paste the grid into the goals
   box:

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

   The reachability line validates every row; a typo shows up as a
   red line number before anything moves.
2. Cycles (full sweeps): **3** (default). Each cycle walks the whole
   grid in order without returning home between goals, so three
   cycles leave a 3-sample cluster at every point.
3. Start Run. Repeat for **3 or more runs**, re-homed between.
4. The grid deviates deliberately from ISO 9283's five poses on a
   cube diagonal: the arm is planar, so a plane-filling grid is the
   meaningful envelope. State that deviation in the thesis.

## 6. Validating the results (report notebook)

Evaluation lives in the experiments repo, outside this package:
`<ws>/experiments/notebooks/exp0_report.py` (percent-format, runs
directly in VSCode). After recording runs, execute it top to bottom -
each section auto-discovers every matching run under
`experiments/data/`, prints its verdict/metrics, and writes figures +
`exp0_summary.md` to `experiments/figures/`.

Sections: 0 noise-gate PASS/FAIL, 1 settle time, 2 sweep accuracy
maps + metrics (P0-2/3/4), 3 anchor AP+RP (P0-5/6), 4 systematic vs
random + measured-vs-analytical overlay (P0-7), 5 frozen weed
positions, 6 summary export. `workspace_figures.py` alongside it
produces the analytic task-region set (P0-1, P0-8a/b/c).

Run selection defaults to auto-discovery (interrupted runs included:
their captured rows are valid data and resumed sweeps reassemble by
pass id); pin the `*_RUNS` selectors at the top of the notebook to an
explicit run list for the final thesis figures so they are
reproducible.

What "validated" looks like:

- The discovered run/pass counts printed by each section match what
  you actually recorded.
- Noise gate: PASS (worst axis <= 2 mm after the 30-frame median).
- Sweep: passes agree per point (small between-pass spread means the
  error is systematic and calibratable; section 4 quantifies the
  split). Quote mean/RMSE/percentiles and the success rates at
  10/15/20 mm.
- Anchors: quote AP and RP per point; with repeat sessions, the
  within vs between decomposition separates short-term repeatability
  from day-to-day drift. Compare against the ~10 mm weeding
  tolerance.

## 7. Mount bias

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

Until this is done, report the standard deviation as the positioning
precision and the mean as a fiducial modelling artefact; afterwards
the mean residual is the absolute accuracy.

## 8. Aggregation rules (what the analysis enforces)

- Sweep passes are pooled by the `pass_id` recorded in each run's
  `config.yaml`, with keep-last dedupe per grid point, so an
  interrupted pass plus its resume run reassemble into one pass.
- Anchor sessions are pooled by commanded position (rounded to the
  millimetre); repeat sessions add cycles and unlock the
  within/between-session decomposition.
- Every run's `config.yaml` records the URDF mount snapshot
  (`urdf_mounts`); do not mix runs recorded before/after a mount
  recalibration in one statistic - pin the run selectors instead.

## 9. Deviations from ISO 9283 to state in the thesis

- Measurement by fiducial camera tracking instead of a laser tracker;
  the detector noise floor (sub-millimetre at this standoff) bounds
  the resolvable repeatability.
- No rated-load / rated-speed conditioning; tests run unloaded at the
  default trajectory speed.
- Metrics are planar (world Y-Z projection) because the manipulator
  has 2 degrees of freedom.
- Workspace characterisation uses a plane grid instead of the cube
  diagonal.

## 10. Data layout

Runs are written to the experiments repo, outside this package:

```
<ws>/experiments/data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/
  config.yaml           run config + git SHA + mount snapshot + status
  tag_observations.csv  one row per sample (world Y-Z origins,
                        d_detected / d_urdf / d_error, tip quaternion,
                        visit label + approach tag)
  fk_poses.csv          analytic FK at each captured visit
```

`config.yaml` records the final status (`completed`, `canceled`,
`failed` with a `failure_reason`) plus the Exp0 metadata
(`pass_id`, `session_note`, `samples_per_capture`, `skip_visits`).
The report notebook auto-discovers interrupted runs too - their
captured rows are valid data and resumed sweeps reassemble by pass
id.

## 11. Troubleshooting

- **Detection label stays red**: check both tags are unobstructed and
  inside the camera frame at the current pose; check the apriltag
  node is up (`ros2 topic hz /tf` from the detector). Raise the
  detection fresh window if detections arrive but age out.
- **Run fails with "detection lost during sampling"**: the EE tag was
  not seen within the detection timeout after the arm settled at a
  goal, usually an occlusion, an edge-on viewing angle, or a gappy
  detector (check its rate with `ros2 topic hz`). Click **Resume
  run** on the banner to keep the captured cycles and finish the run.
  Raise the detection timeout spinbox for a gappy detector; if double
  digits are needed, fix lighting / exposure / tag angle instead of
  hiding it behind a longer timeout.
- **Home-confirm keeps timing out** (repeatability): the tolerance is
  below the current mount bias. Raise the "Y-Z segment tol" spinbox
  above the mean `d_error` a static accuracy run reports, or remove
  the mount bias first (section 7).
- **Start refuses with "unreachable pose"**: the pose has no IK
  solution (outside the linkage envelope). The red reachability line
  names the offending pose or goals-list line.
- **`lookup_transform(world, apriltag_marker_*)` fails at run start**:
  the camera pose is not localized yet; run Camera Localization
  first.
- **A notebook excludes runs you expected to aggregate**: read the
  printed reason. Different goals means the runs really are not
  comparable; different mount keys with unchanged physical mounts is
  the `ALLOW_MOUNT_KEYS` case.
