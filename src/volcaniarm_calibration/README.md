# volcaniarm_calibration

AprilTag-based calibration and accuracy testing for the Volcaniarm.

## What it does

Compares the arm's analytical FK against ground truth measured by an
external camera looking at two AprilTags: one fixed on the base, one
on the end effector. Each run records the camera-resolved
`base_tag → ee_tag` transform and the FK at the commanded joint
state, side by side, so the analysis notebooks can compute pose
error metrics.

Calibration is real-hardware only. The simulated arm is never
calibrated, so the sim calibration flow was removed.

## Tests (left sidebar)

| Step                 | What it does                                              |
|----------------------|-----------------------------------------------------------|
| Camera Localization  | Measure the camera pose relative to the arm base.         |
| `static_accuracy`    | Single goal × N iterations, returns to initial each time. |
| `repeatability`      | Same goal, gated on a tag-confirmed home between visits.  |
| `workspace_coverage` | Sweeps a list of goals once each across the envelope.     |

## Run it

Two terminals, MoveIt-Setup-Assistant style:

```bash
# Terminal 1: robot + camera + AprilTag detector + RViz + TF
ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=tests calibration:=true

# Terminal 2: calibration GUI
ros2 launch volcaniarm_calibration calibration_gui.launch.py
```

Pick a step in the left sidebar, set the initial and goal pose(s),
click **Start Run**. At each goal the arm settles and waits: adjust
the camera until both markers are visible, then click **Continue**
(or leave auto-continue on to advance automatically).

Buttons:
- **Move to initial** parks the arm at the typed initial pose
  without starting a run.
- **Continue** captures the snapshot once detection is fresh
  (button enables when both tags are seen).
- **Reset** aborts the run and drives the arm back to initial.
- **Cancel** stops in place.

## Data layout

Each run writes to:

```
data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/
  config.yaml           run config + git SHA + status
  tag_observations.csv  ground-truth base→ee transform per sample
  fk_poses.csv          analytical FK at each visited goal
```

