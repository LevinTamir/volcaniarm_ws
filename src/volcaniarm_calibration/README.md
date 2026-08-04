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

| Step                 | What it does                                              | Protocol default        |
|----------------------|-----------------------------------------------------------|-------------------------|
| Camera Localization  | Measure the camera pose relative to the arm base.         | before every session    |
| `static_accuracy`    | Single goal × N iterations, returns to initial each time. | 30 cycles, 3+ runs      |
| `repeatability`      | Same goal, gated on a tag-confirmed home between visits.  | 30 cycles, 3+ runs      |
| `workspace_coverage` | Sweeps a goal grid N times across the envelope.           | 9 goals × 3 sweeps, 3+ runs |

Experiment 0 adds three test types, each with its own dashboard page
and also runnable headless (`ros2 run volcaniarm_calibration
accuracy_test`, modes documented in
[config/exp0_params.yaml](config/exp0_params.yaml)):

| Test            | What it does                                                              |
|-----------------|---------------------------------------------------------------------------|
| `noise_gate`    | Static burst (~500 samples) at a few poses: measurement noise floor. Exp0's blocking validation gate. |
| `settle_probe`  | Zero settle + timestamped burst after each move: measures the true settle time. |
| `backlash`      | Each goal approached from -Y and +Y via capture-free pre-points; rows tagged with `approach`. |

Exp0 additions to the runner: per-visit multi-sampling
(`samples_per_capture`), sweep `pass_id` metadata, `skip_visits`
resume for interrupted sweeps, and `goals_source: grid` (serpentine
grid over a task rectangle, filtered in-process with IK validity,
joint limits, and closure margin -- see `volcaniarm_calibration/grid.py`).

The full operating procedure (cycle/run counts and why, the tag
mount bias, aggregation rules) is in
[CALIBRATION.md](CALIBRATION.md); the Exp0 step-by-step pipeline is
`<ws>/experiments/RUNBOOK.md`. Analysis lives in the separate
experiments repo (nested at `<ws>/experiments/`, its own private git
repo): run the tests here, then evaluate with
`experiments/notebooks/exp0_report.py`.

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

Each run writes to the workspace-level `experiments/` tree (outside
`src/`, invisible to colcon):

```
<ws>/experiments/data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/
  config.yaml           run config + git SHA + status
  tag_observations.csv  ground-truth base→ee transform per sample
  fk_poses.csv          analytical FK at each visited goal
```

