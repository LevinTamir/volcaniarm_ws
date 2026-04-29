# volcaniarm_calibration

AprilTag-based calibration and accuracy testing for the Volcaniarm.

## What it does

Compares the arm's analytical FK against ground truth measured by an
external camera looking at two AprilTags: one fixed on the base, one
on the end effector. Each run records the camera-resolved
`base_tag → ee_tag` transform and the FK at the commanded joint
state, side by side, so the analysis notebooks can compute pose
error metrics.

## Tests

| Combo entry          | What it does                                              |
|----------------------|-----------------------------------------------------------|
| `static_accuracy`    | Single goal × N iterations, returns to initial each time. |
| `repeatability`      | Same as accuracy. Differs in how the data is interpreted. |
| `workspace_coverage` | Sweeps a list of goals once each across the envelope.     |

## Run it

```bash
# Sim (Gazebo + dashboard + RViz)
ros2 launch volcaniarm_bringup sim_bringup.launch.py calibration:=true

# Real hardware
ros2 launch volcaniarm_bringup real_bringup.launch.py calibration:=true
```

The rqt dashboard opens automatically. Pick a test, set the initial
and goal pose(s), click **Start Run**. At each goal the arm settles
and waits — adjust the camera (real hardware) until both markers are
visible, then click **Continue**.

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

## Headless

`ros2 run volcaniarm_calibration accuracy_test` runs the same flow
without the dashboard, auto-clicking Continue at every gate. Configure
via [`config/accuracy_test_params.yaml`](config/accuracy_test_params.yaml).
