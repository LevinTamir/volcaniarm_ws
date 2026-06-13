# volcaniarm_controller

ros2_control plugins for the Volcaniarm 5-bar planar arm.

## Available controllers

### `volcaniarm_controller/ClosedLoopTrajectoryController`

Drop-in replacement for the upstream `joint_trajectory_controller` that
also publishes the passive joint angles of the 5-bar linkage (derived
from the two actuated elbow joints) so RViz, TF, and downstream
consumers see a complete joint state. Use this when you want
deterministic trajectory tracking with a known IK plan, e.g. the
calibration accuracy/repeatability sweeps and the joystick teleop.

Config: [`config/volcaniarm_controllers.yaml`](config/volcaniarm_controllers.yaml).

### `volcaniarm_controller/RLPolicyController`

Inference-time runtime for an Isaac-Lab-trained reinforcement-learning
policy exported as ONNX. Subscribes to a target EE pose, builds the
`Volcaniarm-Reach-v0` observation vector, runs the policy, and writes
position targets to the two elbow joints.

**Observation contract** (matches training, order matters):

```
[ joint_pos_rel (N=2),    # current_q - default_q
  pose_command (7),       # x, y, z, qw, qx, qy, qz in target_frame
  last_action (N=2) ]     # raw policy output from the previous tick
```

**Action contract**:

```
target_q[i] = default_joint_positions[i] + action_scale * raw_action[i]
```

then optionally clamped per-joint and EMA-smoothed (see safety params
below). Defaults to `action_scale=0.5` to match the training
`JointPositionActionCfg(scale=0.5)`.

**Stub mode**: if `model_path` is empty or the ONNX file fails to
load, the controller runs but every tick commands `default_joint_positions`
(arm holds at home). Useful for wiring tests on real hardware before a
trained policy is available.

**Safety polish** (all opt-in, defaults preserve raw policy
behaviour):

| Param                    | Default | Effect                                      |
|--------------------------|---------|---------------------------------------------|
| `joint_position_min/max` | `[]`    | Per-joint position clamps after scaling.    |
| `action_smoothing_alpha` | `1.0`   | EMA on raw actions; `<1.0` damps chatter.   |
| `target_max_age_s`       | `1.0`   | Hold default if latest target older than this.|

NaN/Inf in the policy output triggers an automatic fallback to the
default pose for that tick. The "no target ever received" startup case
also holds default to avoid out-of-distribution inference.

Config: [`config/volcaniarm_rl_controller.yaml`](config/volcaniarm_rl_controller.yaml).
Model: drop the trained `policy.onnx` under [`models/`](models/) (or
point `model_path` elsewhere via `$(find-pkg-share <pkg>)`).

## Bringup

The bringup launches expose `controller:=traj|policy|all`:

```bash
# Just the trajectory controller (default)
ros2 launch volcaniarm_bringup sim_bringup.launch.py controller:=traj

# Just the RL policy controller
ros2 launch volcaniarm_bringup sim_bringup.launch.py controller:=policy

# Both loaded; trajectory active, policy inactive (claim with
# `ros2 control switch_controllers ...` to flip them at runtime)
ros2 launch volcaniarm_bringup sim_bringup.launch.py controller:=all
```

`real_bringup.launch.py` accepts the same arg.
