# LeRobot integration

Runs the Volcaniarm through the full [LeRobot](https://github.com/huggingface/lerobot) loop
(teleoperate, record, train, deploy) using LeRobot's own CLI. The arm is exposed to LeRobot
as a device plugin that lives in its own repo, vendored here as a git submodule:

- Submodule: `src/volcaniarm_lerobot` (repo: `LevinTamir/volcaniarm_lerobot`, package
  `lerobot_robot_volcaniarm`). It's a pip package, not a colcon one, so colcon skips it.
- Full plugin docs (robot/camera/teleop, every CLI flag): see that repo's `README.md`.

## Why a submodule + conda

LeRobot pulls in torch and a large dependency tree that has no place in the colcon build, so
it lives in a Python 3.12 conda env, separate from the ROS 2 workspace. The plugin registers
the arm as the `volcaniarm` robot, a `ros_image` camera (reads a ROS image topic, so it works
in Gazebo and on the real RealSense), and a `volcaniarm_gamepad` teleop that reuses
`volcaniarm_kinematics_py` for the same EE-space feel as `joystick_teleop_node`.

## First-time setup

```bash
git submodule update --init src/volcaniarm_lerobot   # if you cloned without --recurse
cd src/volcaniarm_lerobot && conda env create -f environment.yml   # creates volcaniarm-lerobot
```

Already have a working lerobot env? Just `conda activate <it> && pip install -e .` from the
submodule instead.

## Running the loop

Terminal 1 (ROS) - the arm:

```bash
ros2 launch volcaniarm_bringup sim_bringup.launch.py controller:=traj   # or real_bringup
```

Terminal 2 (ROS) - the joystick driver (publishes `/joy`):

```bash
ros2 launch volcaniarm_controllers lerobot_joy.launch.py
```

Terminal 3 (conda + ROS) - LeRobot. Drive through the LeRobot teleop, NOT the ROS
`joystick_teleop_node`, so LeRobot can record the actions:

```bash
conda activate volcaniarm-lerobot
source /opt/ros/jazzy/setup.bash
lerobot-teleoperate --robot.type=volcaniarm --robot.id=volcaniarm_0 \
  --teleop.type=volcaniarm_gamepad --teleop.id=joy --display_data=true
```

Joystick (PS4): everything needs R1 (deadman) held, as a safety interlock. R1 + left stick
drives the EE in Y-Z, R1 + Cross homes the arm, and during `lerobot-record` R1 + D-pad
controls episodes (Right = next, Left = re-record, Down = stop). Then `lerobot-record` to
collect demos, `lerobot-train` to train, and `lerobot-record --policy.path=...` to deploy.
Exact commands are in the submodule README.

## Recording datasets: how and where they save

A recorded dataset is a `LeRobotDataset` folder, written incrementally as you record:

```
<dataset>/
  meta/info.json      # schema: features, shapes, fps, counts, robot_type, video codec
  meta/stats.json     # per-feature normalization stats (mean/std/min/max) used by training
  meta/episodes/...   # per-episode metadata (parquet)
  meta/tasks.parquet  # the task strings
  data/chunk-*/file-*.parquet   # tabular stream: observation.state, action, timestamps, indices
  videos/observation.images.camera/chunk-*/...   # each camera encoded as VIDEO (libsvtav1)
```

Two things worth knowing about the pipeline:

- **Images are stored as video, not raw frames.** Each camera stream is encoded (libsvtav1
  by default) per episode, so a dataset stays small even with thousands of frames. That's why
  `observation.images.camera` shows up as dtype `video` in `info.json`. Video encoding happens
  on dedicated writer threads while you record.
- **The tabular stream (state/action) is parquet**, keyed by `episode_index` / `frame_index`,
  recorded at the loop rate (`fps`, default 30). The `ros_image` camera samples the latest ROS
  frame each tick, so the sim's 15 Hz camera is simply sampled into the 30 Hz timeline.

### Where it saves (`--dataset.root`)

- **Unset (default):** goes to the HuggingFace cache,
  `~/.cache/huggingface/lerobot/<repo_id>/` (or `$HF_LEROBOT_HOME`). Fine for a one-shot
  recording, but `--resume` will NOT write here: that cache is the shared, revision-safe Hub
  snapshot area, and resuming into it could corrupt it.
- **`--dataset.root=<dir>`:** the dataset lives directly at `<dir>`. Use this for anything you
  will build up over multiple sessions or train on. It is **required** for `--resume`.

**Recommended:** give each dataset its own root on a disk with space (encoded video still adds
up over many episodes), kept out of both the colcon workspace and the HF cache, e.g.
`~/lerobot_data/volcaniarm_<task>/`. Pass the **same `--dataset.root` on the first (create) run
and every later (`--resume=true`) run** so episodes accumulate into one dataset:

```bash
ROOT=~/lerobot_data/volcaniarm_demo
# first run (creates):
lerobot-record --robot.type=volcaniarm --robot.id=volcaniarm_0 \
  --teleop.type=volcaniarm_gamepad --teleop.id=joy --display_data=true \
  --dataset.repo_id=tamirlevin/volcaniarm_demo --dataset.root=$ROOT \
  --dataset.single_task="Reach the weed" --dataset.num_episodes=10
# later runs (append more episodes):
lerobot-record ... --dataset.repo_id=tamirlevin/volcaniarm_demo --dataset.root=$ROOT --resume=true ...
```

### Pushing to the Hub

`push_to_hub` defaults to **true**, so the dataset uploads to
`huggingface.co/datasets/<repo_id>` when recording finishes (run `hf auth login` first; use
`robotics-course/...` as the namespace to put it under the org). Add
`--dataset.push_to_hub=false` to keep it local only; you can always push later.

## Notes

- The `lerobot_joy.launch.py` here starts only `joy_node` (the LeRobot teleop is the driver).
- Sim must run with `controller:=traj` so the JointTrajectoryController and `/joint_states`
  exist.
