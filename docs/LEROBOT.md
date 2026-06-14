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

## Notes

- The `lerobot_joy.launch.py` here starts only `joy_node` (the LeRobot teleop is the driver).
- Sim must run with `controller:=traj` so the JointTrajectoryController and `/joint_states`
  exist.
