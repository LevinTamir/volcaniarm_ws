# Policy models

Drop the ONNX policy produced by Isaac Lab training here as `policy.onnx`.

Produced by `scripts/rsl_rl/play.py` in the `volcaniarm_isaaclab` project
(exports both `policy.pt` and `policy.onnx` next to the `.pt` checkpoint).

The `RLPolicyController` reads this file at controller `on_configure`.
If it is missing, the controller runs in stub mode (holds the arm at
`default_joint_positions`).
