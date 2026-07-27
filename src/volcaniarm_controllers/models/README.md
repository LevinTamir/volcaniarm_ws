# Policy models

Both ONNX policies are committed (see `MODEL_VERSION.txt` for the exact
training-run provenance and md5 of each):

- `policy.onnx` — state-based policy, read by `RLPolicyController`.
  Produced by `scripts/rsl_rl/play.py` in the `volcaniarm_isaaclab`
  project (exports both `policy.pt` and `policy.onnx` next to the `.pt`
  checkpoint).
- `policy_vision.onnx` — vision bundle (image + joint_pos_rel +
  last_action → action), read by `RLVisionPolicyController`. Produced by
  `scripts/rsl_rl/export_onnx_bundle.py` in `volcaniarm_isaaclab`.

Each controller reads its file at `on_configure`. If the file is
missing, the controller runs in stub mode (holds the arm at
`default_joint_positions`).

When deploying a new model, replace the file here and update
`MODEL_VERSION.txt` (source run path + md5) in the same commit.
