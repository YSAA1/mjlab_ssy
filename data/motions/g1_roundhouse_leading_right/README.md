# G1 roundhouse leading right motion

This directory stages the `roundhouse_leading_right` source motion and contains
the derived Unitree G1 tracking asset.

## Files

- `raw/roundhouse_leading_right_poses.npz`: source SMPL-style motion copied from
  `data/motions/z2_lite_mocap/roundhouse_leading_right/source/`.
- `keypoints_50hz/`: 50 Hz ProtoMotions SMPL keypoints copied from the same
  source motion family.
- `retarget/g1_roundhouse_leading_right_g1_retargeted.npz`: PyRoki retarget
  output for Unitree G1, with 29 joint columns.
- `mjlab/g1_roundhouse_leading_right.csv`: G1 CSV replay input.
- `mjlab/motion.npz`: standard mjlab tracking motion artifact for training.

## Rebuild notes

G1 retarget was run with:

```bash
uv run --active --no-sync python src/mjlab/scripts/smpl_keypoints_to_g1_npz.py \
  --input-file data/motions/g1_roundhouse_leading_right/keypoints_50hz/roundhouse_leading_right_poses_keypoints.npy \
  --output-file data/motions/g1_roundhouse_leading_right/retarget/g1_roundhouse_leading_right_g1_retargeted.npz \
  --protomotions-root /home/ssy/ssy_files/mjlab/.external/ProtoMotions \
  --pyroki-python /home/ssy/anaconda3/envs/pyroki/bin/python \
  --source-type smpl \
  --subsample-factor 1 \
  --target-raw-frames 500 \
  --input-fps 50.0 \
  --force-remake True
```

The final `motion.npz` was generated from
`mjlab/g1_roundhouse_leading_right.csv` at 50 Hz with `skip_wandb=True`.
