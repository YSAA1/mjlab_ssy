# G1 flying kick motion

This directory stages the G4 spinning back kick source motion on `main` and
contains the derived Unitree G1 tracking asset.

## Files

- `raw/g4_spinning_back_kick_poses.npz`: promoted source motion from
  `feature/z2-lite-motion-retarget`.
- `keypoints_50hz/`: 50 Hz ProtoMotions SMPL keypoints derived from the raw
  source.
- `retarget/g1_g4_spinning_back_kick_g1_retargeted.npz`: PyRoki retarget output
  for Unitree G1, with 29 joint columns.
- `mjlab/g1_g4_spinning_back_kick.csv`: G1 CSV replay input.
- `mjlab/motion.npz`: standard mjlab tracking motion artifact for training.

## Rebuild notes

The source NPZ uses the `pose_body` / `root_orient` / `trans` layout rather
than the older AMASS `poses` / `mocap_framerate` layout expected by
`build_tracking_motion`. The committed keypoints are therefore part of the
reproducible input set for this motion.

G1 retarget was run with:

```bash
uv run --active python src/mjlab/scripts/smpl_keypoints_to_g1_npz.py \
  --input-file data/motions/g1_flying_kick/keypoints_50hz/g4_spinning_back_kick_poses_keypoints.npy \
  --output-file data/motions/g1_flying_kick/retarget/g1_g4_spinning_back_kick_g1_retargeted.npz \
  --protomotions-root /home/ssy/ssy_files/mjlab/.external/ProtoMotions \
  --pyroki-python /home/ssy/anaconda3/envs/pyroki/bin/python \
  --source-type smpl \
  --subsample-factor 1 \
  --target-raw-frames 500 \
  --input-fps 50.0 \
  --force-remake True
```

The final `motion.npz` was generated from `mjlab/g1_g4_spinning_back_kick.csv`
at 50 Hz with `skip_wandb=True`.
