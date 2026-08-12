
### Repository Structure

```
src/
├── holosoma/              # Core training framework (locomotion & whole-body tracking)
├── holosoma_inference/    # Inference and deployment pipeline
└── holosoma_retargeting/  # Motion retargeting from human motion data to robots
```

---

### Robot Config (YAML)

Per-robot configuration values live in `holosoma_retargeting/config_types/robot_configs/*.yaml`
(e.g. `alice5.yaml`). Every `*.yaml` in that directory is loaded once at import time and merged
into `config_types/robot.py` and `config_types/data_type.py`, so tuning a robot — or adding a new
one — needs no Python change: edit the YAML only.

```yaml
robot_type: alice5        # defaults to the file stem

# robot.py: _ROBOT_DEFAULTS entry
robot_defaults:
  robot_dof: 23
  robot_height: 1.65
  object_name: ground

# robot.py: RobotConfig.FOOT_STICKING_LINKS
foot_sticking_links:
  - left_foot_corner_fl
  # ...

# robot.py: RobotConfig.MANUAL_COST — joint index -> cost weight
manual_cost:
  "22": 0.5

# data_type.py: JOINTS_MAPPINGS — keyed by data format, then human joint -> robot link
joints_mapping:
  lafan: &lafan_mapping
    Hips: pelvis
    Spine1: waist_yaw_link
    # ... map all relevant joints
  sfu: *lafan_mapping     # reuse another format's mapping with a YAML anchor

  smplx:
    Pelvis: pelvis
    Spine2: waist_yaw_link
```

Every block is optional. A YAML entry wins over the corresponding literal/branch in
`robot.py`/`data_type.py`, and omitting a block falls back to that Python default (so `g1` and `t1`
keep their existing inline values). CLI overrides such as `--robot-config.manual-cost` still take
precedence over the YAML.

---

### BVH to .npy (Pre-Retargeting)

BVH-based sources (LAFAN, SFU, bones, ...) must first be baked into `(frames, joints, 3)` global
joint positions. `data_utils/extract_global_positions.py` parses the BVH, runs FK, and writes one
`.npy` per sequence — this is the file `robot_retarget.py` loads for those formats.

```bash
# run from data_utils/ (the script imports the vendored lafan1 package)
cd data_utils

# LAFAN
python extract_global_positions.py --input_dir ./lafan1/lafan --output_dir ../demo_data/lafan

# bones (or any other BVH source)
python extract_global_positions.py --input_dir ../demo_data/bones \
    --output_dir ../demo_data/bones/processed
```

`--scale` is the source unit in meters and defaults to `0.01`, which suits centimeter-authored BVH
(LAFAN, SFU, bones). Pass `--scale 1.0` for files already authored in meters.

The parser handles arbitrary joint hierarchies and per-joint channel layouts, so skeletons where
more than the root joint translates (bones has both `Root` and `Hips` on 6 channels) work without
changes. Axis convention and frame-rate handling stay per-format in
[`robot_retarget.py`](holosoma_retargeting/examples/robot_retarget.py) — e.g. LAFAN gets a y-up to
z-up transform, SFU is rotated and downsampled 120 to 30 fps.

---

### Single Sequence Retargeting
```bash
# --data_format lafan/smplx/sfu/bones
python examples/robot_retarget.py --data_format lafan --data_path demo_data/lafan \
    --task-type robot_only --task-name dance2_subject1 --task-config.ground-range -10 10 \
    --save_dir demo_results/alice5/robot_only/lafan \
    --retargeter.debug --retargeter.visualize --retargeter.foot-sticking-tolerance 0.02 --motion-data-config.segment-scaling
```
**Note:** For LAFAN data, you need to relax the foot sticking constraint by setting --retargeter.foot-sticking-tolerance (default is stricter). You can adjust this tolerance number based on your data quality and retargeting results.

---

### Batch Processing Retargeting
```bash
# --data_format lafan/smplx/sfu/bones
python examples/parallel_robot_retarget.py --data-dir demo_data/bones --task-type robot_only \
    --data_format bones --save_dir demo_results_parallel/alice5/bones --task-config.object-name ground \
    --task-config.ground-range -10 10 --retargeter.foot-sticking-tolerance 0.02 --motion-data-config.segment-scaling
```

---

### Check Visualizations of Saved Retargeting Results

```bash
# .pkl viser player
python viser_player.py --robot_urdf models/alice5/alice5_23dof.urdf \
    --mimickit_pkl pkl/lafan/walk1_subject1.pkl

# .npz viser player
python viser_player.py --robot_urdf models/alice5/alice5_23dof.urdf \
    --qpos_npz demo_results/alice5/robot_only/lafan/walk3_subject4.npz
```

---

### Data Conversion
```bash
# .npz to .pkl
python data_conversion/export_mimickit.py --input demo_results/alice5/robot_only/lafan/walk1_subject1.npz \
    --output pkl/lafan/walk1_subject1.pkl --output-filter-hz 12 --resample-fps 30
```
