# Stage 5 — Automated 3D Crack-Width Measurement

Final stage of:

> 3D modeling and automated measurement of concrete cracks via segment anything refinement and visual inertial LiDAR fusion

Given the per-frame colorized point clouds and refined crack masks from the upstream stages, this stage computes real-world crack width in 3D. For each selected skeleton point it fits a local plane, finds the two crack edges, back-projects them to 3D, and reports the Euclidean width.

## Contents

```text
05_crack_width_measurement/
  genNormAndDistanceMask.py     Width-measurement pipeline (standalone Python)
  requirements-width.txt        Python dependencies
```

## Installation

```bash
pip install -r 05_crack_width_measurement/requirements-width.txt
```

Validated dependency versions (Python 3.8) are listed in [`docs/REPRODUCIBILITY.md`](../docs/REPRODUCIBILITY.md). `open3d` wheels require Python ≤ 3.12.

## Inputs

The demo assumes the companion data folder next to the repository (`../CCIE2026_Crack3D_demo_data`). Per frame `{timestamp}` it expects:

- `filtered_pcd/{timestamp}.pcd` — visible per-frame colorized point cloud (Stage 4 output).
- `mask_select/{timestamp}.png` — refined crack mask (Stage 1 output).
- `raw_images/{timestamp}.jpg` — source RGB image.
- `selected_points.json` — skeleton points for non-interactive runs, or use `--manual-select`.

## Run

Non-interactive demo (recommended, uses the bundled `selected_points.json`):

```bash
bash scripts/run_width_measurement_demo.sh ../CCIE2026_Crack3D_demo_data
```

Equivalent direct invocation:

```bash
python 05_crack_width_measurement/genNormAndDistanceMask.py \
  --data-root ../CCIE2026_Crack3D_demo_data/width_measurement_demo \
  --camera-config configs/camera_4096x3000_paper.json \
  --stage all \
  --selected-points-json ../CCIE2026_Crack3D_demo_data/width_measurement_demo/selected_points.json
```

Manual point selection (requires an OpenCV GUI session) replaces the JSON with `--manual-select`.

## Outputs

Written under the data root:

- `norm_masks/*_norm.png`
- `distance_mask/*_distance.png`, `distance_mask/*_3d_pts_on_img.png`
- `edt_skeleton/*_edt.png`, `*_skeleton.png`, `*_skeleton_edge_pts.png`
- `crack_width_3d_results.json` — **main result** (per-point 2D/3D edges, local plane, `width_mm`).

The demo ships an `expected/` folder; regenerated `width_mm` should match it to within floating-point tolerance (sub-micron on the bundled two frames). See [`docs/PIPELINE_IO.md`](../docs/PIPELINE_IO.md) for the full I/O contract.

## License

Apache License, Version 2.0 — see the repository-root `LICENSE`.
