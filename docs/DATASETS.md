# Dataset Packages

This code release expects large files to be distributed outside Git.

**Download:** https://drive.google.com/drive/folders/1gFtSRWuKS7C0BrCifCmJpb7bshKcNFeG

Place the downloaded `CCIE2026_Crack3D_demo_data/` folder next to the repository (the demo scripts resolve it via `../CCIE2026_Crack3D_demo_data`).

Recommended companion folder:

```text
CCIE2026_Crack3D_demo_data/
|-- segmentation_demo/
|-- models/
|-- colorization_smoke/
|-- width_measurement_demo/
`-- full_dataset_note/
```

## segmentation_demo

Purpose: reproduce the segmentation visualization and optionally re-run SAM refinement.

```text
segmentation_demo/
|-- data/
|   |-- images/
|   |-- mask35/
|   `-- refine35/
`-- outputs/
    `-- comparison_grid.png
```

Samples:

- `1710490542.876314`
- `1710833369.347823`
- `1715852650.974332`

Useful result:

```text
segmentation_demo/outputs/comparison_grid.png
```

## models

Large checkpoints used by the reproducibility demos.

```text
models/
|-- checkpoint35.pth.tar
`-- sam_vit_h_4b8939.pth
```

`checkpoint35.pth.tar` is the DeepLabV3+ crack segmentation checkpoint used to regenerate the bundled `mask35`. Its SHA-256 in the current demo package is:

```text
7e19d3a499352abe4ac17306652f681d1d4cbf636ef94b76174ffecf5cb0cf6b
```

`sam_vit_h_4b8939.pth` is the SAM ViT-H checkpoint used to regenerate the bundled `refine35`.

Use `scripts/run_segmentation_full_demo.sh` to regenerate `mask35_rerun/` from the images and then regenerate `refine35_rerun/` from those masks. The `_rerun` suffix is used so users can compare regenerated results against the packaged release results without overwriting them.

## colorization_smoke

Purpose: test the C++ point-cloud colorization chain on three frames.

```text
colorization_smoke/
|-- scans-crop.pcd
|-- vo_interpolated_odom.txt
`-- raw_images/
```

Useful result:

```text
colorization_smoke/output/cloudInWorldWithRGB.pcd
```

The complete 39-frame colorization case requires substantially more memory and is documented separately.

## width_measurement_demo

Purpose: reproduce non-interactive 3D crack-width measurement.

```text
width_measurement_demo/
|-- filtered_pcd/
|-- mask_select/
|-- raw_images/
|-- selected_points.json
`-- expected/
```

Useful results:

```text
width_measurement_demo/crack_width_3d_results.json
width_measurement_demo/edt_skeleton/*_skeleton_edge_pts.png
```

The demo `selected_points.json` uses image coordinates:

```json
{
  "points": [
    {"timestamp": 1721034869.448148, "points_xy": [[1869, 5]]}
  ]
}
```

## Full Raw Dataset

The raw bag `_2024-07-15-17-12-54.bag` is about 34 GB and is not part of the default demo data package. Publish it through a dataset host when full end-to-end reconstruction must be reproduced.

After packaging data, generate checksums:

```bash
sha256sum $(find CCIE2026_Crack3D_demo_data -type f | sort) > CCIE2026_Crack3D_demo_data/checksums.sha256
```
