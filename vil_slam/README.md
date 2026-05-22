# VIL-SLAM Fusion (placeholder)

This directory will host the Visual–Inertial–LiDAR fusion code from [Deng et al., 2025](https://arxiv.org/abs/2501.09203). The stage:

1. Runs **FastLIO2** on the LiDAR + IMU stream to obtain a drift-corrected dense point cloud together with per-frame camera poses.
2. Projects per-frame 2D crack masks produced by [`../sam_refine/`](../sam_refine/) into the fused point cloud using the calibrated extrinsics.
3. Aggregates multi-frame projections into a semantically-enriched 3D crack point cloud at real-world scale, which feeds the [`../measurement/`](../measurement/) stage.

Planned contents:
- Calibration utilities for the LiDAR / IMU / camera rig used in the paper.
- A FastLIO2 launch / config bundle and the multi-frame mask projection scripts.
- Sample input sequences (LiDAR `.bag` + RGB frames) and reference fused outputs from the in-house field dataset.

Code, calibration files, and the paired sequences will appear here once cleared for distribution. Track [the repo Releases](https://github.com/XR-Lee/CrackSeg/releases) for updates.
