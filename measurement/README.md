# Automated 3D Crack Measurement (placeholder)

This directory will host the automated measurement pipeline from [Deng et al., 2025](https://arxiv.org/abs/2501.09203). Given the semantically-enriched 3D crack point cloud produced by [`../vil_slam/`](../vil_slam/), the pipeline extracts:

- Per-segment crack **width** — mean absolute error < 0.1 mm versus a Dino-Lite AF4915 microscope ground truth in the paper's evaluation.
- Crack **spatial location** in real-world coordinates.
- Total crack **length** and per-crack width statistics for inspection reports.

Planned contents:
- Width / length estimators that operate directly on the 3D crack point cloud (no manual post-processing).
- A reproducible evaluation harness against the microscope-measured ground truth used in the paper.
- Worked examples on the in-house field sequences shared via [GitHub Releases](https://github.com/XR-Lee/CrackSeg/releases).

Code will land here alongside the [`../vil_slam/`](../vil_slam/) stage.
