# Training & Evaluation (placeholder)

This directory will host the DeepLabV3+ training and few-shot evaluation pipeline used in [Deng et al., 2025](https://arxiv.org/abs/2501.09203). The SAM refinement under [`../sam_refine/`](../sam_refine/) is applied on top of the DeepLabV3+ output at inference time and does not require its own training.

Two-stage training:

1. **Pre-training** on the ten aggregated public crack datasets — 50 iterations, batch size 32, initial learning rate 0.01.
2. **Fine-tuning** on the unseen-scenario in-house field data — 200 iterations, batch size 4, learning rate 0.001.

Data augmentations: horizontal / vertical flips, rotations, Gaussian blur. Evaluation is reported under **0-shot, 10-shot, and 110-shot** configurations on the field test set, using IoU.

Planned contents:
- DeepLabV3+ training / fine-tuning configs and launch scripts.
- The pre-aggregation script that builds the combined ten-dataset training set.
- Few-shot evaluation harness covering the 0/10/110-shot splits.
- Released DeepLabV3+ checkpoints via [GitHub Releases](https://github.com/XR-Lee/CrackSeg/releases).
