# DeepLab Crack Segmentation

This folder contains the DeepLabV3+ crack segmentation model used to produce coarse masks before SAM refinement.

Examples below are written from the `CCIE2026_Crack3D_pipeline_release` repository root.

## Training Dataset

Set `CRACK_DATASET_ROOT` to a folder with:

```text
Dataset/
  Images/
  Masks/
  train.txt
  val.txt
```

Each line in `train.txt` and `val.txt` should be an image/mask filename shared by `Images/` and `Masks/`.

## Train

```bash
export CRACK_DATASET_ROOT="$(pwd)/../CCIE2026_Crack3D_demo_data/training_dataset_example"
cd 01_segmentation/DeepLab
bash train_crack.sh
```

For finetuning:

```bash
export CRACK_DATASET_ROOT="$(pwd)/../CCIE2026_Crack3D_demo_data/training_dataset_example"
export RESUME_CHECKPOINT="$(pwd)/../CCIE2026_Crack3D_demo_data/models/checkpoint35.pth.tar"
cd 01_segmentation/DeepLab
bash train_crack_finetuning.sh
```

## Inference

```bash
DEMO_DATA=../CCIE2026_Crack3D_demo_data

python 01_segmentation/DeepLab/eval_test.py \
  --checkpoint "$DEMO_DATA/models/checkpoint35.pth.tar" \
  --input-dir "$DEMO_DATA/segmentation_demo/data/images" \
  --output-dir "$DEMO_DATA/segmentation_demo/data/mask35_rerun" \
  --device cuda:0
```

Run `python eval_test.py --help` for all options.

## Acknowledgement

The model implementation is adapted from PyTorch DeepLabV3+ implementations, including `pytorch-deeplab-xception`.
