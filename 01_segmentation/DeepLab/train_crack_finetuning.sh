#!/usr/bin/env bash
set -e

if [ -z "${RESUME_CHECKPOINT:-}" ]; then
  echo "Set RESUME_CHECKPOINT=../../../CCIE2026_Crack3D_demo_data/models/checkpoint35.pth.tar before running finetuning."
  exit 1
fi

CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0} \
python3 train_feat.py \
--backbone resnet \
--lr 0.005 \
--workers 4 \
--epochs 100 \
--batch-size 3 \
--gpu-ids 0 \
--dataset crack \
--start_epoch 0 \
--eval-interval 3 \
--base-size 1024 \
--crop-size 1024 \
--resume "${RESUME_CHECKPOINT}"
