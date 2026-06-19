#!/usr/bin/env bash
set -e

CUDA_VISIBLE_DEVICES=0 \
python3 train_feat.py \
--backbone resnet \
--lr 0.05 \
--weight-decay 0.0001 \
--workers 8 \
--epochs 200 \
--batch-size 34 \
--gpu-ids 0 \
--dataset crack \
--start_epoch 0 \
--eval-interval 1 \
--base-size 448 \
--crop-size 448
