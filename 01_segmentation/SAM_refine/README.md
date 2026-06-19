# SAM Mask Refinement

This folder refines coarse crack masks with Segment Anything Model (SAM). The main public entry point is `refine_masks.py`.

## Inputs

Prepare a case folder with RGB images and coarse masks:

```text
data/refine_case/
  crop/
    image_0001.png
  maskcrop/
    image_0001.png
```

Image and mask files are matched by filename stem.

## Run

```bash
python SAM_refine/refine_masks.py \
  --root-path data/refine_case \
  --image-sub-folder crop \
  --mask-sub-folder maskcrop \
  --refine-sub-mask-folder croprefine \
  --refine-sub-folder croprefinevis \
  --sam-checkpoint checkpoints/sam_vit_h_4b8939.pth \
  --sam-model-type vit_h \
  --device cuda:0
```

Outputs:

- `croprefine/`: refined binary masks.
- `croprefinevis/`: visual QA images.

Run `python SAM_refine/refine_masks.py --help` for all options.
