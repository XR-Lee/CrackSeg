import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tqdm

from load_images import ImageLoader
from load_SAM import load_SAM
from mask_process import EDT_to_pts, get_EDT_max, mask_EDT, show_mask


def parse_args():
    parser = argparse.ArgumentParser(
        description="Refine coarse crack masks with Segment Anything prompts."
    )
    parser.add_argument("--root-path", required=True, help="Dataset root containing image and mask folders.")
    parser.add_argument("--image-sub-folder", default="crop", help="Image folder under root-path.")
    parser.add_argument("--mask-sub-folder", default="maskcrop", help="Coarse mask folder under root-path.")
    parser.add_argument(
        "--refine-sub-mask-folder",
        default="croprefine",
        help="Output folder for refined binary masks under root-path.",
    )
    parser.add_argument(
        "--refine-sub-folder",
        default="croprefinevis",
        help="Output folder for refinement visualizations under root-path.",
    )
    parser.add_argument("--sam-checkpoint", required=True, help="Path to a SAM checkpoint file.")
    parser.add_argument("--sam-model-type", default="vit_h", choices=["vit_b", "vit_l", "vit_h"])
    parser.add_argument("--device", default="cuda:0", help="Torch device for SAM, for example cuda:0 or cpu.")
    parser.add_argument("--image-exts", nargs="+", default=[".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"])
    parser.add_argument("--mask-exts", nargs="+", default=[".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"])
    parser.add_argument("--edt-threshold", type=float, default=2.0, help="EDT threshold for point prompts.")
    parser.add_argument("--min-points", type=int, default=15, help="Reject crops with too few prompt points.")
    parser.add_argument(
        "--width-reject-ratio",
        type=float,
        default=2.0,
        help="Reject SAM masks whose EDT max exceeds this ratio against the coarse crop.",
    )
    return parser.parse_args()


def normalize_subfolder(path):
    return path.strip("/\\")


def iter_image_stems(image_dir, extensions):
    extensions = {ext.lower() if ext.startswith(".") else f".{ext.lower()}" for ext in extensions}
    for path in sorted(image_dir.iterdir()):
        if path.is_file() and path.suffix.lower() in extensions:
            yield path.stem, path


def find_by_stem(folder, stem, extensions):
    for ext in extensions:
        ext = ext if ext.startswith(".") else f".{ext}"
        candidate = folder / f"{stem}{ext}"
        if candidate.is_file():
            return candidate
    return None


def refine_file(args, predictor, image_path, mask_path, vis_dir, mask_dir):
    image_loader = ImageLoader(str(image_path), str(mask_path))
    image_loader.dialate_mask()
    image_loader.clustering()
    crop_bundles = image_loader.get_crop_bundle()

    if len(crop_bundles) == 0:
        return False

    success_count = 0
    for crop_bundle in crop_bundles:
        pts, label = EDT_to_pts(crop_bundle.get_edt(), threas=args.edt_threshold)
        if len(pts) <= args.min_points:
            image_loader.update_with_bundle(crop_bundle)
            continue

        predictor.set_image(crop_bundle.get_img())
        masks, scores, _logits = predictor.predict(
            point_coords=pts,
            point_labels=label,
            multimask_output=False,
        )

        for mask, _score in zip(masks, scores):
            h, w = mask.shape[-2:]
            mask_image = mask.reshape(h, w).astype(float)
            edt = mask_EDT(mask_image)
            reject_flag = get_EDT_max(edt) > args.width_reject_ratio * get_EDT_max(crop_bundle.get_edt())

            mask_image = mask.reshape(h, w).astype(np.uint8) * 255
            if not reject_flag:
                success_count += 1
                crop_bundle.write_results(mask_image)
            image_loader.update_with_bundle(crop_bundle)

    fig, axe = plt.subplots(2, 2, figsize=(16, 12))
    axe[0][0].imshow(image_loader.get_rawmask(), cmap="viridis")
    axe[0][1].imshow(image_loader.get_rawedt(), cmap="viridis")
    axe[1][0].imshow(image_loader.get_mask(), cmap="viridis")
    axe[1][1].imshow(image_loader.get_plotimg())
    show_mask(image_loader.get_mask(), axe[1][1])

    for axs in axe:
        for ax in axs:
            ax.axis("off")

    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=0, hspace=0)
    plt.savefig(vis_dir / f"{image_path.stem}.png")
    cv2.imwrite(str(mask_dir / f"{image_path.stem}.png"), image_loader.get_mask())
    plt.close(fig)
    return success_count > 0


def main():
    args = parse_args()
    root_path = Path(args.root_path)
    image_dir = root_path / normalize_subfolder(args.image_sub_folder)
    coarse_mask_dir = root_path / normalize_subfolder(args.mask_sub_folder)
    refine_mask_dir = root_path / normalize_subfolder(args.refine_sub_mask_folder)
    refine_vis_dir = root_path / normalize_subfolder(args.refine_sub_folder)

    if not image_dir.is_dir():
        raise FileNotFoundError(f"Image folder not found: {image_dir}")
    if not coarse_mask_dir.is_dir():
        raise FileNotFoundError(f"Mask folder not found: {coarse_mask_dir}")

    refine_mask_dir.mkdir(parents=True, exist_ok=True)
    refine_vis_dir.mkdir(parents=True, exist_ok=True)

    predictor = load_SAM(args.sam_checkpoint, model_type=args.sam_model_type, device=args.device)
    files = list(iter_image_stems(image_dir, args.image_exts))
    if not files:
        raise FileNotFoundError(f"No images found in {image_dir}")

    processed = 0
    skipped = 0
    for stem, image_path in tqdm.tqdm(files):
        mask_path = find_by_stem(coarse_mask_dir, stem, args.mask_exts)
        if mask_path is None:
            skipped += 1
            print(f"Skipping {stem}: matching coarse mask not found.")
            continue
        refine_file(args, predictor, image_path, mask_path, refine_vis_dir, refine_mask_dir)
        processed += 1

    print(f"Processed {processed} image(s). Skipped {skipped} image(s).")
    print(f"Refined masks: {refine_mask_dir}")
    print(f"Visualizations: {refine_vis_dir}")


if __name__ == "__main__":
    main()
