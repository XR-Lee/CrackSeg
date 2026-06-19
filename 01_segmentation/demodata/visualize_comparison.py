import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize DeepLab coarse masks and SAM-refined crack masks."
    )
    parser.add_argument(
        "--data-root",
        default=Path(__file__).resolve().parent / "data",
        type=Path,
        help="Demo data root containing images/, mask35/, and refine35/.",
    )
    parser.add_argument("--image-folder", default="images", help="Image folder name under data-root.")
    parser.add_argument("--mask-folder", default="mask35", help="DeepLab mask folder name under data-root.")
    parser.add_argument("--refine-folder", default="refine35", help="SAM refined mask folder name under data-root.")
    parser.add_argument(
        "--output",
        default=Path(__file__).resolve().parent / "outputs" / "comparison_grid.png",
        type=Path,
        help="Path to save the comparison figure.",
    )
    parser.add_argument("--max-samples", default=3, type=int, help="Maximum number of samples to show.")
    return parser.parse_args()


def find_image(image_dir, stem):
    for suffix in IMAGE_EXTENSIONS:
        candidate = image_dir / f"{stem}{suffix}"
        if candidate.is_file():
            return candidate
    return None


def common_stems(data_root, image_folder, mask_folder, refine_folder):
    image_dir = data_root / image_folder
    mask_dir = data_root / mask_folder
    refine_dir = data_root / refine_folder
    image_stems = {path.stem for path in image_dir.iterdir() if path.is_file()}
    mask_stems = {path.stem for path in mask_dir.glob("*.png")}
    refine_stems = {path.stem for path in refine_dir.glob("*.png")}
    return sorted(image_stems & mask_stems & refine_stems)


def load_mask(path, size):
    mask = Image.open(path).convert("L")
    if mask.size != size:
        mask = mask.resize(size, Image.Resampling.NEAREST)
    return np.array(mask) > 0


def overlay_mask(image, mask, color, alpha=0.55):
    output = np.array(image).astype(np.float32)
    color = np.array(color, dtype=np.float32)
    output[mask] = output[mask] * (1.0 - alpha) + color * alpha
    return output.astype(np.uint8)


def mask_stats(base_mask, refined_mask):
    base_pixels = int(base_mask.sum())
    refined_pixels = int(refined_mask.sum())
    added_pixels = int(np.logical_and(refined_mask, ~base_mask).sum())
    removed_pixels = int(np.logical_and(base_mask, ~refined_mask).sum())
    return base_pixels, refined_pixels, added_pixels, removed_pixels


def main():
    args = parse_args()
    data_root = args.data_root
    image_dir = data_root / args.image_folder
    mask_dir = data_root / args.mask_folder
    refine_dir = data_root / args.refine_folder

    for folder in (image_dir, mask_dir, refine_dir):
        if not folder.is_dir():
            raise FileNotFoundError(f"Required demo folder not found: {folder}")

    stems = common_stems(data_root, args.image_folder, args.mask_folder, args.refine_folder)[: args.max_samples]
    if not stems:
        raise FileNotFoundError(f"No matching image/mask/refine samples found under {data_root}")

    fig, axes = plt.subplots(len(stems), 4, figsize=(16, 4 * len(stems)), squeeze=False)
    summaries = []

    for row, stem in enumerate(stems):
        image_path = find_image(image_dir, stem)
        if image_path is None:
            continue

        image = Image.open(image_path).convert("RGB")
        base_mask = load_mask(mask_dir / f"{stem}.png", image.size)
        refined_mask = load_mask(refine_dir / f"{stem}.png", image.size)
        added_mask = np.logical_and(refined_mask, ~base_mask)

        base_pixels, refined_pixels, added_pixels, removed_pixels = mask_stats(base_mask, refined_mask)
        summaries.append((stem, base_pixels, refined_pixels, added_pixels, removed_pixels))

        panels = [
            ("Image", np.array(image)),
            ("DeepLab coarse", overlay_mask(image, base_mask, color=(255, 0, 0))),
            ("SAM refine", overlay_mask(image, refined_mask, color=(0, 220, 80))),
            ("Added by refine", overlay_mask(image, added_mask, color=(0, 120, 255))),
        ]

        for col, (title, panel) in enumerate(panels):
            ax = axes[row][col]
            ax.imshow(panel)
            ax.set_title(title)
            ax.axis("off")

        axes[row][0].set_ylabel(stem, rotation=0, labelpad=55, va="center")

    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(f"Saved comparison figure to {args.output}")
    print("sample,deeplab_pixels,refined_pixels,added_pixels,removed_pixels")
    for summary in summaries:
        print(",".join(str(value) for value in summary))


if __name__ == "__main__":
    main()
