import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tqdm
from scipy.ndimage import distance_transform_edt


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


class CropBundle:
    def __init__(self, image, mask, edt, crop_box):
        self.image = image
        self.mask = mask
        self.edt = edt
        self.crop_box = crop_box
        self.suc_state = False

    def write_results(self, output):
        self.mask = output
        self.suc_state = True

    def get_img(self):
        return self.image

    def get_mask(self):
        return self.mask

    def get_edt(self):
        return self.edt

    def is_suc(self):
        return self.suc_state

    def get_crop_box(self):
        return self.crop_box


class ImageLoader:
    def __init__(self, image_path, mask_path):
        self.image_path = image_path
        self.mask_path = mask_path
        self.load_raw()

    def load_raw(self):
        image = cv2.imread(str(self.image_path))
        if image is None:
            raise FileNotFoundError(f"Image not readable: {self.image_path}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        mask = cv2.imread(str(self.mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise FileNotFoundError(f"Mask not readable: {self.mask_path}")

        self.raw_mask = mask.copy()
        self.raw_edt = mask_edt(mask)
        self.plot_image = image.copy()
        self.image = image
        self.mask = mask
        self.edt = self.raw_edt.copy()
        self.clusters = []

    def dialate_mask(self):
        kernel = np.ones((3, 3), np.uint8)
        self.dialated_mask = cv2.dilate(self.mask, kernel, iterations=10)

    def clustering(self):
        binary_mask = cv2.inRange(self.dialated_mask, 1, 255)
        num_labels, _labels, stats, _centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
        clusters = []
        for label in range(1, num_labels):
            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            w = stats[label, cv2.CC_STAT_WIDTH]
            h = stats[label, cv2.CC_STAT_HEIGHT]
            if w > 100 or h > 100:
                clusters.append([x, y, w, h])
        self.clusters = clusters
        return clusters

    def get_crop_bundle(self):
        if self.clusters is None or len(self.clusters) == 0:
            return []

        crop_bundles = []
        for x, y, w, h in self.clusters:
            if x - 200 < 0:
                x = 200
            if y - 200 < 0:
                y = 200
            crop_frame = [y - 200, y + h + 200, x - 200, x + w + 200]
            crop_bundles.append(
                CropBundle(
                    self.image[crop_frame[0] : crop_frame[1], crop_frame[2] : crop_frame[3]],
                    self.mask[crop_frame[0] : crop_frame[1], crop_frame[2] : crop_frame[3]],
                    self.edt[crop_frame[0] : crop_frame[1], crop_frame[2] : crop_frame[3]],
                    crop_frame,
                )
            )
        return crop_bundles

    def update_with_bundle(self, crop_bundle):
        crop_frame = crop_bundle.get_crop_box()
        new_mask = crop_bundle.get_mask()
        if crop_bundle.is_suc():
            self.mask[crop_frame[0] : crop_frame[1], crop_frame[2] : crop_frame[3]] = cv2.bitwise_or(
                self.mask[crop_frame[0] : crop_frame[1], crop_frame[2] : crop_frame[3]],
                new_mask,
            )
            cv2.rectangle(self.raw_edt, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), 5, 6)
            cv2.rectangle(self.plot_image, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), (0, 255, 0), 6)
        else:
            cv2.rectangle(self.raw_edt, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), 1, 3)
            cv2.rectangle(self.plot_image, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), (100, 100, 100), 3)

    def get_img(self):
        return self.image

    def get_plotimg(self):
        return self.plot_image

    def get_mask(self):
        return self.mask

    def get_rawmask(self):
        return self.raw_mask

    def get_rawedt(self):
        return self.raw_edt

    def get_edt(self):
        return self.edt


def parse_args():
    parser = argparse.ArgumentParser(description="Run self-contained SAM refinement on the demo samples.")
    default_checkpoint = Path(__file__).resolve().parent / "checkpoints" / "sam_vit_h_4b8939.pth"
    parser.add_argument("--data-root", default=Path(__file__).resolve().parent / "data", type=Path)
    parser.add_argument("--image-folder", default="images")
    parser.add_argument("--mask-folder", default="mask35")
    parser.add_argument("--output-mask-folder", default="refine35_rerun")
    parser.add_argument("--output-vis-folder", default="refinevis35_rerun")
    parser.add_argument(
        "--sam-checkpoint",
        default=default_checkpoint,
        type=Path,
        help="SAM checkpoint path. Defaults to demodata/checkpoints/sam_vit_h_4b8939.pth.",
    )
    parser.add_argument("--sam-model-type", default="vit_h", choices=["vit_b", "vit_l", "vit_h"])
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--edt-threshold", type=float, default=2.0)
    parser.add_argument("--min-points", type=int, default=15)
    parser.add_argument("--width-reject-ratio", type=float, default=2.0)
    return parser.parse_args()


def load_sam_predictor(checkpoint, model_type, device):
    try:
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError as exc:
        raise ImportError(
            "segment-anything is required. Install it with "
            "`pip install segment-anything`."
        ) from exc

    if not checkpoint.is_file():
        raise FileNotFoundError(f"SAM checkpoint not found: {checkpoint}")

    sam = sam_model_registry[model_type](checkpoint=str(checkpoint))
    sam.to(device=device)
    return SamPredictor(sam)


def mask_edt(mask):
    _, binary_image = cv2.threshold(mask.astype(np.uint8), 0, 255, cv2.THRESH_BINARY)
    return distance_transform_edt(binary_image)


def sort_and_downsample(coords, target_size):
    sorted_coords = sorted(coords, key=lambda x: (x[0], x[1]))
    if len(sorted_coords) > target_size:
        step = len(sorted_coords) // target_size
    else:
        step = 1
    return sorted_coords[::step][:target_size]


def get_edt_max(edt):
    return np.max(edt)


def edt_to_points(edt, threas=7):
    _, binary_image = cv2.threshold(edt, threas, 255, cv2.THRESH_BINARY)
    binary_image = binary_image.astype(np.uint8)
    contours, _hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    key_points = []
    for contour in contours:
        for point in contour:
            key_points.append(tuple(point[0]))

    key_points = sort_and_downsample(key_points, 20)
    return np.array(key_points), np.ones((len(key_points)))


def show_mask(mask, ax):
    color = np.array([1.0, 0.1, 0.1, 0.8])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) / 255 * color.reshape(1, 1, -1)
    ax.imshow(mask_image)


def iter_image_stems(image_dir):
    for path in sorted(image_dir.iterdir()):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
            yield path.stem, path


def refine_file(args, predictor, image_path, mask_path, output_mask_dir, output_vis_dir):
    loader = ImageLoader(image_path, mask_path)
    loader.dialate_mask()
    loader.clustering()
    crop_bundles = loader.get_crop_bundle()

    if len(crop_bundles) == 0:
        return False

    success_count = 0
    for crop_bundle in crop_bundles:
        pts, label = edt_to_points(crop_bundle.get_edt(), threas=args.edt_threshold)
        if len(pts) <= args.min_points:
            loader.update_with_bundle(crop_bundle)
            continue

        predictor.set_image(crop_bundle.get_img())
        masks, scores, _logits = predictor.predict(
            point_coords=pts,
            point_labels=label,
            multimask_output=False,
        )

        for mask, _score in zip(masks, scores):
            h, w = mask.shape[-2:]
            mask_float = mask.reshape(h, w).astype(float)
            reject = get_edt_max(mask_edt(mask_float)) > args.width_reject_ratio * get_edt_max(crop_bundle.get_edt())
            if not reject:
                success_count += 1
                crop_bundle.write_results(mask.reshape(h, w).astype(np.uint8) * 255)
            loader.update_with_bundle(crop_bundle)

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes[0][0].imshow(loader.get_rawmask(), cmap="viridis")
    axes[0][1].imshow(loader.get_rawedt(), cmap="viridis")
    axes[1][0].imshow(loader.get_mask(), cmap="viridis")
    axes[1][1].imshow(loader.get_plotimg())
    show_mask(loader.get_mask(), axes[1][1])
    for row in axes:
        for ax in row:
            ax.axis("off")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=0, hspace=0)

    output_mask_dir.mkdir(parents=True, exist_ok=True)
    output_vis_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_vis_dir / f"{image_path.stem}.png")
    cv2.imwrite(str(output_mask_dir / f"{image_path.stem}.png"), loader.get_mask())
    plt.close(fig)
    return success_count > 0


def main():
    args = parse_args()
    image_dir = args.data_root / args.image_folder
    mask_dir = args.data_root / args.mask_folder
    output_mask_dir = args.data_root / args.output_mask_folder
    output_vis_dir = args.data_root / args.output_vis_folder

    predictor = load_sam_predictor(args.sam_checkpoint, args.sam_model_type, args.device)
    files = list(iter_image_stems(image_dir))
    if not files:
        raise FileNotFoundError(f"No images found in {image_dir}")

    processed = 0
    skipped = 0
    for stem, image_path in tqdm.tqdm(files):
        mask_path = mask_dir / f"{stem}.png"
        if not mask_path.is_file():
            skipped += 1
            print(f"Skipping {stem}: missing mask {mask_path}")
            continue
        refine_file(args, predictor, image_path, mask_path, output_mask_dir, output_vis_dir)
        processed += 1

    print(f"Processed {processed} image(s). Skipped {skipped} image(s).")
    print(f"Refined masks: {output_mask_dir}")
    print(f"Visualizations: {output_vis_dir}")


if __name__ == "__main__":
    main()
