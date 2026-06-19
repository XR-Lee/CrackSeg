import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run DeepLabV3+ crack segmentation on an image folder."
    )
    parser.add_argument("--checkpoint", required=True, help="Path to a DeepLab checkpoint.")
    parser.add_argument("--input-dir", required=True, help="Folder containing RGB images.")
    parser.add_argument("--output-dir", required=True, help="Folder for binary mask outputs.")
    parser.add_argument("--backbone", default="resnet", choices=["resnet", "xception", "drn", "mobilenet"])
    parser.add_argument("--output-stride", default=16, type=int, choices=[8, 16])
    parser.add_argument("--num-classes", default=2, type=int)
    parser.add_argument(
        "--device",
        default=None,
        help="Torch device, for example cuda:0 or cpu.",
    )
    parser.add_argument(
        "--extensions",
        nargs="+",
        default=[".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"],
        help="Image extensions to process.",
    )
    parser.add_argument("--recursive", action="store_true", help="Search input images recursively.")
    parser.add_argument(
        "--pretrained-backbone",
        action="store_true",
        help="Download/use ImageNet-pretrained backbone before loading the checkpoint. Disabled by default for offline checkpoint inference.",
    )
    return parser.parse_args()


def iter_images(input_dir, extensions, recursive=False):
    extensions = {ext.lower() if ext.startswith(".") else f".{ext.lower()}" for ext in extensions}
    pattern = "**/*" if recursive else "*"
    for path in sorted(Path(input_dir).glob(pattern)):
        if path.is_file() and path.suffix.lower() in extensions:
            yield path


def load_checkpoint(torch, model, checkpoint_path, device):
    # weights_only=False: this is a trusted, locally distributed training checkpoint
    # (contains numpy scalars/optimizer state). PyTorch 2.6+ defaults to weights_only=True,
    # which rejects such checkpoints, so loading would otherwise fail on modern torch.
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    state_dict = checkpoint.get("state_dict", checkpoint)
    clean_state_dict = {}
    for key, value in state_dict.items():
        clean_key = key[7:] if key.startswith("module.") else key
        clean_state_dict[clean_key] = value
    model.load_state_dict(clean_state_dict)


def main():
    args = parse_args()

    import cv2
    import numpy as np
    import torch
    from PIL import Image
    from torchvision import transforms

    from dataloaders import custom_transforms as tr
    from modeling.deeplab import DeepLab

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    checkpoint_path = Path(args.checkpoint)

    if not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device(args.device or ("cuda:0" if torch.cuda.is_available() else "cpu"))
    model = DeepLab(
        num_classes=args.num_classes,
        backbone=args.backbone,
        output_stride=args.output_stride,
        sync_bn=False,
        freeze_bn=False,
        pretrained_backbone=args.pretrained_backbone,
    ).to(device)
    load_checkpoint(torch, model, checkpoint_path, device)
    model.eval()
    print(f"Model loaded from {checkpoint_path}")

    composed_transforms = transforms.Compose([
        tr.Normalize(),
        tr.Ignore_label(),
        tr.ToTensor(),
    ])

    image_paths = list(iter_images(input_dir, args.extensions, args.recursive))
    if not image_paths:
        raise FileNotFoundError(f"No input images found in {input_dir}")

    for image_path in image_paths:
        img = Image.open(image_path).convert("RGB")
        sample = {"image": img, "label": img}
        sample = composed_transforms(sample)
        tensor = sample["image"].unsqueeze(0).to(device)

        with torch.no_grad():
            output = model(tensor)

        pred = torch.argmax(output, dim=1).squeeze(0).detach().cpu().numpy()
        pred = (pred.astype(np.uint8) * 255)

        relative_path = image_path.relative_to(input_dir) if args.recursive else Path(image_path.name)
        output_path = output_dir / relative_path.with_suffix(".png")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), pred)
        print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
