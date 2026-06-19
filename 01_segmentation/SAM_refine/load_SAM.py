from pathlib import Path


def load_SAM(sam_checkpoint, model_type="vit_h", device="cuda:0"):
    try:
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError as exc:
        raise ImportError(
            "segment-anything is required for SAM refinement. Install it with "
            "`pip install git+https://github.com/facebookresearch/segment-anything.git`."
        ) from exc

    sam_checkpoint = Path(sam_checkpoint)
    if not sam_checkpoint.is_file():
        raise FileNotFoundError(f"SAM checkpoint not found: {sam_checkpoint}")

    sam = sam_model_registry[model_type](checkpoint=str(sam_checkpoint))
    sam.to(device=device)
    predictor = SamPredictor(sam)

    return predictor
