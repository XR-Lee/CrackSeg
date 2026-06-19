import argparse
import time
import urllib.error
import urllib.request
from pathlib import Path


SAM_CHECKPOINTS = {
    "vit_h": {
        "filename": "sam_vit_h_4b8939.pth",
        "url": "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth",
    },
    "vit_l": {
        "filename": "sam_vit_l_0b3195.pth",
        "url": "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth",
    },
    "vit_b": {
        "filename": "sam_vit_b_01ec64.pth",
        "url": "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth",
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description="Download a SAM checkpoint for the demo.")
    parser.add_argument("--model-type", default="vit_h", choices=sorted(SAM_CHECKPOINTS))
    parser.add_argument(
        "--output-dir",
        default=Path(__file__).resolve().parent / "checkpoints",
        type=Path,
        help="Folder where the checkpoint will be saved.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Download even if the target file exists.")
    parser.add_argument("--retries", type=int, default=10, help="Number of retry attempts for interrupted downloads.")
    return parser.parse_args()


def get_remote_size(url):
    try:
        request = urllib.request.Request(url, method="HEAD")
        with urllib.request.urlopen(request, timeout=60) as response:
            length = response.headers.get("Content-Length")
            return int(length) if length else None
    except OSError:
        return None


def parse_total_size(response, existing_size):
    content_range = response.headers.get("Content-Range")
    if content_range and "/" in content_range:
        total = content_range.rsplit("/", 1)[-1]
        if total.isdigit():
            return int(total)

    length = response.headers.get("Content-Length")
    if length and length.isdigit():
        return existing_size + int(length)

    return None


def print_progress(downloaded, total_size):
    if not total_size:
        print(f"\rDownloaded {downloaded / 1024 / 1024:.1f} MB", end="")
        return
    percent = downloaded * 100 / total_size
    print(f"\rDownloaded {downloaded / 1024 / 1024:.1f} / {total_size / 1024 / 1024:.1f} MB ({percent:.1f}%)", end="")


def download_with_resume(url, output_path, retries):
    remote_size = get_remote_size(url)
    chunk_size = 1024 * 1024

    for attempt in range(1, retries + 1):
        existing_size = output_path.stat().st_size if output_path.exists() else 0

        if remote_size and existing_size == remote_size:
            print(f"Checkpoint already complete: {output_path}")
            return

        if remote_size and existing_size > remote_size:
            output_path.unlink()
            existing_size = 0

        headers = {}
        if existing_size:
            headers["Range"] = f"bytes={existing_size}-"
            print(f"Resuming from {existing_size / 1024 / 1024:.1f} MB")

        request = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                status = getattr(response, "status", None)
                if existing_size and status != 206:
                    print("\nServer did not accept resume request; restarting download.")
                    existing_size = 0

                mode = "ab" if existing_size else "wb"
                total_size = parse_total_size(response, existing_size) or remote_size
                downloaded = existing_size

                with output_path.open(mode + "") as file_obj:
                    while True:
                        chunk = response.read(chunk_size)
                        if not chunk:
                            break
                        file_obj.write(chunk)
                        downloaded += len(chunk)
                        print_progress(downloaded, total_size)

            final_size = output_path.stat().st_size
            if remote_size and final_size != remote_size:
                raise urllib.error.ContentTooShortError(
                    f"retrieval incomplete: got {final_size} out of {remote_size} bytes",
                    None,
                )

            print(f"\nDone: {output_path}")
            return
        except (OSError, urllib.error.URLError, urllib.error.ContentTooShortError) as exc:
            print(f"\nDownload interrupted on attempt {attempt}/{retries}: {exc}")
            if attempt == retries:
                raise
            time.sleep(min(30, attempt * 5))


def main():
    args = parse_args()
    info = SAM_CHECKPOINTS[args.model_type]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / info["filename"]

    if output_path.exists() and args.overwrite:
        output_path.unlink()

    remote_size = get_remote_size(info["url"])
    if output_path.exists() and not args.overwrite and remote_size and output_path.stat().st_size == remote_size:
        print(f"Checkpoint already exists: {output_path}")
        return

    print(f"Downloading {args.model_type} checkpoint from {info['url']}")
    print(f"Saving to {output_path}")
    download_with_resume(info["url"], output_path, args.retries)


if __name__ == "__main__":
    main()
