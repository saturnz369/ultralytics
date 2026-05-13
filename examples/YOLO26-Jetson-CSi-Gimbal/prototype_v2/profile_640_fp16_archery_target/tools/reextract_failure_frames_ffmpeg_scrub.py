#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import zipfile
from pathlib import Path


# Jetson's ffmpeg delogo filter rejects regions that start exactly at 0,0.
# Keep the mask slightly inset while still covering the debug panel and crosshair.
FFMPEG_FILTER = "delogo=x=1:y=1:w=438:h=94,delogo=x=613:y=333:w=54:h=54"


def run_ffmpeg_extract(video: Path, time_s: float, output_path: Path) -> None:
    cmd = [
        "ffmpeg",
        "-y",
        "-ss",
        f"{time_s:.6f}",
        "-i",
        str(video),
        "-frames:v",
        "1",
        "-vf",
        FFMPEG_FILTER,
        str(output_path),
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def zip_dir(src_dir: Path, zip_path: Path) -> None:
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(src_dir.rglob("*")):
            if path.is_file():
                zf.write(path, arcname=str(path.relative_to(src_dir.parent)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Re-extract failure frames with ffmpeg delogo filters.")
    parser.add_argument("--input-dir", required=True, help="Existing extracted failure-frame directory.")
    parser.add_argument("--video", required=True, help="Source video used for extraction.")
    parser.add_argument("--output-dir", help="Optional output directory. Defaults to <input>_ffmpeg_scrubbed.")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).expanduser().resolve()
    video = Path(args.video).expanduser().resolve()
    if not input_dir.is_dir():
        raise SystemExit(f"Input directory not found: {input_dir}")
    if not video.is_file():
        raise SystemExit(f"Video not found: {video}")

    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else input_dir.with_name(f"{input_dir.name}_ffmpeg_scrubbed")
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    manifest_path = input_dir / "manifest.jsonl"
    summary_path = input_dir / "summary.json"
    if not manifest_path.is_file():
        raise SystemExit(f"Manifest missing: {manifest_path}")

    image_count = 0
    with manifest_path.open() as manifest_fp:
        for line in manifest_fp:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            image_name = record["image_name"]
            video_time_s = float(record["video_time_s"])
            run_ffmpeg_extract(video, video_time_s, output_dir / image_name)
            image_count += 1

    shutil.copy2(manifest_path, output_dir / "manifest.jsonl")
    if summary_path.is_file():
        shutil.copy2(summary_path, output_dir / "summary.json")

    zip_path = output_dir.with_suffix(".zip")
    zip_dir(output_dir, zip_path)

    print(f"input_dir={input_dir}")
    print(f"video={video}")
    print(f"output_dir={output_dir}")
    print(f"zip_file={zip_path}")
    print(f"images={image_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
