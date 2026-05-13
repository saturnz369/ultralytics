#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import zipfile
from pathlib import Path

import cv2
import numpy as np


def build_mask(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Color masks for the known OSD palette: green boxes/text, cyan crosshair, yellow guide line.
    green = cv2.inRange(hsv, np.array([35, 70, 70]), np.array([90, 255, 255]))
    cyan = cv2.inRange(hsv, np.array([80, 70, 70]), np.array([110, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([15, 70, 70]), np.array([40, 255, 255]))
    mask = cv2.bitwise_or(green, cyan)
    mask = cv2.bitwise_or(mask, yellow)

    # Fixed debug-text panel in the upper-left corner.
    text_panel_w = min(width, 440)
    text_panel_h = min(height, 96)
    cv2.rectangle(mask, (0, 0), (text_panel_w, text_panel_h), 255, thickness=-1)

    # Fixed center crosshair region.
    cx = width // 2
    cy = height // 2
    cv2.rectangle(mask, (max(0, cx - 28), max(0, cy - 28)),
                  (min(width - 1, cx + 28), min(height - 1, cy + 28)),
                  255, thickness=-1)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def scrub_image(src: Path, dst: Path) -> None:
    image = cv2.imread(str(src), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to read image: {src}")
    mask = build_mask(image)
    cleaned = cv2.inpaint(image, mask, 5, cv2.INPAINT_TELEA)
    if not cv2.imwrite(str(dst), cleaned):
        raise RuntimeError(f"Failed to write image: {dst}")


def zip_dir(src_dir: Path, zip_path: Path) -> None:
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(src_dir.rglob("*")):
            if path.is_file():
                zf.write(path, arcname=str(path.relative_to(src_dir.parent)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Scrub fixed overlays from extracted failure frames.")
    parser.add_argument("--input-dir", required=True, help="Directory containing extracted failure frames.")
    parser.add_argument("--output-dir", help="Optional output directory. Defaults to <input>_scrubbed.")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).expanduser().resolve()
    if not input_dir.is_dir():
        raise SystemExit(f"Input directory not found: {input_dir}")

    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else input_dir.with_name(f"{input_dir.name}_scrubbed")
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_count = 0
    for path in sorted(input_dir.iterdir()):
        if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}:
            scrub_image(path, output_dir / path.name)
            image_count += 1
        elif path.is_file():
            shutil.copy2(path, output_dir / path.name)

    zip_path = output_dir.with_suffix(".zip")
    zip_dir(output_dir, zip_path)

    print(f"input_dir={input_dir}")
    print(f"output_dir={output_dir}")
    print(f"zip_file={zip_path}")
    print(f"images={image_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
