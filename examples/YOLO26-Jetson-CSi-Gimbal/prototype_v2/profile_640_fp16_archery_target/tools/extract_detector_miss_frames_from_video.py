#!/usr/bin/env python3
from __future__ import annotations

import argparse
import faulthandler
import json
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
from ultralytics import YOLO


@dataclass
class Segment:
    start_frame: int
    end_frame: int
    length: int


@dataclass
class ExtractedFrame:
    image_name: str
    segment_index: int
    frame_idx: int
    segment_length_frames: int
    segment_start_frame: int
    segment_end_frame: int
    detector_conf_threshold: float
    class_id: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fallback extractor: find consecutive detector-miss windows from a recorded video."
    )
    parser.add_argument("--video", required=True, help="Recorded video path")
    parser.add_argument("--weights", required=True, help="YOLO weights path")
    parser.add_argument("--class-id", type=int, default=0, help="Class ID treated as target present")
    parser.add_argument("--conf", type=float, default=0.10, help="Detection confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    parser.add_argument("--min-miss-frames", type=int, default=6, help="Minimum consecutive miss length")
    parser.add_argument("--max-miss-frames", type=int, default=600, help="Maximum consecutive miss length")
    parser.add_argument("--frames-per-segment", type=int, default=4, help="Frames to save per miss segment")
    parser.add_argument("--device", default="", help="Optional YOLO device, for example '0' or 'cpu'")
    parser.add_argument("--progress-every", type=int, default=300, help="Print progress every N frames")
    parser.add_argument("--limit-frames", type=int, default=0, help="Optional frame limit for debugging")
    parser.add_argument(
        "--output-root",
        default="",
        help="Optional output directory; defaults beside the input video",
    )
    return parser.parse_args()


def choose_row_positions(length: int, frames_per_segment: int) -> list[int]:
    if frames_per_segment <= 1 or length <= 1:
        return [length // 2]
    fractions = [(idx + 1) / (frames_per_segment + 1) for idx in range(frames_per_segment)]
    positions = sorted({min(length - 1, max(0, int(round((length - 1) * frac)))) for frac in fractions})
    return positions


def build_miss_segments(present_flags: list[bool], min_len: int, max_len: int) -> list[Segment]:
    if not present_flags:
        return []

    segments: list[Segment] = []
    current_absent = not present_flags[0]
    start = 0

    for idx, present in enumerate(present_flags[1:], start=1):
        absent = not present
        if absent == current_absent:
            continue
        if current_absent:
            length = idx - start
            if min_len <= length <= max_len:
                segments.append(Segment(start_frame=start, end_frame=idx - 1, length=length))
        current_absent = absent
        start = idx

    if current_absent:
        length = len(present_flags) - start
        if min_len <= length <= max_len:
            segments.append(Segment(start_frame=start, end_frame=len(present_flags) - 1, length=length))
    return segments


def collect_present_flags(
    model: YOLO,
    video_path: Path,
    imgsz: int,
    conf: float,
    class_id: int,
    device: str,
    progress_every: int,
    limit_frames: int,
) -> list[bool]:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    present_flags: list[bool] = []
    frame_idx = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            predict_kwargs = {
                "source": frame,
                "imgsz": imgsz,
                "conf": conf,
                "verbose": False,
            }
            if device:
                predict_kwargs["device"] = device

            results = model.predict(**predict_kwargs)
            present = False
            if results:
                boxes = results[0].boxes
                if boxes is not None and boxes.cls is not None:
                    for cls_value in boxes.cls.tolist():
                        if int(cls_value) == class_id:
                            present = True
                            break
            present_flags.append(present)
            frame_idx += 1

            if progress_every > 0 and frame_idx % progress_every == 0:
                if total_frames > 0:
                    print(f"[progress] scanned {frame_idx}/{total_frames} frames")
                else:
                    print(f"[progress] scanned {frame_idx} frames")

            if limit_frames > 0 and frame_idx >= limit_frames:
                break
    finally:
        cap.release()

    return present_flags


def save_selected_frames(
    video_path: Path,
    output_root: Path,
    miss_segments: list[Segment],
    frames_per_segment: int,
    conf: float,
    class_id: int,
) -> list[ExtractedFrame]:
    targets: dict[int, list[tuple[int, int, Segment]]] = {}
    for seg_idx, seg in enumerate(miss_segments, start=1):
        positions = choose_row_positions(seg.length, frames_per_segment)
        for shot_idx, relative_pos in enumerate(positions, start=1):
            frame_idx = seg.start_frame + relative_pos
            targets.setdefault(frame_idx, []).append((seg_idx, shot_idx, seg))

    if not targets:
        return []

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    extracted: list[ExtractedFrame] = []
    frame_idx = 0
    final_target = max(targets)

    try:
        while frame_idx <= final_target:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            if frame_idx in targets:
                for seg_idx, shot_idx, seg in targets[frame_idx]:
                    image_name = f"miss_{seg_idx:03d}_f{frame_idx}_p{shot_idx}.jpg"
                    image_path = output_root / image_name
                    if not cv2.imwrite(str(image_path), frame):
                        continue
                    extracted.append(
                        ExtractedFrame(
                            image_name=image_name,
                            segment_index=seg_idx,
                            frame_idx=frame_idx,
                            segment_length_frames=seg.length,
                            segment_start_frame=seg.start_frame,
                            segment_end_frame=seg.end_frame,
                            detector_conf_threshold=conf,
                            class_id=class_id,
                        )
                    )
            frame_idx += 1
    finally:
        cap.release()

    return extracted


def main() -> int:
    faulthandler.enable()
    args = parse_args()
    video_path = Path(args.video)
    weights_path = Path(args.weights)
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")
    if not weights_path.exists():
        raise SystemExit(f"Weights not found: {weights_path}")

    if args.output_root:
        output_root = Path(args.output_root)
    else:
        output_root = video_path.parent / f"{video_path.stem}_detector_miss_frames"
    output_root.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(weights_path))
    present_flags = collect_present_flags(
        model=model,
        video_path=video_path,
        imgsz=args.imgsz,
        conf=args.conf,
        class_id=args.class_id,
        device=args.device,
        progress_every=args.progress_every,
        limit_frames=args.limit_frames,
    )

    miss_segments = build_miss_segments(
        present_flags=present_flags,
        min_len=args.min_miss_frames,
        max_len=args.max_miss_frames,
    )

    extracted = save_selected_frames(
        video_path=video_path,
        output_root=output_root,
        miss_segments=miss_segments,
        frames_per_segment=args.frames_per_segment,
        conf=args.conf,
        class_id=args.class_id,
    )

    manifest_path = output_root / "manifest.jsonl"
    with manifest_path.open("w", encoding="utf-8") as handle:
        for item in extracted:
            handle.write(json.dumps(asdict(item), ensure_ascii=False) + "\n")

    summary = {
        "mode": "detector_miss_fallback",
        "video": str(video_path),
        "weights": str(weights_path),
        "class_id": args.class_id,
        "conf": args.conf,
        "imgsz": args.imgsz,
        "frames_total": len(present_flags),
        "selected_miss_segments": len(miss_segments),
        "extracted_images": len(extracted),
        "min_miss_frames": args.min_miss_frames,
        "max_miss_frames": args.max_miss_frames,
        "frames_per_segment": args.frames_per_segment,
    }
    (output_root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    zip_path = output_root.with_suffix(".zip")
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(output_root.rglob("*")):
            if path.is_file():
                archive.write(path, arcname=path.relative_to(output_root.parent))

    print(json.dumps(summary, indent=2))
    print(f"output_dir={output_root}")
    print(f"zip_file={zip_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
