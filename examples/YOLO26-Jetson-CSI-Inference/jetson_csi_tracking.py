#!/usr/bin/env python3
# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ultralytics import YOLO

IMAGE_SUFFIXES = {".bmp", ".dng", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def build_csi_gstreamer_pipeline(
    sensor_id: int,
    capture_width: int,
    capture_height: int,
    display_width: int,
    display_height: int,
    framerate: int,
    flip_method: int,
) -> str:
    """Build a Jetson CSI camera pipeline for appsink consumption."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink name=appsink drop=true max-buffers=1 sync=false"
    )


class GStreamerCapture:
    """Minimal appsink-based capture for Jetson CSI input when OpenCV lacks GStreamer support."""

    def __init__(self, pipeline: str, width: int, height: int, fps: float):
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self._gst = Gst
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            raise RuntimeError("Failed to locate appsink in GStreamer pipeline.")
        self.pipeline.set_state(Gst.State.PLAYING)
        change_ret, _, _ = self.pipeline.get_state(5 * Gst.SECOND)
        self.opened = change_ret in {Gst.StateChangeReturn.SUCCESS, Gst.StateChangeReturn.ASYNC}

    def isOpened(self) -> bool:
        """Return True if the pipeline is running."""
        return self.opened

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Read one BGR frame from appsink."""
        sample = self.appsink.emit("try-pull-sample", self._gst.SECOND)
        if sample is None:
            return False, None

        buffer = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        ok, map_info = buffer.map(self._gst.MapFlags.READ)
        if not ok:
            return False, None
        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(map_info)
        return True, frame

    def get(self, prop_id: int) -> float:
        """Return basic capture metadata for writer setup."""
        if prop_id == cv2.CAP_PROP_FPS:
            return float(self.fps)
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.width)
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.height)
        return 0.0

    def release(self) -> None:
        """Stop the GStreamer pipeline."""
        self.pipeline.set_state(self._gst.State.NULL)
        self.opened = False


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run Ultralytics YOLO26 tracking on video, webcam, or Jetson CSI camera."
    )
    parser.add_argument("--model", default="yolo26n.pt", help="Model path, e.g. yolo26n.pt or yolo26n.engine")
    parser.add_argument(
        "--source",
        default="0",
        help="Input source: video path, webcam index like 0, RTSP URL, or 'csi' for Jetson CSI camera",
    )
    parser.add_argument("--device", default="0", help="Inference device, e.g. cpu, 0, 0,1")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    parser.add_argument("--conf", type=float, default=0.10, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="IoU threshold")
    parser.add_argument("--max-det", type=int, default=300, help="Maximum detections per frame")
    parser.add_argument("--classes", type=int, nargs="+", default=None, help="Optional class filter, e.g. --classes 0")
    parser.add_argument("--half", action="store_true", help="Use FP16 inference where supported")
    parser.add_argument("--line-width", type=int, default=2, help="Annotation line width")
    parser.add_argument("--hide-labels", action="store_true", help="Hide class labels in output")
    parser.add_argument("--hide-conf", action="store_true", help="Hide confidence scores in output")
    parser.add_argument("--show", action="store_true", help="Show a live OpenCV window")
    parser.add_argument("--save", action="store_true", help="Save annotated output to disk")
    parser.add_argument("--output", default=None, help="Output path for saved video")
    parser.add_argument("--window-title", default="YOLO26 Jetson Tracking", help="OpenCV window title")
    parser.add_argument("--exit-key", default="q", help="Keyboard key used to stop live tracking")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames for testing; 0 means unlimited")
    parser.add_argument("--tracker", default="bytetrack.yaml", help="Tracker config, e.g. bytetrack.yaml or botsort.yaml")
    parser.add_argument("--show-trails", action="store_true", help="Draw short motion trails for tracked IDs")
    parser.add_argument("--trail-len", type=int, default=30, help="Track history length for trail drawing")

    parser.add_argument("--sensor-id", type=int, default=0, help="Jetson CSI sensor ID")
    parser.add_argument("--capture-width", type=int, default=1280, help="CSI capture width from nvarguscamerasrc")
    parser.add_argument("--capture-height", type=int, default=720, help="CSI capture height from nvarguscamerasrc")
    parser.add_argument("--display-width", type=int, default=1280, help="BGR frame width exposed to the app")
    parser.add_argument("--display-height", type=int, default=720, help="BGR frame height exposed to the app")
    parser.add_argument("--framerate", type=int, default=30, help="CSI framerate")
    parser.add_argument("--flip-method", type=int, default=0, help="Jetson nvvidconv flip-method")
    return parser.parse_args()


def should_disable_half(device: str | None) -> bool:
    """Return True when FP16 should be disabled for the selected device."""
    if device is None:
        return False
    return str(device).strip().lower() in {"cpu", "mps"}


def parse_capture_source(source: str) -> int | str:
    """Convert numeric webcam indices while leaving other sources untouched."""
    return int(source) if source.isdigit() else source


def is_file_source(source: str) -> bool:
    """Return True if the source points to an existing filesystem path."""
    return Path(source).exists()


def default_output_path(source: str) -> Path:
    """Generate a default output path for tracked videos."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    source_name = "csi" if source == "csi" else Path(source).stem or "camera"
    return Path("runs") / "jetson_tracking" / f"{source_name}_{timestamp}.mp4"


def ensure_parent_dir(path: Path) -> None:
    """Create the parent directory if needed."""
    path.parent.mkdir(parents=True, exist_ok=True)


def draw_overlay(frame: np.ndarray, source_label: str, fps: float, tracker_name: str, tracked_objects: int) -> None:
    """Draw source, tracker, and FPS overlays on the annotated frame."""
    lines = [
        f"source: {source_label}",
        f"tracker: {tracker_name}",
        f"fps: {fps:.1f}",
        f"tracks: {tracked_objects}",
    ]
    y = 28
    for text in lines:
        cv2.rectangle(frame, (10, y - 18), (330, y + 8), (0, 0, 0), -1)
        cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        y += 32


def draw_track_trails(
    frame: np.ndarray,
    result,
    track_history: dict[int, deque[tuple[int, int]]],
    trail_len: int,
) -> int:
    """Update and draw short motion trails for tracked IDs."""
    boxes = result.boxes
    if boxes is None or not getattr(boxes, "is_track", False) or boxes.id is None:
        return 0

    track_ids = boxes.id.int().cpu().tolist()
    centers = boxes.xywh.cpu().tolist()
    active_ids = set(track_ids)

    for track_id, (x, y, _, _) in zip(track_ids, centers):
        history = track_history.setdefault(track_id, deque(maxlen=trail_len))
        history.append((int(x), int(y)))

    stale_ids = [track_id for track_id in track_history if track_id not in active_ids]
    for track_id in stale_ids:
        if len(track_history[track_id]) <= 1:
            del track_history[track_id]

    for track_id in active_ids:
        points = np.array(track_history[track_id], dtype=np.int32)
        if len(points) >= 2:
            cv2.polylines(frame, [points.reshape((-1, 1, 2))], isClosed=False, color=(255, 255, 0), thickness=2)
        if len(points) >= 1:
            cv2.circle(frame, tuple(points[-1]), 3, (255, 255, 0), -1)

    return len(active_ids)


def open_stream(args: argparse.Namespace) -> tuple[cv2.VideoCapture, str]:
    """Open a CSI, webcam, video, or RTSP stream."""
    if args.source == "csi":
        pipeline = build_csi_gstreamer_pipeline(
            sensor_id=args.sensor_id,
            capture_width=args.capture_width,
            capture_height=args.capture_height,
            display_width=args.display_width,
            display_height=args.display_height,
            framerate=args.framerate,
            flip_method=args.flip_method,
        )
        cap = GStreamerCapture(
            pipeline=pipeline,
            width=args.display_width,
            height=args.display_height,
            fps=args.framerate,
        )
        return cap, f"csi(sensor={args.sensor_id})"

    parsed_source = parse_capture_source(args.source)
    return cv2.VideoCapture(parsed_source), str(parsed_source)


def run_stream(model: YOLO, args: argparse.Namespace) -> None:
    """Run tracking on a video-like source."""
    if Path(args.source).suffix.lower() in IMAGE_SUFFIXES:
        raise ValueError("Tracking expects a video-like source, webcam, or CSI stream, not a single image.")

    cap, source_label = open_stream(args)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open source '{args.source}'. "
            "If using --source csi, confirm Jetson Argus/GStreamer is available and the camera is connected."
        )

    output_path = Path(args.output) if args.output else default_output_path(args.source)
    writer = None
    last_frame = None
    fps_samples: deque[float] = deque(maxlen=30)
    previous_time = time.perf_counter()
    frame_count = 0
    track_history: dict[int, deque[tuple[int, int]]] = defaultdict(lambda: deque(maxlen=args.trail_len))

    if args.save:
        ensure_parent_dir(output_path)

    if args.show:
        cv2.namedWindow(args.window_title, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_count += 1

            result = model.track(
                source=frame,
                persist=True,
                tracker=args.tracker,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                max_det=args.max_det,
                classes=args.classes,
                half=args.half,
                device=args.device,
                verbose=False,
            )[0]
            annotated = result.plot(conf=not args.hide_conf, labels=not args.hide_labels, line_width=args.line_width)
            tracked_objects = 0
            if args.show_trails:
                tracked_objects = draw_track_trails(annotated, result, track_history, args.trail_len)
            elif result.boxes is not None and getattr(result.boxes, "is_track", False) and result.boxes.id is not None:
                tracked_objects = len(result.boxes.id)
            last_frame = annotated

            current_time = time.perf_counter()
            delta = max(current_time - previous_time, 1e-6)
            fps_samples.append(1.0 / delta)
            previous_time = current_time
            fps = sum(fps_samples) / len(fps_samples)
            draw_overlay(annotated, source_label, fps, Path(args.tracker).name, tracked_objects)

            if args.save and writer is None:
                frame_height, frame_width = annotated.shape[:2]
                source_fps = cap.get(cv2.CAP_PROP_FPS)
                write_fps = source_fps if source_fps and source_fps > 0 else max(fps, 1.0)
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(str(output_path), fourcc, write_fps, (frame_width, frame_height))
                if not writer.isOpened():
                    raise RuntimeError(f"Failed to create video writer for {output_path}")

            if writer is not None:
                writer.write(annotated)

            if args.show:
                cv2.imshow(args.window_title, annotated)
                if cv2.waitKey(1) & 0xFF == ord(args.exit_key):
                    break

            if args.max_frames and frame_count >= args.max_frames:
                break

        if args.show and last_frame is not None and is_file_source(args.source):
            cv2.imshow(args.window_title, last_frame)
            print("Playback finished. Press any key in the video window to close it.")
            cv2.waitKey(0)
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    if args.save:
        print(f"Saved annotated video to {output_path}")


def main() -> None:
    """Run the selected tracking workflow."""
    args = parse_args()
    if args.half and should_disable_half(args.device):
        print(f"Disabling --half because device '{args.device}' does not support this inference path.")
        args.half = False
    model = YOLO(args.model)
    run_stream(model, args)


if __name__ == "__main__":
    main()
