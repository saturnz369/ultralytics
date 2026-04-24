#!/usr/bin/env python3
# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ultralytics import YOLO

IMAGE_SUFFIXES = {".bmp", ".dng", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


@dataclass
class TrackCandidate:
    """Tracked detection candidate for target-follow selection."""

    track_id: int
    class_id: int
    class_name: str
    conf: float
    x1: int
    y1: int
    x2: int
    y2: int
    cx: float
    cy: float
    width: float
    height: float
    area: float


@dataclass
class FollowState:
    """Per-frame target-follow state for future control integration."""

    frame_idx: int
    timestamp: float
    source: str
    selection: str
    status: str
    visible_tracks: int
    lost_frames: int
    target_id: int | None
    class_id: int | None
    class_name: str | None
    confidence: float | None
    cx: float | None
    cy: float | None
    dx: float | None
    dy: float | None
    dx_norm: float | None
    dy_norm: float | None
    width: float | None
    height: float | None
    area: float | None


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
        description="Run single-target follow on top of Ultralytics YOLO tracking for video, webcam, or Jetson CSI."
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
    parser.add_argument("--line-width", type=int, default=3, help="Annotation line width")
    parser.add_argument("--show", action="store_true", help="Show a live OpenCV window")
    parser.add_argument("--save", action="store_true", help="Save annotated output to disk")
    parser.add_argument("--output", default=None, help="Output path for saved video")
    parser.add_argument("--window-title", default="YOLO26 Jetson Target Follow", help="OpenCV window title")
    parser.add_argument("--exit-key", default="q", help="Keyboard key used to stop live follow mode")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames for testing; 0 means unlimited")
    parser.add_argument("--tracker", default="bytetrack.yaml", help="Tracker config, e.g. bytetrack.yaml or botsort.yaml")
    parser.add_argument(
        "--selection",
        default="center",
        choices=("center", "largest", "manual-id"),
        help="Target selection rule for automatic target lock",
    )
    parser.add_argument("--target-id", type=int, default=None, help="Manual track ID when --selection manual-id is used")
    parser.add_argument("--lost-buffer", type=int, default=15, help="Frames to hold the target lock before reselection")
    parser.add_argument("--show-all-tracks", action="store_true", help="Draw non-selected tracks in a muted style")
    parser.add_argument("--print-state", action="store_true", help="Print per-frame follow state as JSON lines")
    parser.add_argument("--state-file", default=None, help="Optional JSONL file path for follow-state output")

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
    """Generate a default output path for target-follow videos."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    source_name = "csi" if source == "csi" else Path(source).stem or "camera"
    return Path("runs") / "jetson_target_follow" / f"{source_name}_{timestamp}.mp4"


def ensure_parent_dir(path: Path) -> None:
    """Create the parent directory if needed."""
    path.parent.mkdir(parents=True, exist_ok=True)


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


def extract_candidates(result) -> list[TrackCandidate]:
    """Convert tracking results into structured target candidates."""
    boxes = result.boxes
    if boxes is None or not getattr(boxes, "is_track", False) or boxes.id is None:
        return []

    xyxy = boxes.xyxy.cpu().tolist()
    xywh = boxes.xywh.cpu().tolist()
    class_ids = boxes.cls.int().cpu().tolist()
    confs = boxes.conf.cpu().tolist()
    track_ids = boxes.id.int().cpu().tolist()
    names = result.names

    candidates: list[TrackCandidate] = []
    for track_id, class_id, conf, xyxy_box, xywh_box in zip(track_ids, class_ids, confs, xyxy, xywh):
        x1, y1, x2, y2 = [int(v) for v in xyxy_box]
        cx, cy, width, height = [float(v) for v in xywh_box]
        class_name = names[class_id] if names is not None else str(class_id)
        candidates.append(
            TrackCandidate(
                track_id=int(track_id),
                class_id=int(class_id),
                class_name=str(class_name),
                conf=float(conf),
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                cx=cx,
                cy=cy,
                width=width,
                height=height,
                area=max(width * height, 0.0),
            )
        )
    return candidates


def find_candidate_by_id(candidates: list[TrackCandidate], track_id: int | None) -> TrackCandidate | None:
    """Return the candidate matching the requested track ID."""
    if track_id is None:
        return None
    return next((candidate for candidate in candidates if candidate.track_id == track_id), None)


def choose_candidate(
    candidates: list[TrackCandidate],
    selection: str,
    frame_width: int,
    frame_height: int,
    manual_target_id: int | None,
) -> TrackCandidate | None:
    """Choose a new target according to the requested selection rule."""
    if not candidates:
        return None
    if selection == "manual-id":
        return find_candidate_by_id(candidates, manual_target_id)
    if selection == "largest":
        return max(candidates, key=lambda c: (c.area, c.conf))

    frame_cx = frame_width / 2.0
    frame_cy = frame_height / 2.0
    return min(candidates, key=lambda c: ((c.cx - frame_cx) ** 2 + (c.cy - frame_cy) ** 2, -c.area))


def draw_crosshair(frame: np.ndarray, center_x: int, center_y: int) -> None:
    """Draw a crosshair at the frame center."""
    color = (255, 255, 0)
    cv2.drawMarker(frame, (center_x, center_y), color, markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)


def draw_candidate_box(frame: np.ndarray, candidate: TrackCandidate, color: tuple[int, int, int], thickness: int) -> None:
    """Draw a bounding box for one tracked candidate."""
    cv2.rectangle(frame, (candidate.x1, candidate.y1), (candidate.x2, candidate.y2), color, thickness)


def draw_label(frame: np.ndarray, anchor_x: int, anchor_y: int, text: str, color: tuple[int, int, int]) -> None:
    """Draw a compact label box above the target."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    y1 = max(anchor_y - text_height - baseline - 8, 0)
    y2 = y1 + text_height + baseline + 8
    x2 = anchor_x + text_width + 12
    cv2.rectangle(frame, (anchor_x, y1), (x2, y2), color, -1)
    cv2.putText(frame, text, (anchor_x + 6, y2 - baseline - 4), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)


def draw_overlay(
    frame: np.ndarray,
    source_label: str,
    tracker_name: str,
    selection: str,
    fps: float,
    status: str,
    visible_tracks: int,
    selected: TrackCandidate | None,
    lost_frames: int,
    dx: float | None,
    dy: float | None,
) -> None:
    """Draw target-follow status and error overlays."""
    target_text = "none"
    if selected is not None:
        target_text = f"id:{selected.track_id} {selected.class_name}"
    error_text = "err: n/a"
    if dx is not None and dy is not None:
        error_text = f"err: {dx:+.0f},{dy:+.0f}px"
    lines = [
        f"{source_label} | {tracker_name} | {selection}",
        f"{status} | target {target_text} | lost {lost_frames}",
        f"fps {fps:.1f} | tracks {visible_tracks} | {error_text}",
    ]

    y = 22
    for text in lines:
        cv2.rectangle(frame, (8, y - 15), (420, y + 6), (0, 0, 0), -1)
        cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        y += 23


def build_follow_state(
    frame_idx: int,
    source_label: str,
    selection: str,
    status: str,
    visible_tracks: int,
    lost_frames: int,
    frame_width: int,
    frame_height: int,
    selected: TrackCandidate | None,
) -> FollowState:
    """Build the structured follow-state record for one frame."""
    frame_cx = frame_width / 2.0
    frame_cy = frame_height / 2.0
    if selected is None:
        return FollowState(
            frame_idx=frame_idx,
            timestamp=time.time(),
            source=source_label,
            selection=selection,
            status=status,
            visible_tracks=visible_tracks,
            lost_frames=lost_frames,
            target_id=None,
            class_id=None,
            class_name=None,
            confidence=None,
            cx=None,
            cy=None,
            dx=None,
            dy=None,
            dx_norm=None,
            dy_norm=None,
            width=None,
            height=None,
            area=None,
        )

    dx = selected.cx - frame_cx
    dy = selected.cy - frame_cy
    return FollowState(
        frame_idx=frame_idx,
        timestamp=time.time(),
        source=source_label,
        selection=selection,
        status=status,
        visible_tracks=visible_tracks,
        lost_frames=lost_frames,
        target_id=selected.track_id,
        class_id=selected.class_id,
        class_name=selected.class_name,
        confidence=selected.conf,
        cx=selected.cx,
        cy=selected.cy,
        dx=dx,
        dy=dy,
        dx_norm=dx / max(frame_cx, 1.0),
        dy_norm=dy / max(frame_cy, 1.0),
        width=selected.width,
        height=selected.height,
        area=selected.area,
    )


def emit_state(state: FollowState, print_state: bool, state_file_handle) -> None:
    """Emit the follow state to stdout and/or a JSONL file."""
    payload = json.dumps(asdict(state), separators=(",", ":"))
    if print_state:
        print(payload)
    if state_file_handle is not None:
        state_file_handle.write(payload + "\n")
        state_file_handle.flush()


def run_stream(model: YOLO, args: argparse.Namespace) -> None:
    """Run single-target follow mode on a video-like source."""
    if Path(args.source).suffix.lower() in IMAGE_SUFFIXES:
        raise ValueError("Target-follow expects a video-like source, webcam, or CSI stream, not a single image.")
    if args.selection == "manual-id" and args.target_id is None:
        raise ValueError("--selection manual-id requires --target-id")

    cap, source_label = open_stream(args)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open source '{args.source}'. "
            "If using --source csi, confirm Jetson Argus/GStreamer is available and the camera is connected."
        )

    output_path = Path(args.output) if args.output else default_output_path(args.source)
    writer = None
    state_file_handle = None
    last_frame = None
    fps_samples: list[float] = []
    previous_time = time.perf_counter()
    frame_count = 0
    locked_target_id = args.target_id if args.selection == "manual-id" else None
    lost_frames = 0
    last_selected: TrackCandidate | None = None

    if args.save:
        ensure_parent_dir(output_path)
    if args.state_file:
        state_path = Path(args.state_file)
        ensure_parent_dir(state_path)
        state_file_handle = state_path.open("w", encoding="utf-8")
    if args.show:
        cv2.namedWindow(args.window_title, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_count += 1
            frame_height, frame_width = frame.shape[:2]
            frame_center = (frame_width // 2, frame_height // 2)

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
            candidates = extract_candidates(result)

            selected = find_candidate_by_id(candidates, locked_target_id)
            status = "tracked"

            if selected is not None:
                status = "reacquired" if lost_frames > 0 else ("acquired" if last_selected is None else "tracked")
                lost_frames = 0
                last_selected = selected
            else:
                if locked_target_id is not None:
                    lost_frames += 1
                    status = "lost"
                    if args.selection != "manual-id" and lost_frames > args.lost_buffer:
                        locked_target_id = None
                else:
                    status = "searching"

                if locked_target_id is None:
                    selected = choose_candidate(
                        candidates=candidates,
                        selection=args.selection,
                        frame_width=frame_width,
                        frame_height=frame_height,
                        manual_target_id=args.target_id,
                    )
                    if selected is not None:
                        locked_target_id = selected.track_id
                        lost_frames = 0
                        status = "acquired"
                        last_selected = selected

            annotated = frame.copy()
            draw_crosshair(annotated, frame_center[0], frame_center[1])

            if args.show_all_tracks:
                for candidate in candidates:
                    if selected is not None and candidate.track_id == selected.track_id:
                        continue
                    draw_candidate_box(annotated, candidate, color=(110, 110, 110), thickness=1)
                    draw_label(
                        annotated,
                        candidate.x1,
                        candidate.y1,
                        f"id:{candidate.track_id} {candidate.class_name}",
                        color=(110, 110, 110),
                    )

            dx = None
            dy = None
            overlay_target = selected

            if selected is not None:
                draw_candidate_box(annotated, selected, color=(0, 255, 0), thickness=args.line_width)
                draw_label(
                    annotated,
                    selected.x1,
                    selected.y1,
                    f"TARGET id:{selected.track_id} {selected.class_name} {selected.conf:.2f}",
                    color=(0, 255, 0),
                )
                target_point = (int(round(selected.cx)), int(round(selected.cy)))
                cv2.circle(annotated, target_point, 5, (0, 255, 0), -1)
                cv2.line(annotated, frame_center, target_point, (0, 255, 255), 2, cv2.LINE_AA)
                dx = selected.cx - frame_center[0]
                dy = selected.cy - frame_center[1]
            elif status == "lost" and last_selected is not None:
                target_point = (int(round(last_selected.cx)), int(round(last_selected.cy)))
                cv2.circle(annotated, target_point, 6, (0, 0, 255), -1)
                cv2.line(annotated, frame_center, target_point, (0, 0, 255), 2, cv2.LINE_AA)
                draw_label(
                    annotated,
                    last_selected.x1,
                    max(last_selected.y1 - 26, 0),
                    f"LOST id:{last_selected.track_id} {last_selected.class_name}",
                    color=(0, 0, 255),
                )
                overlay_target = last_selected
                dx = last_selected.cx - frame_center[0]
                dy = last_selected.cy - frame_center[1]

            current_time = time.perf_counter()
            delta = max(current_time - previous_time, 1e-6)
            fps_samples.append(1.0 / delta)
            fps_samples = fps_samples[-30:]
            previous_time = current_time
            fps = sum(fps_samples) / len(fps_samples)

            draw_overlay(
                annotated,
                source_label=source_label,
                tracker_name=Path(args.tracker).name,
                selection=args.selection,
                fps=fps,
                status=status,
                visible_tracks=len(candidates),
                selected=overlay_target,
                lost_frames=lost_frames,
                dx=dx,
                dy=dy,
            )
            last_frame = annotated

            state = build_follow_state(
                frame_idx=frame_count,
                source_label=source_label,
                selection=args.selection,
                status=status,
                visible_tracks=len(candidates),
                lost_frames=lost_frames,
                frame_width=frame_width,
                frame_height=frame_height,
                selected=overlay_target if status != "searching" else None,
            )
            emit_state(state, print_state=args.print_state, state_file_handle=state_file_handle)

            if args.save and writer is None:
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
        if state_file_handle is not None:
            state_file_handle.close()
        cv2.destroyAllWindows()

    if args.save:
        print(f"Saved annotated video to {output_path}")
    if args.state_file:
        print(f"Saved follow-state JSONL to {args.state_file}")


def main() -> None:
    """Run the selected target-follow workflow."""
    args = parse_args()
    if args.half and should_disable_half(args.device):
        print(f"Disabling --half because device '{args.device}' does not support this inference path.")
        args.half = False
    model = YOLO(args.model)
    run_stream(model, args)


if __name__ == "__main__":
    main()
