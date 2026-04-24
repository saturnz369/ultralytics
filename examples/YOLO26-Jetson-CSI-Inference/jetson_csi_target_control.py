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

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
for path in (REPO_ROOT, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ultralytics import YOLO

from jetson_csi_target_follow import (
    IMAGE_SUFFIXES,
    TrackCandidate,
    choose_candidate,
    draw_candidate_box,
    draw_crosshair,
    draw_label,
    ensure_parent_dir,
    extract_candidates,
    find_candidate_by_id,
    is_file_source,
    open_stream,
    should_disable_half,
)


@dataclass
class ControlState:
    """Virtual gimbal command state for one frame."""

    frame_idx: int
    timestamp: float
    source: str
    selection: str
    status: str
    command_status: str
    visible_tracks: int
    lost_frames: int
    target_id: int | None
    class_id: int | None
    class_name: str | None
    confidence: float | None
    dx_norm: float | None
    dy_norm: float | None
    smooth_dx_norm: float
    smooth_dy_norm: float
    pan_cmd: float
    tilt_cmd: float
    pan_gain: float
    tilt_gain: float
    deadzone: float


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Preview virtual gimbal control commands from Ultralytics YOLO target-follow output."
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
    parser.add_argument("--window-title", default="YOLO26 Jetson Virtual Gimbal Control", help="OpenCV window title")
    parser.add_argument("--exit-key", default="q", help="Keyboard key used to stop live control preview")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames for testing; 0 means unlimited")
    parser.add_argument("--tracker", default="bytetrack.yaml", help="Tracker config, e.g. bytetrack.yaml or botsort.yaml")
    parser.add_argument(
        "--selection",
        default="center",
        choices=("center", "largest", "manual-id"),
        help="Target selection rule for automatic target lock",
    )
    parser.add_argument("--target-id", type=int, default=None, help="Manual track ID when --selection manual-id is used")
    parser.add_argument("--lost-buffer", type=int, default=15, help="Frames to hold target lock before reselection")
    parser.add_argument("--show-all-tracks", action="store_true", help="Draw non-selected tracks in a muted style")
    parser.add_argument("--print-state", action="store_true", help="Print per-frame control state as JSON lines")
    parser.add_argument("--state-file", default=None, help="Optional JSONL file path for virtual-control output")

    parser.add_argument("--pan-gain", type=float, default=0.65, help="Proportional gain for pan command")
    parser.add_argument("--tilt-gain", type=float, default=0.65, help="Proportional gain for tilt command")
    parser.add_argument("--deadzone", type=float, default=0.04, help="Normalized error dead-zone around frame center")
    parser.add_argument("--smooth-alpha", type=float, default=0.35, help="EMA alpha for command smoothing")
    parser.add_argument("--max-command", type=float, default=1.0, help="Absolute clamp for virtual pan/tilt command")
    parser.add_argument("--invert-pan", action="store_true", help="Invert pan command sign")
    parser.add_argument("--invert-tilt", action="store_true", help="Invert tilt command sign")

    parser.add_argument("--sensor-id", type=int, default=0, help="Jetson CSI sensor ID")
    parser.add_argument("--capture-width", type=int, default=1280, help="CSI capture width from nvarguscamerasrc")
    parser.add_argument("--capture-height", type=int, default=720, help="CSI capture height from nvarguscamerasrc")
    parser.add_argument("--display-width", type=int, default=1280, help="BGR frame width exposed to the app")
    parser.add_argument("--display-height", type=int, default=720, help="BGR frame height exposed to the app")
    parser.add_argument("--framerate", type=int, default=30, help="CSI framerate")
    parser.add_argument("--flip-method", type=int, default=0, help="Jetson nvvidconv flip-method")
    return parser.parse_args()


def default_output_path(source: str) -> Path:
    """Generate a default output path for virtual-control videos."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    source_name = "csi" if source == "csi" else Path(source).stem or "camera"
    return Path("runs") / "jetson_target_control" / f"{source_name}_{timestamp}.mp4"


def clamp(value: float, limit: float) -> float:
    """Clamp a value to +/- limit."""
    limit = abs(limit)
    return max(-limit, min(limit, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply a center dead-zone to a normalized error value."""
    deadzone = max(0.0, min(abs(deadzone), 0.99))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return scaled if value > 0 else -scaled


def smooth(previous: float | None, current: float, alpha: float) -> float:
    """Exponential moving average smoothing."""
    alpha = max(0.0, min(alpha, 1.0))
    if previous is None:
        return current
    return alpha * current + (1.0 - alpha) * previous


def compute_commands(
    dx_norm: float | None,
    dy_norm: float | None,
    previous_pan_error: float | None,
    previous_tilt_error: float | None,
    args: argparse.Namespace,
) -> tuple[float, float, float, float, str]:
    """Convert normalized target error into virtual gimbal commands."""
    if dx_norm is None or dy_norm is None:
        pan_error = smooth(previous_pan_error, 0.0, args.smooth_alpha)
        tilt_error = smooth(previous_tilt_error, 0.0, args.smooth_alpha)
        return 0.0, 0.0, pan_error, tilt_error, "no_target"

    raw_pan_error = apply_deadzone(dx_norm, args.deadzone)
    raw_tilt_error = apply_deadzone(dy_norm, args.deadzone)
    pan_error = smooth(previous_pan_error, raw_pan_error, args.smooth_alpha)
    tilt_error = smooth(previous_tilt_error, raw_tilt_error, args.smooth_alpha)

    pan_cmd = clamp(pan_error * args.pan_gain, args.max_command)
    tilt_cmd = clamp(tilt_error * args.tilt_gain, args.max_command)
    if args.invert_pan:
        pan_cmd *= -1.0
    if args.invert_tilt:
        tilt_cmd *= -1.0

    command_status = "deadzone" if pan_cmd == 0.0 and tilt_cmd == 0.0 else "active"
    return pan_cmd, tilt_cmd, pan_error, tilt_error, command_status


def draw_command_overlay(
    frame,
    source_label: str,
    status: str,
    command_status: str,
    target: TrackCandidate | None,
    fps: float,
    visible_tracks: int,
    lost_frames: int,
    dx_norm: float | None,
    dy_norm: float | None,
    pan_cmd: float,
    tilt_cmd: float,
) -> None:
    """Draw compact virtual-control HUD."""
    target_text = "none" if target is None else f"id:{target.track_id} {target.class_name}"
    err_text = "err: n/a"
    if dx_norm is not None and dy_norm is not None:
        err_text = f"err_norm {dx_norm:+.2f},{dy_norm:+.2f}"
    lines = [
        f"{source_label} | virtual gimbal | {status}",
        f"target {target_text} | tracks {visible_tracks} | lost {lost_frames}",
        f"cmd pan {pan_cmd:+.2f} tilt {tilt_cmd:+.2f} | {command_status}",
        f"fps {fps:.1f} | {err_text}",
    ]
    y = 22
    for text in lines:
        cv2.rectangle(frame, (8, y - 15), (470, y + 6), (0, 0, 0), -1)
        cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        y += 23


def emit_state(state: ControlState, print_state: bool, state_file_handle) -> None:
    """Emit the virtual-control state to stdout and/or JSONL."""
    payload = json.dumps(asdict(state), separators=(",", ":"))
    if print_state:
        print(payload)
    if state_file_handle is not None:
        state_file_handle.write(payload + "\n")
        state_file_handle.flush()


def build_control_state(
    frame_idx: int,
    source_label: str,
    selection: str,
    status: str,
    command_status: str,
    visible_tracks: int,
    lost_frames: int,
    selected: TrackCandidate | None,
    frame_width: int,
    frame_height: int,
    smooth_dx_norm: float,
    smooth_dy_norm: float,
    pan_cmd: float,
    tilt_cmd: float,
    args: argparse.Namespace,
) -> ControlState:
    """Create a serializable virtual-control record."""
    dx_norm = None
    dy_norm = None
    if selected is not None:
        dx_norm = (selected.cx - frame_width / 2.0) / max(frame_width / 2.0, 1.0)
        dy_norm = (selected.cy - frame_height / 2.0) / max(frame_height / 2.0, 1.0)

    return ControlState(
        frame_idx=frame_idx,
        timestamp=time.time(),
        source=source_label,
        selection=selection,
        status=status,
        command_status=command_status,
        visible_tracks=visible_tracks,
        lost_frames=lost_frames,
        target_id=None if selected is None else selected.track_id,
        class_id=None if selected is None else selected.class_id,
        class_name=None if selected is None else selected.class_name,
        confidence=None if selected is None else selected.conf,
        dx_norm=dx_norm,
        dy_norm=dy_norm,
        smooth_dx_norm=smooth_dx_norm,
        smooth_dy_norm=smooth_dy_norm,
        pan_cmd=pan_cmd,
        tilt_cmd=tilt_cmd,
        pan_gain=args.pan_gain,
        tilt_gain=args.tilt_gain,
        deadzone=args.deadzone,
    )


def run_stream(model: YOLO, args: argparse.Namespace) -> None:
    """Run virtual gimbal-control preview on a video-like source."""
    if Path(args.source).suffix.lower() in IMAGE_SUFFIXES:
        raise ValueError("Target-control expects a video-like source, webcam, or CSI stream, not a single image.")
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
    smooth_pan_error: float | None = None
    smooth_tilt_error: float | None = None

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

            control_target = selected
            if selected is None and status == "lost" and last_selected is not None:
                control_target = last_selected

            dx_norm = None
            dy_norm = None
            if control_target is not None:
                dx_norm = (control_target.cx - frame_width / 2.0) / max(frame_width / 2.0, 1.0)
                dy_norm = (control_target.cy - frame_height / 2.0) / max(frame_height / 2.0, 1.0)

            pan_cmd, tilt_cmd, smooth_pan_error, smooth_tilt_error, command_status = compute_commands(
                dx_norm=dx_norm,
                dy_norm=dy_norm,
                previous_pan_error=smooth_pan_error,
                previous_tilt_error=smooth_tilt_error,
                args=args,
            )
            if status in {"lost", "searching"}:
                command_status = status
                pan_cmd = 0.0
                tilt_cmd = 0.0

            annotated = frame.copy()
            draw_crosshair(annotated, frame_center[0], frame_center[1])

            if args.show_all_tracks:
                for candidate in candidates:
                    if control_target is not None and candidate.track_id == control_target.track_id:
                        continue
                    draw_candidate_box(annotated, candidate, color=(110, 110, 110), thickness=1)
                    draw_label(
                        annotated,
                        candidate.x1,
                        candidate.y1,
                        f"id:{candidate.track_id} {candidate.class_name}",
                        color=(110, 110, 110),
                    )

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

            current_time = time.perf_counter()
            delta = max(current_time - previous_time, 1e-6)
            fps_samples.append(1.0 / delta)
            fps_samples = fps_samples[-30:]
            previous_time = current_time
            fps = sum(fps_samples) / len(fps_samples)

            draw_command_overlay(
                annotated,
                source_label=source_label,
                status=status,
                command_status=command_status,
                target=control_target,
                fps=fps,
                visible_tracks=len(candidates),
                lost_frames=lost_frames,
                dx_norm=dx_norm,
                dy_norm=dy_norm,
                pan_cmd=pan_cmd,
                tilt_cmd=tilt_cmd,
            )
            last_frame = annotated

            state = build_control_state(
                frame_idx=frame_count,
                source_label=source_label,
                selection=args.selection,
                status=status,
                command_status=command_status,
                visible_tracks=len(candidates),
                lost_frames=lost_frames,
                selected=control_target if status != "searching" else None,
                frame_width=frame_width,
                frame_height=frame_height,
                smooth_dx_norm=smooth_pan_error or 0.0,
                smooth_dy_norm=smooth_tilt_error or 0.0,
                pan_cmd=pan_cmd,
                tilt_cmd=tilt_cmd,
                args=args,
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
        print(f"Saved virtual-control JSONL to {args.state_file}")


def main() -> None:
    """Run virtual target-control preview."""
    args = parse_args()
    if args.half and should_disable_half(args.device):
        print(f"Disabling --half because device '{args.device}' does not support this inference path.")
        args.half = False
    model = YOLO(args.model)
    run_stream(model, args)


if __name__ == "__main__":
    main()
