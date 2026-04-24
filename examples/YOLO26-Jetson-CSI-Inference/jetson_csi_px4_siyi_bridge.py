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

from jetson_csi_target_control import compute_commands
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
from px4_siyi_live_bridge import PX4SiyiBridge, mavutil


@dataclass
class BridgeState:
    """Per-frame vision and MAVLink bridge state."""

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
    control_mode: str
    yaw_rate_dps: float
    pitch_rate_dps: float
    yaw_target_deg: float
    pitch_target_deg: float
    yaw_flag: str
    mavlink_connected: bool
    mavlink_link: str
    px4_target_system: int
    px4_target_component: int
    gimbal_device_id: int
    last_configure_ack: str | None
    last_pitchyaw_send: float | None


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run YOLO target tracking and bridge virtual pan/tilt commands to PX4 -> SIYI over MAVLink."
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
    parser.add_argument("--window-title", default="YOLO26 PX4 SIYI Bridge", help="OpenCV window title")
    parser.add_argument("--exit-key", default="q", help="Keyboard key used to stop live bridge mode")
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
    parser.add_argument("--print-state", action="store_true", help="Print per-frame bridge state as JSON lines")
    parser.add_argument("--state-file", default=None, help="Optional JSONL file path for bridge-state output")

    parser.add_argument("--pan-gain", type=float, default=0.65, help="Proportional gain for pan command")
    parser.add_argument("--tilt-gain", type=float, default=0.65, help="Proportional gain for tilt command")
    parser.add_argument("--deadzone", type=float, default=0.04, help="Normalized error dead-zone around frame center")
    parser.add_argument("--smooth-alpha", type=float, default=0.35, help="EMA alpha for command smoothing")
    parser.add_argument("--max-command", type=float, default=1.0, help="Absolute clamp for virtual pan/tilt command")
    parser.add_argument("--invert-pan", action="store_true", help="Invert pan command sign before MAVLink mapping")
    parser.add_argument("--invert-tilt", action="store_true", help="Invert tilt command sign before MAVLink mapping")

    parser.add_argument("--serial-device", default="/dev/ttyUSB0", help="Jetson -> PX4 MAVLink serial device")
    parser.add_argument("--serial-baud", type=int, default=921600, help="Jetson -> PX4 MAVLink baud rate")
    parser.add_argument("--mav-source-system", type=int, default=42, help="MAVLink source system ID for the Jetson")
    parser.add_argument(
        "--mav-source-component",
        type=int,
        default=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        help="MAVLink source component ID for the Jetson",
    )
    parser.add_argument("--mav-target-system", type=int, default=1, help="PX4 target system ID")
    parser.add_argument(
        "--mav-target-component",
        type=int,
        default=154,
        help="PX4 gimbal manager component ID (matches MNT_MAV_COMPID in this setup)",
    )
    parser.add_argument("--gimbal-device-id", type=int, default=154, help="SIYI gimbal device component ID")
    parser.add_argument("--heartbeat-timeout", type=float, default=5.0, help="Seconds to wait for PX4 heartbeat")
    parser.add_argument("--ack-timeout", type=float, default=2.0, help="Seconds to wait for COMMAND_ACK")
    parser.add_argument("--send-rate-hz", type=float, default=20.0, help="Maximum MAVLink pitch/yaw command rate")
    parser.add_argument(
        "--max-yaw-rate-dps", type=float, default=60.0, help="Maximum yaw rate in deg/s at pan_cmd=+/-1"
    )
    parser.add_argument(
        "--max-pitch-rate-dps", type=float, default=45.0, help="Maximum pitch rate in deg/s at tilt_cmd=+/-1"
    )
    parser.add_argument("--yaw-lock", action="store_true", help="Use yaw-lock mode instead of vehicle-frame yaw")
    parser.add_argument("--pitch-lock", action="store_true", help="Set the pitch-lock flag in manager commands")
    parser.add_argument("--mav-invert-pan", action="store_true", help="Invert pan when converting to yaw rate")
    parser.add_argument(
        "--mav-invert-tilt",
        action="store_true",
        help="Invert tilt when converting to pitch rate; off means positive tilt_cmd -> negative pitch rate",
    )
    parser.add_argument(
        "--live-control-mode",
        default="angle-target",
        choices=("angle-target", "rate"),
        help="Use angle-target integration or direct rate commands for live CSI bridge output",
    )
    parser.add_argument("--max-yaw-angle-deg", type=float, default=60.0, help="Clamp live yaw target within +/- this angle")
    parser.add_argument("--min-pitch-angle-deg", type=float, default=-45.0, help="Minimum live pitch target angle")
    parser.add_argument("--max-pitch-angle-deg", type=float, default=25.0, help="Maximum live pitch target angle")
    parser.add_argument(
        "--skip-configure",
        action="store_true",
        help="Skip MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE and assume control is already assigned",
    )
    parser.add_argument(
        "--control-api",
        default="attitude",
        choices=("attitude", "manual", "pitchyaw", "command", "message"),
        help="Send setpoints using GIMBAL_MANAGER_SET_ATTITUDE, SET_MANUAL_CONTROL, SET_PITCHYAW, or MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW",
    )
    parser.add_argument(
        "--secondary-control-system",
        type=int,
        default=-1,
        help="Secondary control sysid for configure command (-1 leaves unchanged, 0 clears)",
    )
    parser.add_argument(
        "--secondary-control-component",
        type=int,
        default=-1,
        help="Secondary control compid for configure command (-1 leaves unchanged, 0 clears)",
    )
    parser.add_argument("--dry-run-mavlink", action="store_true", help="Compute and display commands but do not send MAVLink")

    parser.add_argument("--sensor-id", type=int, default=0, help="Jetson CSI sensor ID")
    parser.add_argument("--capture-width", type=int, default=1280, help="CSI capture width from nvarguscamerasrc")
    parser.add_argument("--capture-height", type=int, default=720, help="CSI capture height from nvarguscamerasrc")
    parser.add_argument("--display-width", type=int, default=1280, help="BGR frame width exposed to the app")
    parser.add_argument("--display-height", type=int, default=720, help="BGR frame height exposed to the app")
    parser.add_argument("--framerate", type=int, default=30, help="CSI framerate")
    parser.add_argument("--flip-method", type=int, default=0, help="Jetson nvvidconv flip-method")
    return parser.parse_args()


def default_output_path(source: str) -> Path:
    """Generate a default output path for PX4 bridge videos."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    source_name = "csi" if source == "csi" else Path(source).stem or "camera"
    return Path("runs") / "jetson_px4_siyi_bridge" / f"{source_name}_{timestamp}.mp4"


def emit_state(state: BridgeState, print_state: bool, state_file_handle) -> None:
    """Emit the bridge state to stdout and/or JSONL."""
    payload = json.dumps(asdict(state), separators=(",", ":"))
    if print_state:
        print(payload)
    if state_file_handle is not None:
        state_file_handle.write(payload + "\n")
        state_file_handle.flush()
def draw_bridge_overlay(
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
    control_mode: str,
    yaw_rate_dps: float,
    pitch_rate_dps: float,
    yaw_target_deg: float,
    pitch_target_deg: float,
    bridge: "PX4SiyiBridge",
    yaw_flag: str,
) -> None:
    """Draw compact bridge HUD."""
    target_text = "none" if target is None else f"id:{target.track_id} {target.class_name}"
    err_text = "err_norm n/a"
    if dx_norm is not None and dy_norm is not None:
        err_text = f"err_norm {dx_norm:+.2f},{dy_norm:+.2f}"
    motion_text = f"mav yaw {yaw_rate_dps:+.1f} pitch {pitch_rate_dps:+.1f} dps"
    if control_mode == "angle-target":
        motion_text = f"mav yaw {yaw_target_deg:+.1f} pitch {pitch_target_deg:+.1f} deg"
    lines = [
        f"{source_label} | px4 s{bridge.args.mav_target_system}c{bridge.args.mav_target_component} -> g{bridge.args.gimbal_device_id}",
        f"{status} | target {target_text} | tracks {visible_tracks} | lost {lost_frames}",
        f"vcmd pan {pan_cmd:+.2f} tilt {tilt_cmd:+.2f} | {command_status}",
        f"{motion_text} | {control_mode} | {yaw_flag} | {bridge.link_state()}",
        f"fps {fps:.1f} | {err_text}",
    ]
    y = 22
    for text in lines:
        cv2.rectangle(frame, (8, y - 15), (620, y + 6), (0, 0, 0), -1)
        cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        y += 23


def build_bridge_state(
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
    control_mode: str,
    yaw_rate_dps: float,
    pitch_rate_dps: float,
    yaw_target_deg: float,
    pitch_target_deg: float,
    yaw_flag: str,
    bridge: "PX4SiyiBridge",
) -> BridgeState:
    """Create a serializable bridge record."""
    dx_norm = None
    dy_norm = None
    if selected is not None:
        dx_norm = (selected.cx - frame_width / 2.0) / max(frame_width / 2.0, 1.0)
        dy_norm = (selected.cy - frame_height / 2.0) / max(frame_height / 2.0, 1.0)

    return BridgeState(
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
        control_mode=control_mode,
        yaw_rate_dps=yaw_rate_dps,
        pitch_rate_dps=pitch_rate_dps,
        yaw_target_deg=yaw_target_deg,
        pitch_target_deg=pitch_target_deg,
        yaw_flag=yaw_flag,
        mavlink_connected=bridge.connected,
        mavlink_link=bridge.link_state(),
        px4_target_system=bridge.args.mav_target_system,
        px4_target_component=bridge.args.mav_target_component,
        gimbal_device_id=bridge.args.gimbal_device_id,
        last_configure_ack=bridge.last_configure_ack,
        last_pitchyaw_send=bridge.last_pitchyaw_send,
    )


def map_commands_to_rates(
    pan_cmd: float,
    tilt_cmd: float,
    args: argparse.Namespace,
) -> tuple[float, float]:
    """Map normalized virtual commands to PX4 gimbal-manager pitch/yaw rates."""
    yaw_rate_dps = pan_cmd * args.max_yaw_rate_dps
    if args.mav_invert_pan:
        yaw_rate_dps *= -1.0

    # Positive tilt_cmd means the target is below image center, so the gimbal usually
    # needs to pitch down. MAVLink pitch rate is positive when pitching up.
    pitch_rate_dps = -tilt_cmd * args.max_pitch_rate_dps
    if args.mav_invert_tilt:
        pitch_rate_dps *= -1.0

    return pitch_rate_dps, yaw_rate_dps


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a value within inclusive bounds."""
    return max(lower, min(upper, value))


def update_angle_targets(
    current_pitch_deg: float,
    current_yaw_deg: float,
    pitch_rate_dps: float,
    yaw_rate_dps: float,
    dt: float,
    args: argparse.Namespace,
) -> tuple[float, float]:
    """Integrate rate-like commands into bounded angle targets."""
    next_pitch = current_pitch_deg + pitch_rate_dps * dt
    next_yaw = current_yaw_deg + yaw_rate_dps * dt
    next_pitch = clamp(next_pitch, args.min_pitch_angle_deg, args.max_pitch_angle_deg)
    next_yaw = clamp(next_yaw, -args.max_yaw_angle_deg, args.max_yaw_angle_deg)
    return next_pitch, next_yaw


def run_stream(model: YOLO, args: argparse.Namespace) -> None:
    """Run target tracking and bridge the output to PX4."""
    if Path(args.source).suffix.lower() in IMAGE_SUFFIXES:
        raise ValueError("PX4 bridge expects a video-like source, webcam, or CSI stream, not a single image.")
    if args.selection == "manual-id" and args.target_id is None:
        raise ValueError("--selection manual-id requires --target-id")

    bridge = PX4SiyiBridge(args)
    bridge.connect()

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
    target_pitch_deg = 0.0
    target_yaw_deg = 0.0

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

            current_time = time.perf_counter()
            delta = min(max(current_time - previous_time, 1e-6), 0.1)
            pitch_rate_dps, yaw_rate_dps = map_commands_to_rates(pan_cmd, tilt_cmd, args)
            if args.live_control_mode == "angle-target":
                target_pitch_deg, target_yaw_deg = update_angle_targets(
                    current_pitch_deg=target_pitch_deg,
                    current_yaw_deg=target_yaw_deg,
                    pitch_rate_dps=pitch_rate_dps,
                    yaw_rate_dps=yaw_rate_dps,
                    dt=delta,
                    args=args,
                )
                bridge.send_pitchyaw(
                    pitch_angle_deg=target_pitch_deg,
                    yaw_angle_deg=target_yaw_deg,
                    force=False,
                    wait_ack=False,
                )
            else:
                bridge.send_rates(pitch_rate_dps, yaw_rate_dps)

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

            fps_samples.append(1.0 / delta)
            fps_samples = fps_samples[-30:]
            previous_time = current_time
            fps = sum(fps_samples) / len(fps_samples)

            draw_bridge_overlay(
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
                control_mode=args.live_control_mode,
                yaw_rate_dps=yaw_rate_dps,
                pitch_rate_dps=pitch_rate_dps,
                yaw_target_deg=target_yaw_deg,
                pitch_target_deg=target_pitch_deg,
                bridge=bridge,
                yaw_flag=bridge.yaw_flag,
            )
            last_frame = annotated

            state = build_bridge_state(
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
                control_mode=args.live_control_mode,
                yaw_rate_dps=yaw_rate_dps,
                pitch_rate_dps=pitch_rate_dps,
                yaw_target_deg=target_yaw_deg,
                pitch_target_deg=target_pitch_deg,
                yaw_flag=bridge.yaw_flag,
                bridge=bridge,
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
        bridge.close()
        cv2.destroyAllWindows()

    if args.save:
        print(f"Saved annotated video to {output_path}")
    if args.state_file:
        print(f"Saved bridge-state JSONL to {args.state_file}")


def main() -> None:
    """Run target tracking and stream commands to PX4."""
    args = parse_args()
    if args.half and should_disable_half(args.device):
        print(f"Disabling --half because device '{args.device}' does not support this inference path.")
        args.half = False
    model = YOLO(args.model)
    run_stream(model, args)


if __name__ == "__main__":
    main()
