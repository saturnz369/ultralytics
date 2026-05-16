#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import mmap
import os
import subprocess
import struct
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from px4_siyi_live_bridge import PX4SiyiBridge, mavutil


METADATA_IPC_MAGIC = 0x5056324D
METADATA_IPC_VERSION = 3
METADATA_IPC_PAYLOAD_MAX = 8192
METADATA_IPC_HEADER = struct.Struct("<IIIIQQqII")
METADATA_IPC_PAYLOAD = struct.Struct("<qqqqqQQIqIqIqIqII8xQiIddd64s32s32s")
METADATA_IPC_SIZE = METADATA_IPC_HEADER.size + METADATA_IPC_PAYLOAD_MAX


def env_int_default(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        return default


def env_float_default(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


def env_bool_default(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass
class PrototypeV2BridgeState:
    frame_idx: int
    timestamp: float
    # ===== LATENCY STAGE 1 ADDED START =====
    # Stage 1: DeepStream metadata output -> Python read -> MAVLink send point.
    metadata_sequence: int
    metadata_gap_frames: int
    video_frame_gap: int
    frame_pts_ns: int | None
    frame_ntp_ns: int | None
    ds_write_mono_ns: int | None
    py_read_mono_ns: int
    mav_send_mono_ns: int
    vision_latency_ms: float | None
    metadata_age_ms: float | None
    control_age_ms: float | None
    metadata_is_fresh: bool
    ds_to_py_ms: float | None
    py_to_mav_ms: float
    mavlink_delay_ms: float
    ds_to_mav_ms: float | None
    # ===== LATENCY STAGE 1 ADDED END =====

    # ===== LATENCY STAGE 2 ADDED START =====
    # Stage 2: C tracker-probe/control timing inside deepstream_yolo26_rtsp_target_control.c.
    c_probe_start_mono_ns: int | None
    c_control_done_mono_ns: int | None
    c_probe_to_control_ms: float | None
    c_control_to_json_ms: float | None
    c_probe_to_json_ms: float | None
    # ===== LATENCY STAGE 2 ADDED END =====

    # ===== LATENCY STAGE 3 ADDED START =====
    # Stage 3: coarse DeepStream element timing between selected pad probes.
    mux_src_mono_ns: int | None
    pgie_src_mono_ns: int | None
    tracker_src_mono_ns: int | None
    video_queue_frame_idx: int | None
    video_queue_src_mono_ns: int | None
    nvosd_frame_idx: int | None
    nvosd_src_mono_ns: int | None
    display_queue_frame_idx: int | None
    display_queue_src_mono_ns: int | None
    rtsp_queue_frame_idx: int | None
    rtsp_queue_src_mono_ns: int | None
    mux_to_pgie_ms: float | None
    pgie_to_tracker_ms: float | None
    mux_to_tracker_ms: float | None
    tracker_to_video_queue_ms: float | None
    video_queue_to_osd_ms: float | None
    tracker_to_osd_ms: float | None
    osd_to_display_queue_ms: float | None
    osd_to_rtsp_queue_ms: float | None
    display_frame_lag: int | None
    rtsp_frame_lag: int | None
    # ===== LATENCY STAGE 3 ADDED END =====

    # ===== LATENCY STAGE 4 ADDED START =====
    # Stage 4: Python MAVLink send function call duration.
    mav_return_mono_ns: int
    mav_call_ms: float
    # ===== LATENCY STAGE 4 ADDED END =====
    # ===== FEEDBACK TIMING ADDED START =====
    feedback_type: str | None
    feedback_mono_ns: int | None
    feedback_wall_time: float | None
    feedback_age_ms: float | None
    mav_send_to_feedback_ms: float | None
    feedback_delay_ms: float | None
    manager_status_mono_ns: int | None
    device_attitude_status_mono_ns: int | None
    # ===== FEEDBACK TIMING ADDED END =====
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
    metadata_state_file: str
    metadata_ipc_file: str


@dataclass
class PrototypeV2HealthState:
    timestamp: float
    camera_ok: bool
    deepstream_ok: bool
    inference_fps: float | None
    metadata_age_ms: float | None
    target_state: str
    target_id: int | None
    control_hz: float | None
    mavlink_ok: bool
    gimbal_feedback_ok: bool
    jetson_temp_c: float | None
    video_branch_ok: bool
    recording_ok: bool
    recording_enabled: bool


@dataclass
class LiveControlFilterState:
    smooth_pan_error: float = 0.0
    smooth_tilt_error: float = 0.0
    smooth_pan_feedforward_rate: float = 0.0
    smooth_tilt_feedforward_rate: float = 0.0
    last_pan_error: float = 0.0
    last_tilt_error: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run prototype_v2 DeepStream RTSP + metadata-control preview and bridge it to PX4 -> SIYI."
    )
    parser.add_argument("--metadata-state-file", default=None, help="JSONL file written by the combined DeepStream app")
    parser.add_argument("--metadata-ipc-file", default=None, help="Memory-mapped latest-only metadata snapshot file")
    parser.add_argument("--bridge-state-file", default=None, help="Optional JSONL file for bridge output")
    parser.add_argument("--print-state", action="store_true", help="Print per-frame bridge state as JSONL")
    parser.add_argument("--deepstream-log-file", default=None, help="Optional log file for the DeepStream subprocess")
    parser.add_argument("--show-deepstream-log", action="store_true", help="Show DeepStream subprocess output in this terminal")
    parser.add_argument(
        "--combined-launcher",
        default=str(SCRIPT_DIR / "run_deepstream_yolo26_rtsp_target_control.sh"),
        help="Launcher used to start the combined prototype_v2 DeepStream app",
    )
    parser.add_argument("--startup-timeout", type=float, default=20.0, help="Seconds to wait for metadata output startup")
    parser.add_argument("--poll-interval", type=float, default=0.005, help="Metadata polling interval in seconds")
    parser.add_argument("--camera-fps-n", type=int, default=env_int_default("CAMERA_FPS_N", 30))
    parser.add_argument("--camera-fps-d", type=int, default=env_int_default("CAMERA_FPS_D", 1))
    parser.add_argument("--show", action="store_true", default=env_bool_default("SHOW", False))
    parser.add_argument("--rtsp-enable", action="store_true", default=env_bool_default("RTSP_ENABLE", True))
    parser.add_argument("--raw-record-enable", action="store_true", default=env_bool_default("RAW_RECORD_ENABLE", False))
    parser.add_argument(
        "--metadata-max-age-ms",
        type=float,
        default=env_float_default("METADATA_MAX_AGE_MS", 150.0),
        help="Treat metadata older than this as stale and fall back to lost-target behavior",
    )
    parser.add_argument("--pan-gain", type=float, default=env_float_default("PAN_GAIN", 0.91))
    parser.add_argument("--tilt-gain", type=float, default=env_float_default("TILT_GAIN", 0.83))
    parser.add_argument("--deadzone", type=float, default=env_float_default("DEADZONE", 0.048))
    parser.add_argument("--smooth-alpha", type=float, default=env_float_default("SMOOTH_ALPHA", 0.39))
    parser.add_argument("--fast-smooth-alpha", type=float, default=env_float_default("FAST_SMOOTH_ALPHA", 0.81))
    parser.add_argument("--fast-error-zone", type=float, default=env_float_default("FAST_ERROR_ZONE", 0.15))
    parser.add_argument("--command-boost-zone", type=float, default=env_float_default("COMMAND_BOOST_ZONE", 0.095))
    parser.add_argument("--min-active-command", type=float, default=env_float_default("MIN_ACTIVE_COMMAND", 0.20))
    parser.add_argument("--response-gamma", type=float, default=env_float_default("RESPONSE_GAMMA", 0.63))
    parser.add_argument(
        "--pan-feedforward-gain",
        type=float,
        default=env_float_default("PAN_FEEDFORWARD_GAIN", 0.18),
    )
    parser.add_argument(
        "--tilt-feedforward-gain",
        type=float,
        default=env_float_default("TILT_FEEDFORWARD_GAIN", 0.12),
    )
    parser.add_argument("--feedforward-alpha", type=float, default=env_float_default("FEEDFORWARD_ALPHA", 0.40))
    parser.add_argument("--feedforward-limit", type=float, default=env_float_default("FEEDFORWARD_LIMIT", 0.14))
    parser.add_argument(
        "--feedforward-activation-zone",
        type=float,
        default=env_float_default("FEEDFORWARD_ACTIVATION_ZONE", 0.07),
    )
    parser.add_argument("--max-command", type=float, default=env_float_default("MAX_COMMAND", 1.0))
    parser.add_argument("--invert-pan", action="store_true", default=env_bool_default("INVERT_PAN", False))
    parser.add_argument("--invert-tilt", action="store_true", default=env_bool_default("INVERT_TILT", False))

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
    parser.add_argument(
        "--live-control-mode",
        default="angle-target",
        choices=("angle-target", "rate"),
        help="Use angle-target integration or direct rate commands",
    )
    parser.add_argument("--max-yaw-angle-deg", type=float, default=60.0, help="Clamp live yaw target within +/- this angle")
    parser.add_argument("--min-pitch-angle-deg", type=float, default=-45.0, help="Minimum live pitch target angle")
    parser.add_argument("--max-pitch-angle-deg", type=float, default=25.0, help="Maximum live pitch target angle")
    parser.add_argument("--yaw-lock", action="store_true", help="Use yaw-lock mode instead of vehicle-frame yaw")
    parser.add_argument("--pitch-lock", action="store_true", help="Set the pitch-lock flag in manager commands")
    parser.add_argument("--mav-invert-pan", action="store_true", help="Invert pan when converting to yaw rate")
    parser.add_argument(
        "--mav-invert-tilt",
        action="store_true",
        help="Invert tilt when converting to pitch rate; off means positive tilt_cmd -> negative pitch rate",
    )
    parser.add_argument("--skip-configure", action="store_true", help="Skip MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE")
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
    parser.add_argument(
        "--control-api",
        default="attitude",
        choices=("attitude", "manual", "pitchyaw", "command", "message"),
        help="Gimbal control API used by the bridge",
    )
    parser.add_argument(
        "--print-health",
        action="store_true",
        default=env_bool_default("PRINT_HEALTH", False),
        help="Print a low-rate field health summary derived from the latest bridge state",
    )
    parser.add_argument(
        "--health-print-interval-sec",
        type=float,
        default=env_float_default("HEALTH_PRINT_INTERVAL_SEC", 1.0),
        help="Minimum seconds between printed health summaries",
    )
    parser.add_argument(
        "--health-state-file",
        default=os.environ.get("HEALTH_STATE_FILE", ""),
        help="Optional JSONL file for low-rate system health summaries",
    )
    parser.add_argument(
        "--health-temp-zone",
        default=os.environ.get("HEALTH_TEMP_ZONE", "tj-thermal"),
        help="Preferred Linux thermal zone name for Jetson temperature reporting",
    )
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_symmetric(value: float, limit: float) -> float:
    abs_limit = abs(limit)
    if value > abs_limit:
        return abs_limit
    if value < -abs_limit:
        return -abs_limit
    return value


def apply_deadzone(value: float, deadzone: float) -> float:
    dz = abs(deadzone)
    dz = min(dz, 0.99)
    if abs(value) <= dz:
        return 0.0
    magnitude = (abs(value) - dz) / (1.0 - dz)
    return magnitude if value > 0.0 else -magnitude


def smooth_value(previous: float, current: float, alpha: float) -> float:
    a = clamp(alpha, 0.0, 1.0)
    return a * current + (1.0 - a) * previous


def select_smooth_alpha(error_mag: float, args: argparse.Namespace) -> float:
    base = clamp(args.smooth_alpha, 0.0, 1.0)
    fast = clamp(args.fast_smooth_alpha, 0.0, 1.0)
    zone = abs(args.fast_error_zone)

    if fast < base:
        fast = base
    if zone <= 1e-6:
        return fast
    if error_mag <= 0.0:
        return base
    if error_mag >= zone:
        return fast
    return base + ((fast - base) * (error_mag / zone))


def shape_command(error_value: float, gain: float, args: argparse.Namespace) -> float:
    limit = abs(args.max_command)
    raw = abs(error_value) * gain
    boost_zone = abs(args.command_boost_zone)
    min_active = abs(args.min_active_command)
    gamma = args.response_gamma

    if limit <= 1e-6 or raw <= 1e-6:
        return 0.0

    raw = clamp_symmetric(raw, limit)
    boost_zone = min(boost_zone, limit)
    min_active = min(min_active, limit)
    gamma = clamp(gamma, 0.10, 3.0)

    if raw <= boost_zone or boost_zone >= limit:
        return -raw if error_value < 0.0 else raw

    normalized = (raw - boost_zone) / (limit - boost_zone)
    normalized = clamp(normalized, 0.0, 1.0)
    boosted = min_active + ((limit - min_active) * math.pow(normalized, gamma))
    if boosted < raw:
        boosted = raw
    return -boosted if error_value < 0.0 else boosted


def apply_feedforward_term(
    base_cmd: float,
    error_value: float,
    error_rate: float,
    feedforward_gain: float,
    args: argparse.Namespace,
) -> float:
    limit = abs(args.max_command)
    activation_zone = abs(args.feedforward_activation_zone)
    ff_limit = abs(args.feedforward_limit)

    if limit <= 1e-6:
        return clamp_symmetric(base_cmd, limit)
    if abs(error_value) < activation_zone or abs(error_rate) <= 1e-6:
        return clamp_symmetric(base_cmd, limit)
    if (error_value * error_rate) <= 0.0:
        return clamp_symmetric(base_cmd, limit)

    ff = clamp_symmetric(abs(error_rate) * abs(feedforward_gain), ff_limit)
    return clamp_symmetric(base_cmd + math.copysign(ff, error_value), limit)


def compute_control_outputs(
    *,
    has_target: bool,
    dx_norm: float,
    dy_norm: float,
    dt: float,
    args: argparse.Namespace,
    filter_state: LiveControlFilterState,
) -> tuple[float, float, float, float, str]:
    nominal_dt = (args.camera_fps_d / args.camera_fps_n) if args.camera_fps_n > 0 else (1.0 / 30.0)

    if not has_target:
        filter_state.smooth_pan_error = smooth_value(filter_state.smooth_pan_error, 0.0, args.smooth_alpha)
        filter_state.smooth_tilt_error = smooth_value(filter_state.smooth_tilt_error, 0.0, args.smooth_alpha)
        filter_state.smooth_pan_feedforward_rate = smooth_value(
            filter_state.smooth_pan_feedforward_rate, 0.0, args.feedforward_alpha
        )
        filter_state.smooth_tilt_feedforward_rate = smooth_value(
            filter_state.smooth_tilt_feedforward_rate, 0.0, args.feedforward_alpha
        )
        filter_state.last_pan_error = 0.0
        filter_state.last_tilt_error = 0.0
        return filter_state.smooth_pan_error, filter_state.smooth_tilt_error, 0.0, 0.0, "no_target"

    pan_error = apply_deadzone(dx_norm, args.deadzone)
    tilt_error = apply_deadzone(dy_norm, args.deadzone)
    pan_alpha = select_smooth_alpha(abs(pan_error), args)
    tilt_alpha = select_smooth_alpha(abs(tilt_error), args)
    ff_alpha = clamp(args.feedforward_alpha, 0.0, 1.0)
    dt_sec = dt if 1e-3 <= dt <= 0.5 else nominal_dt

    filter_state.smooth_pan_error = smooth_value(filter_state.smooth_pan_error, pan_error, pan_alpha)
    filter_state.smooth_tilt_error = smooth_value(filter_state.smooth_tilt_error, tilt_error, tilt_alpha)

    pan_error_rate = (pan_error - filter_state.last_pan_error) / dt_sec
    tilt_error_rate = (tilt_error - filter_state.last_tilt_error) / dt_sec

    filter_state.smooth_pan_feedforward_rate = smooth_value(
        filter_state.smooth_pan_feedforward_rate, pan_error_rate, ff_alpha
    )
    filter_state.smooth_tilt_feedforward_rate = smooth_value(
        filter_state.smooth_tilt_feedforward_rate, tilt_error_rate, ff_alpha
    )

    filter_state.last_pan_error = pan_error
    filter_state.last_tilt_error = tilt_error

    pan_cmd = shape_command(filter_state.smooth_pan_error, args.pan_gain, args)
    tilt_cmd = shape_command(filter_state.smooth_tilt_error, args.tilt_gain, args)
    pan_cmd = apply_feedforward_term(
        pan_cmd,
        filter_state.smooth_pan_error,
        filter_state.smooth_pan_feedforward_rate,
        args.pan_feedforward_gain,
        args,
    )
    tilt_cmd = apply_feedforward_term(
        tilt_cmd,
        filter_state.smooth_tilt_error,
        filter_state.smooth_tilt_feedforward_rate,
        args.tilt_feedforward_gain,
        args,
    )

    if args.invert_pan:
        pan_cmd *= -1.0
    if args.invert_tilt:
        tilt_cmd *= -1.0

    command_status = "deadzone" if pan_cmd == 0.0 and tilt_cmd == 0.0 else "active"
    return filter_state.smooth_pan_error, filter_state.smooth_tilt_error, pan_cmd, tilt_cmd, command_status


def _safe_int_or_none(value) -> int | None:
    """Return int(value), or None if the field is missing/invalid/non-positive."""
    if value is None:
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def _safe_nonnegative_int_or_none(value) -> int | None:
    """Return int(value), or None if the field is missing/invalid/negative."""
    if value is None:
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed >= 0 else None


# ===== LATENCY STAGE 1/2/3/4 ADDED START =====
def _delta_ms(newer_ns: int | None, older_ns: int | None) -> float | None:
    """Return millisecond difference between two monotonic-ns timestamps."""
    if newer_ns is None or older_ns is None:
        return None
    return (newer_ns - older_ns) / 1e6


def _metadata_is_fresh(metadata_age_ms: float | None, max_age_ms: float) -> bool:
    """Return True when the latest-only snapshot is recent enough for control."""
    if metadata_age_ms is None:
        return True
    if max_age_ms <= 0.0:
        return True
    return metadata_age_ms <= max_age_ms
# ===== LATENCY STAGE 1/2/3/4 ADDED END =====


def default_metadata_ipc_dir() -> Path:
    shm_dir = Path("/dev/shm")
    if shm_dir.is_dir() and os.access(shm_dir, os.W_OK):
        return shm_dir
    return Path(tempfile.gettempdir())


def wait_for_metadata_ipc_file(metadata_ipc_file: Path, proc: subprocess.Popen, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if metadata_ipc_file.exists() and metadata_ipc_file.stat().st_size >= METADATA_IPC_SIZE:
            return
        if proc.poll() is not None:
            raise RuntimeError("Combined DeepStream app exited before metadata IPC became available.")
        time.sleep(0.05)
    raise RuntimeError(f"Timed out waiting for metadata IPC file: {metadata_ipc_file}")


def _decode_c_string(raw: bytes) -> str:
    """Decode a null-terminated C string from a fixed-size binary field."""
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def _decode_metadata_payload(
    frame_idx: int,
    ds_write_mono_ns: int,
    payload_bytes: bytes,
) -> dict:
    (
        c_probe_start_mono_ns,
        c_control_done_mono_ns,
        mux_src_mono_ns,
        pgie_src_mono_ns,
        tracker_src_mono_ns,
        frame_pts_ns,
        frame_ntp_ns,
        video_queue_frame_idx,
        video_queue_src_mono_ns,
        nvosd_frame_idx,
        nvosd_src_mono_ns,
        display_queue_frame_idx,
        display_queue_src_mono_ns,
        rtsp_queue_frame_idx,
        rtsp_queue_src_mono_ns,
        visible_tracks,
        lost_frames,
        has_target,
        target_id,
        class_id,
        confidence,
        dx_norm,
        dy_norm,
        class_name_raw,
        status_raw,
        command_status_raw,
    ) = METADATA_IPC_PAYLOAD.unpack(payload_bytes)

    has_target = bool(has_target)
    class_name = _decode_c_string(class_name_raw)
    status = _decode_c_string(status_raw) or "searching"
    command_status = _decode_c_string(command_status_raw) or status

    return {
        "frame_idx": frame_idx,
        "ds_write_mono_ns": ds_write_mono_ns,
        "c_probe_start_mono_ns": c_probe_start_mono_ns,
        "c_control_done_mono_ns": c_control_done_mono_ns,
        "mux_src_mono_ns": mux_src_mono_ns,
        "pgie_src_mono_ns": pgie_src_mono_ns,
        "tracker_src_mono_ns": tracker_src_mono_ns,
        "frame_pts_ns": frame_pts_ns or None,
        "frame_ntp_ns": frame_ntp_ns or None,
        "video_queue_frame_idx": video_queue_frame_idx,
        "video_queue_src_mono_ns": video_queue_src_mono_ns,
        "nvosd_frame_idx": nvosd_frame_idx,
        "nvosd_src_mono_ns": nvosd_src_mono_ns,
        "display_queue_frame_idx": display_queue_frame_idx,
        "display_queue_src_mono_ns": display_queue_src_mono_ns,
        "rtsp_queue_frame_idx": rtsp_queue_frame_idx,
        "rtsp_queue_src_mono_ns": rtsp_queue_src_mono_ns,
        "status": status,
        "command_status": command_status,
        "visible_tracks": visible_tracks,
        "lost_frames": lost_frames,
        "target_id": target_id if has_target else None,
        "class_id": class_id if has_target else None,
        "class_name": class_name if has_target and class_name else None,
        "confidence": confidence if has_target else None,
        "dx_norm": dx_norm if has_target else None,
        "dy_norm": dy_norm if has_target else None,
        "smooth_dx_norm": None,
        "smooth_dy_norm": None,
        "pan_cmd": None,
        "tilt_cmd": None,
    }


def read_latest_metadata_record(
    metadata_mm: mmap.mmap,
    proc: subprocess.Popen,
    poll_interval: float,
    last_sequence: int,
) -> tuple[int, dict] | None:
    payload_offset = METADATA_IPC_HEADER.size

    while True:
        header1 = metadata_mm[: METADATA_IPC_HEADER.size]
        (
            magic,
            version,
            payload_capacity,
            _reserved0,
            sequence1,
            frame_idx,
            ds_write_mono_ns,
            payload_len,
            _reserved1,
        ) = METADATA_IPC_HEADER.unpack(header1)

        if magic != METADATA_IPC_MAGIC or version != METADATA_IPC_VERSION:
            if proc.poll() is not None:
                return None
            time.sleep(poll_interval)
            continue
        if payload_capacity != METADATA_IPC_PAYLOAD_MAX or payload_len > METADATA_IPC_PAYLOAD_MAX:
            raise RuntimeError("Unexpected metadata IPC layout or payload length.")
        if sequence1 == 0 or (sequence1 & 1) == 1 or sequence1 == last_sequence or payload_len == 0:
            if proc.poll() is not None and sequence1 == last_sequence:
                return None
            time.sleep(poll_interval)
            continue
        if payload_len != METADATA_IPC_PAYLOAD.size:
            raise RuntimeError(
                f"Unexpected metadata IPC payload size: got {payload_len}, expected {METADATA_IPC_PAYLOAD.size}."
            )

        payload_bytes = metadata_mm[payload_offset : payload_offset + payload_len]
        header2 = metadata_mm[: METADATA_IPC_HEADER.size]
        sequence2 = METADATA_IPC_HEADER.unpack(header2)[4]
        if sequence1 != sequence2 or (sequence2 & 1) == 1:
            time.sleep(poll_interval)
            continue

        record = _decode_metadata_payload(frame_idx, ds_write_mono_ns, payload_bytes)
        return sequence2, record


def map_commands_to_rates(pan_cmd: float, tilt_cmd: float, args: argparse.Namespace) -> tuple[float, float]:
    yaw_rate_dps = pan_cmd * args.max_yaw_rate_dps
    if args.mav_invert_pan:
        yaw_rate_dps *= -1.0

    pitch_rate_dps = -tilt_cmd * args.max_pitch_rate_dps
    if args.mav_invert_tilt:
        pitch_rate_dps *= -1.0
    return pitch_rate_dps, yaw_rate_dps


def update_angle_targets(
    current_pitch_deg: float,
    current_yaw_deg: float,
    pitch_rate_dps: float,
    yaw_rate_dps: float,
    dt: float,
    args: argparse.Namespace,
) -> tuple[float, float]:
    next_pitch = current_pitch_deg + pitch_rate_dps * dt
    next_yaw = current_yaw_deg + yaw_rate_dps * dt
    next_pitch = clamp(next_pitch, args.min_pitch_angle_deg, args.max_pitch_angle_deg)
    next_yaw = clamp(next_yaw, -args.max_yaw_angle_deg, args.max_yaw_angle_deg)
    return next_pitch, next_yaw


def emit_state(state: PrototypeV2BridgeState, print_state: bool, state_handle) -> None:
    payload = json.dumps(asdict(state), separators=(",", ":"))
    if print_state:
        print(payload)
    if state_handle is not None:
        state_handle.write(payload + "\n")
        state_handle.flush()


def resolve_temp_sensor_path(preferred_zone: str) -> Path | None:
    thermal_root = Path("/sys/devices/virtual/thermal")
    preferred = (preferred_zone or "").strip()
    fallback_names = [preferred, "tj-thermal", "cpu-thermal", "gpu-thermal"]
    seen: set[str] = set()

    for zone_name in fallback_names:
        if not zone_name or zone_name in seen:
            continue
        seen.add(zone_name)
        for zone_path in thermal_root.glob("thermal_zone*"):
            try:
                if zone_path.joinpath("type").read_text(encoding="utf-8").strip() == zone_name:
                    temp_path = zone_path / "temp"
                    if temp_path.exists():
                        return temp_path
            except OSError:
                continue
    return None


def read_jetson_temp_c(temp_sensor_path: Path | None) -> float | None:
    if temp_sensor_path is None:
        return None
    try:
        raw = temp_sensor_path.read_text(encoding="utf-8").strip()
        value = float(raw)
    except (OSError, ValueError):
        return None
    if value > 1000.0:
        value /= 1000.0
    return value


def build_health_state(
    *,
    state: PrototypeV2BridgeState,
    args: argparse.Namespace,
    deepstream_ok: bool,
    elapsed_s: float,
    processed_rows: int,
    first_frame_idx: int | None,
    temp_sensor_path: Path | None,
) -> PrototypeV2HealthState:
    inference_fps = None
    if elapsed_s > 0.0 and first_frame_idx is not None and state.frame_idx >= first_frame_idx:
        inference_fps = ((state.frame_idx - first_frame_idx) + 1) / elapsed_s

    control_hz = (processed_rows / elapsed_s) if elapsed_s > 0.0 else None

    feedback_ok = False
    if state.feedback_age_ms is not None:
        feedback_ok = state.feedback_age_ms <= 2000.0

    display_ok = (not args.show) or state.display_frame_lag is None or state.display_frame_lag <= 4
    rtsp_ok = (not args.rtsp_enable) or state.rtsp_frame_lag is None or state.rtsp_frame_lag <= 4
    video_branch_ok = display_ok and rtsp_ok

    recording_ok = (not args.raw_record_enable) or deepstream_ok

    camera_ok = deepstream_ok and state.frame_idx >= 0 and state.metadata_is_fresh

    return PrototypeV2HealthState(
        timestamp=state.timestamp,
        camera_ok=camera_ok,
        deepstream_ok=deepstream_ok,
        inference_fps=inference_fps,
        metadata_age_ms=state.metadata_age_ms,
        target_state=state.status,
        target_id=state.target_id,
        control_hz=control_hz,
        mavlink_ok=state.mavlink_connected,
        gimbal_feedback_ok=feedback_ok,
        jetson_temp_c=read_jetson_temp_c(temp_sensor_path),
        video_branch_ok=video_branch_ok,
        recording_ok=recording_ok,
        recording_enabled=args.raw_record_enable,
    )


def format_health_summary(health: PrototypeV2HealthState) -> str:
    target_label = health.target_state.upper()
    if health.target_id is not None:
        target_label = f"{target_label}, ID={health.target_id}"

    yolo_text = "n/a" if health.inference_fps is None else f"{health.inference_fps:.1f} FPS"
    age_text = "n/a" if health.metadata_age_ms is None else f"{health.metadata_age_ms:.1f} ms"
    control_text = "n/a" if health.control_hz is None else f"{health.control_hz:.1f} Hz"
    temp_text = "n/a" if health.jetson_temp_c is None else f"{health.jetson_temp_c:.1f} C"
    recording_text = "OFF"
    if health.recording_enabled:
        recording_text = "OK" if health.recording_ok else "BAD"

    return (
        f"CAMERA: {'OK' if health.camera_ok else 'BAD'} | "
        f"DEEPSTREAM: {'OK' if health.deepstream_ok else 'BAD'} | "
        f"YOLO: {yolo_text} | "
        f"TARGET: {target_label} | "
        f"METADATA AGE: {age_text} | "
        f"CONTROL: {control_text} | "
        f"MAVLINK: {'OK' if health.mavlink_ok else 'BAD'} | "
        f"GIMBAL: {'OK' if health.gimbal_feedback_ok else 'BAD'} | "
        f"TEMP: {temp_text} | "
        f"VIDEO: {'OK' if health.video_branch_ok else 'BAD'} | "
        f"RECORDING: {recording_text}"
    )


def emit_health_state(health: PrototypeV2HealthState, print_health: bool, health_handle) -> None:
    if print_health:
        print(format_health_summary(health), flush=True)
    if health_handle is not None:
        payload = json.dumps(asdict(health), separators=(",", ":"))
        health_handle.write(payload + "\n")
        health_handle.flush()


def prime_live_control(bridge: PX4SiyiBridge, args: argparse.Namespace) -> None:
    """Send a neutral live-control setpoint once after configure."""
    if args.live_control_mode == "angle-target":
        bridge.send_pitchyaw(
            pitch_angle_deg=0.0,
            yaw_angle_deg=0.0,
            force=True,
            wait_ack=False,
        )
    else:
        bridge.send_rates(0.0, 0.0, force=True)
    time.sleep(0.05)


def send_live_command(
    bridge: PX4SiyiBridge,
    args: argparse.Namespace,
    pitch_rate_dps: float,
    yaw_rate_dps: float,
    target_pitch_deg: float,
    target_yaw_deg: float,
) -> None:
    """Send the live setpoint using the same stronger forced-send style as the working bench test."""
    if args.live_control_mode == "angle-target":
        bridge.send_pitchyaw(
            pitch_angle_deg=target_pitch_deg,
            yaw_angle_deg=target_yaw_deg,
            force=True,
            wait_ack=False,
        )
    else:
        bridge.send_rates(pitch_rate_dps, yaw_rate_dps, force=True)


def start_combined_app(
    args: argparse.Namespace,
    metadata_ipc_file: Path,
    metadata_state_file: Path | None,
) -> tuple[subprocess.Popen, object | None]:
    env = os.environ.copy()
    env["METADATA_IPC_FILE"] = str(metadata_ipc_file)
    env["STATE_FILE"] = str(metadata_state_file) if metadata_state_file is not None else ""

    cmd = ["bash", args.combined_launcher]
    if args.show_deepstream_log:
        return subprocess.Popen(cmd, cwd=str(SCRIPT_DIR.parent.parent), env=env), None

    log_path = Path(args.deepstream_log_file) if args.deepstream_log_file else Path(
        tempfile.mkstemp(prefix="prototype_v2_px4_bridge_", suffix=".log")[1]
    )
    log_handle = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        cwd=str(SCRIPT_DIR.parent.parent),
        env=env,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
    )
    return proc, log_handle


def main() -> int:
    args = parse_args()

    metadata_state_file = Path(args.metadata_state_file) if args.metadata_state_file else None
    if metadata_state_file is not None and metadata_state_file.exists():
        metadata_state_file.unlink()
    if metadata_state_file is not None:
        metadata_state_file.parent.mkdir(parents=True, exist_ok=True)

    if args.metadata_ipc_file:
        metadata_ipc_file = Path(args.metadata_ipc_file)
        metadata_ipc_file.parent.mkdir(parents=True, exist_ok=True)
        if metadata_ipc_file.exists():
            metadata_ipc_file.unlink()
    else:
        metadata_ipc_dir = default_metadata_ipc_dir()
        metadata_ipc_dir.mkdir(parents=True, exist_ok=True)
        ipc_fd, ipc_path = tempfile.mkstemp(prefix="prototype_v2_metadata_", suffix=".mmap", dir=str(metadata_ipc_dir))
        os.close(ipc_fd)
        metadata_ipc_file = Path(ipc_path)

    bridge_state_handle = None
    if args.bridge_state_file:
        bridge_state_path = Path(args.bridge_state_file)
        bridge_state_path.parent.mkdir(parents=True, exist_ok=True)
        bridge_state_handle = bridge_state_path.open("w", encoding="utf-8")

    health_state_handle = None
    if args.health_state_file:
        health_state_path = Path(args.health_state_file)
        health_state_path.parent.mkdir(parents=True, exist_ok=True)
        health_state_handle = health_state_path.open("w", encoding="utf-8")

    proc, log_handle = start_combined_app(args, metadata_ipc_file, metadata_state_file)
    bridge = PX4SiyiBridge(args)
    target_pitch_deg = 0.0
    target_yaw_deg = 0.0
    previous_time = time.perf_counter()
    filter_state = LiveControlFilterState()
    metadata_fd: int | None = None
    metadata_mm: mmap.mmap | None = None
    metadata_sequence = 0
    previous_frame_idx: int | None = None
    first_frame_idx: int | None = None
    processed_rows = 0
    active_start_time: float | None = None
    last_health_emit_time = 0.0
    temp_sensor_path = resolve_temp_sensor_path(args.health_temp_zone)

    try:
        wait_for_metadata_ipc_file(metadata_ipc_file, proc, args.startup_timeout)
        bridge.connect()
        prime_live_control(bridge, args)

        metadata_fd = os.open(metadata_ipc_file, os.O_RDONLY)
        metadata_mm = mmap.mmap(metadata_fd, METADATA_IPC_SIZE, access=mmap.ACCESS_READ)
        while True:
            latest_record = read_latest_metadata_record(
                metadata_mm,
                proc,
                args.poll_interval,
                metadata_sequence,
            )
            if latest_record is None:
                break
            previous_metadata_sequence = metadata_sequence
            metadata_sequence, record = latest_record
            metadata_gap_frames = 0
            if previous_metadata_sequence > 0 and metadata_sequence > previous_metadata_sequence:
                metadata_gap_frames = max(0, ((metadata_sequence - previous_metadata_sequence) // 2) - 1)

            # ===== LATENCY STAGE 1/2/3 ADDED START =====
            # Stage 1 T2: Python has received/read one metadata snapshot.
            py_read_mono_ns = time.monotonic_ns()

            # Stage 1 T1 from C metadata snapshot.
            frame_pts_ns = _safe_int_or_none(record.get("frame_pts_ns"))
            frame_ntp_ns = _safe_int_or_none(record.get("frame_ntp_ns"))
            ds_write_mono_ns = _safe_int_or_none(record.get("ds_write_mono_ns"))

            # Stage 2 timestamps from C tracker-probe/control timing.
            c_probe_start_mono_ns = _safe_int_or_none(record.get("c_probe_start_mono_ns"))
            c_control_done_mono_ns = _safe_int_or_none(record.get("c_control_done_mono_ns"))

            # Stage 3 timestamps from coarse DeepStream element pad probes.
            mux_src_mono_ns = _safe_int_or_none(record.get("mux_src_mono_ns"))
            pgie_src_mono_ns = _safe_int_or_none(record.get("pgie_src_mono_ns"))
            tracker_src_mono_ns = _safe_int_or_none(record.get("tracker_src_mono_ns"))
            video_queue_frame_idx = _safe_nonnegative_int_or_none(record.get("video_queue_frame_idx"))
            video_queue_src_mono_ns = _safe_int_or_none(record.get("video_queue_src_mono_ns"))
            nvosd_frame_idx = _safe_nonnegative_int_or_none(record.get("nvosd_frame_idx"))
            nvosd_src_mono_ns = _safe_int_or_none(record.get("nvosd_src_mono_ns"))
            display_queue_frame_idx = _safe_nonnegative_int_or_none(record.get("display_queue_frame_idx"))
            display_queue_src_mono_ns = _safe_int_or_none(record.get("display_queue_src_mono_ns"))
            rtsp_queue_frame_idx = _safe_nonnegative_int_or_none(record.get("rtsp_queue_frame_idx"))
            rtsp_queue_src_mono_ns = _safe_int_or_none(record.get("rtsp_queue_src_mono_ns"))
            # ===== LATENCY STAGE 1/2/3 ADDED END =====

            current_frame_idx = int(record.get("frame_idx", -1))
            if first_frame_idx is None and current_frame_idx >= 0:
                first_frame_idx = current_frame_idx
                active_start_time = py_read_mono_ns / 1e9
            raw_frame_gap = (
                max(0, current_frame_idx - previous_frame_idx - 1)
                if previous_frame_idx is not None and previous_frame_idx >= 0 and current_frame_idx >= 0
                else 0
            )
            video_frame_gap = max(0, raw_frame_gap - metadata_gap_frames)
            previous_frame_idx = current_frame_idx

            status = str(record.get("status", "searching"))
            visible_tracks = int(record.get("visible_tracks", 0) or 0)
            lost_frames = int(record.get("lost_frames", 0) or 0)
            target_id = record.get("target_id")
            metadata_age_ms = _delta_ms(py_read_mono_ns, ds_write_mono_ns)
            metadata_is_fresh = _metadata_is_fresh(metadata_age_ms, args.metadata_max_age_ms)
            has_target = metadata_is_fresh and target_id is not None and status not in {"searching", "lost"}
            raw_dx_norm = float(record.get("dx_norm", 0.0) or 0.0) if has_target else 0.0
            raw_dy_norm = float(record.get("dy_norm", 0.0) or 0.0) if has_target else 0.0

            current_time = time.perf_counter()
            delta = min(max(current_time - previous_time, 1e-6), 0.1)
            previous_time = current_time

            smooth_dx_norm, smooth_dy_norm, pan_cmd, tilt_cmd, command_status = compute_control_outputs(
                has_target=has_target,
                dx_norm=raw_dx_norm,
                dy_norm=raw_dy_norm,
                dt=delta,
                args=args,
                filter_state=filter_state,
            )
            if not metadata_is_fresh:
                command_status = "stale"
            if not has_target and status in {"searching", "lost"}:
                command_status = status

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
            # ===== LATENCY STAGE 1/2/3 ADDED START =====
            # Stage 1 T3: immediately before this bridge sends the MAVLink command.
            mav_send_mono_ns = time.monotonic_ns()

            # Stage 1 latency numbers.
            vision_latency_ms = _delta_ms(ds_write_mono_ns, mux_src_mono_ns)
            ds_to_py_ms = metadata_age_ms
            control_age_ms = metadata_age_ms
            py_to_mav_ms = _delta_ms(mav_send_mono_ns, py_read_mono_ns) or 0.0
            mavlink_delay_ms = py_to_mav_ms
            ds_to_mav_ms = _delta_ms(mav_send_mono_ns, ds_write_mono_ns)

            # Stage 2 latency numbers.
            c_probe_to_control_ms = _delta_ms(c_control_done_mono_ns, c_probe_start_mono_ns)
            c_control_to_json_ms = _delta_ms(ds_write_mono_ns, c_control_done_mono_ns)
            c_probe_to_json_ms = _delta_ms(ds_write_mono_ns, c_probe_start_mono_ns)

            # Stage 3 latency numbers.
            mux_to_pgie_ms = _delta_ms(pgie_src_mono_ns, mux_src_mono_ns)
            pgie_to_tracker_ms = _delta_ms(tracker_src_mono_ns, pgie_src_mono_ns)
            mux_to_tracker_ms = _delta_ms(tracker_src_mono_ns, mux_src_mono_ns)
            tracker_to_video_queue_ms = _delta_ms(tracker_src_mono_ns, video_queue_src_mono_ns)
            video_queue_to_osd_ms = _delta_ms(nvosd_src_mono_ns, video_queue_src_mono_ns)
            tracker_to_osd_ms = _delta_ms(tracker_src_mono_ns, nvosd_src_mono_ns)
            osd_to_display_queue_ms = _delta_ms(display_queue_src_mono_ns, nvosd_src_mono_ns)
            osd_to_rtsp_queue_ms = _delta_ms(rtsp_queue_src_mono_ns, nvosd_src_mono_ns)
            display_frame_lag = (
                current_frame_idx - display_queue_frame_idx
                if display_queue_frame_idx is not None and current_frame_idx >= 0
                else None
            )
            rtsp_frame_lag = (
                current_frame_idx - rtsp_queue_frame_idx
                if rtsp_queue_frame_idx is not None and current_frame_idx >= 0
                else None
            )
            # ===== LATENCY STAGE 1/2/3 ADDED END =====

            # ===== LATENCY STAGE 4 ADDED START =====
            # Stage 4: measure how long the Python MAVLink send wrapper takes to return.
            send_live_command(
                bridge=bridge,
                args=args,
                pitch_rate_dps=pitch_rate_dps,
                yaw_rate_dps=yaw_rate_dps,
                target_pitch_deg=target_pitch_deg,
                target_yaw_deg=target_yaw_deg,
            )
            mav_return_mono_ns = time.monotonic_ns()
            mav_call_ms = (mav_return_mono_ns - mav_send_mono_ns) / 1e6
            # ===== LATENCY STAGE 4 ADDED END =====
            bridge.poll_feedback()
            feedback_mono_ns = bridge.last_feedback_mono_ns
            feedback_wall_time = bridge.last_feedback_wall_time
            feedback_age_ms = _delta_ms(mav_return_mono_ns, feedback_mono_ns)
            mav_send_to_feedback_ms = (
                _delta_ms(feedback_mono_ns, mav_send_mono_ns)
                if feedback_mono_ns is not None and feedback_mono_ns >= mav_send_mono_ns
                else None
            )
            feedback_delay_ms = mav_send_to_feedback_ms

            state = PrototypeV2BridgeState(
                frame_idx=current_frame_idx,
                timestamp=time.time(),
                # ===== LATENCY STAGE 1 ADDED START =====
                metadata_sequence=metadata_sequence,
                metadata_gap_frames=metadata_gap_frames,
                video_frame_gap=video_frame_gap,
                frame_pts_ns=frame_pts_ns,
                frame_ntp_ns=frame_ntp_ns,
                ds_write_mono_ns=ds_write_mono_ns,
                py_read_mono_ns=py_read_mono_ns,
                mav_send_mono_ns=mav_send_mono_ns,
                vision_latency_ms=vision_latency_ms,
                metadata_age_ms=metadata_age_ms,
                control_age_ms=control_age_ms,
                metadata_is_fresh=metadata_is_fresh,
                ds_to_py_ms=ds_to_py_ms,
                py_to_mav_ms=py_to_mav_ms,
                mavlink_delay_ms=mavlink_delay_ms,
                ds_to_mav_ms=ds_to_mav_ms,
                # ===== LATENCY STAGE 1 ADDED END =====

                # ===== LATENCY STAGE 2 ADDED START =====
                c_probe_start_mono_ns=c_probe_start_mono_ns,
                c_control_done_mono_ns=c_control_done_mono_ns,
                c_probe_to_control_ms=c_probe_to_control_ms,
                c_control_to_json_ms=c_control_to_json_ms,
                c_probe_to_json_ms=c_probe_to_json_ms,
                # ===== LATENCY STAGE 2 ADDED END =====

                # ===== LATENCY STAGE 3 ADDED START =====
                mux_src_mono_ns=mux_src_mono_ns,
                pgie_src_mono_ns=pgie_src_mono_ns,
                tracker_src_mono_ns=tracker_src_mono_ns,
                video_queue_frame_idx=video_queue_frame_idx,
                video_queue_src_mono_ns=video_queue_src_mono_ns,
                nvosd_frame_idx=nvosd_frame_idx,
                nvosd_src_mono_ns=nvosd_src_mono_ns,
                display_queue_frame_idx=display_queue_frame_idx,
                display_queue_src_mono_ns=display_queue_src_mono_ns,
                rtsp_queue_frame_idx=rtsp_queue_frame_idx,
                rtsp_queue_src_mono_ns=rtsp_queue_src_mono_ns,
                mux_to_pgie_ms=mux_to_pgie_ms,
                pgie_to_tracker_ms=pgie_to_tracker_ms,
                mux_to_tracker_ms=mux_to_tracker_ms,
                tracker_to_video_queue_ms=tracker_to_video_queue_ms,
                video_queue_to_osd_ms=video_queue_to_osd_ms,
                tracker_to_osd_ms=tracker_to_osd_ms,
                osd_to_display_queue_ms=osd_to_display_queue_ms,
                osd_to_rtsp_queue_ms=osd_to_rtsp_queue_ms,
                display_frame_lag=display_frame_lag,
                rtsp_frame_lag=rtsp_frame_lag,
                # ===== LATENCY STAGE 3 ADDED END =====

                # ===== LATENCY STAGE 4 ADDED START =====
                mav_return_mono_ns=mav_return_mono_ns,
                mav_call_ms=mav_call_ms,
                # ===== LATENCY STAGE 4 ADDED END =====
                # ===== FEEDBACK TIMING ADDED START =====
                feedback_type=bridge.last_feedback_type,
                feedback_mono_ns=feedback_mono_ns,
                feedback_wall_time=feedback_wall_time,
                feedback_age_ms=feedback_age_ms,
                mav_send_to_feedback_ms=mav_send_to_feedback_ms,
                feedback_delay_ms=feedback_delay_ms,
                manager_status_mono_ns=bridge.last_manager_status_mono_ns,
                device_attitude_status_mono_ns=bridge.last_device_attitude_status_mono_ns,
                # ===== FEEDBACK TIMING ADDED END =====
                status=status,
                command_status=command_status,
                visible_tracks=visible_tracks,
                lost_frames=lost_frames,
                target_id=target_id,
                class_id=record.get("class_id"),
                class_name=record.get("class_name"),
                confidence=record.get("confidence"),
                dx_norm=record.get("dx_norm"),
                dy_norm=record.get("dy_norm"),
                smooth_dx_norm=smooth_dx_norm,
                smooth_dy_norm=smooth_dy_norm,
                pan_cmd=pan_cmd,
                tilt_cmd=tilt_cmd,
                control_mode=args.live_control_mode,
                yaw_rate_dps=yaw_rate_dps,
                pitch_rate_dps=pitch_rate_dps,
                yaw_target_deg=target_yaw_deg,
                pitch_target_deg=target_pitch_deg,
                yaw_flag=bridge.yaw_flag,
                mavlink_connected=bridge.connected,
                mavlink_link=bridge.link_state(),
                px4_target_system=args.mav_target_system,
                px4_target_component=args.mav_target_component,
                gimbal_device_id=args.gimbal_device_id,
                last_configure_ack=bridge.last_configure_ack,
                last_pitchyaw_send=bridge.last_pitchyaw_send,
                metadata_state_file=str(metadata_state_file) if metadata_state_file is not None else "",
                metadata_ipc_file=str(metadata_ipc_file),
            )
            emit_state(state, args.print_state, bridge_state_handle)
            processed_rows += 1

            now_wall = state.timestamp
            interval = max(0.1, args.health_print_interval_sec)
            if args.print_health or health_state_handle is not None:
                if last_health_emit_time == 0.0 or (now_wall - last_health_emit_time) >= interval:
                    elapsed_s = 0.0
                    if active_start_time is not None:
                        elapsed_s = max(0.0, (py_read_mono_ns / 1e9) - active_start_time)
                    health = build_health_state(
                        state=state,
                        args=args,
                        deepstream_ok=(proc.poll() is None),
                        elapsed_s=elapsed_s,
                        processed_rows=processed_rows,
                        first_frame_idx=first_frame_idx,
                        temp_sensor_path=temp_sensor_path,
                    )
                    emit_health_state(health, args.print_health, health_state_handle)
                    last_health_emit_time = now_wall
    finally:
        try:
            bridge.close()
        except Exception:
            pass
        if metadata_mm is not None:
            metadata_mm.close()
        if metadata_fd is not None:
            os.close(metadata_fd)
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.0)
        if log_handle is not None:
            log_handle.close()
        if bridge_state_handle is not None:
            bridge_state_handle.close()
        if health_state_handle is not None:
            health_state_handle.close()
        try:
            if metadata_ipc_file.exists():
                metadata_ipc_file.unlink()
        except FileNotFoundError:
            pass

    return proc.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
