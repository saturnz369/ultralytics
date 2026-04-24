#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
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


@dataclass
class PrototypeV2BridgeState:
    frame_idx: int
    timestamp: float
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run prototype_v2 DeepStream RTSP + metadata-control preview and bridge it to PX4 -> SIYI."
    )
    parser.add_argument("--metadata-state-file", default=None, help="JSONL file written by the combined DeepStream app")
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
    parser.add_argument("--poll-interval", type=float, default=0.02, help="Tail polling interval in seconds")

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
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


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


def start_combined_app(args: argparse.Namespace, metadata_state_file: Path) -> tuple[subprocess.Popen, object | None]:
    env = os.environ.copy()
    env["STATE_FILE"] = str(metadata_state_file)

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


def wait_for_metadata_file(metadata_state_file: Path, proc: subprocess.Popen, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if metadata_state_file.exists():
            return
        if proc.poll() is not None:
            raise RuntimeError("Combined DeepStream app exited before metadata file was created.")
        time.sleep(0.1)
    raise RuntimeError(f"Timed out waiting for metadata file: {metadata_state_file}")


def main() -> int:
    args = parse_args()

    metadata_state_file = Path(args.metadata_state_file) if args.metadata_state_file else Path(
        tempfile.mkstemp(prefix="prototype_v2_metadata_", suffix=".jsonl")[1]
    )
    if metadata_state_file.exists():
        metadata_state_file.unlink()
    metadata_state_file.parent.mkdir(parents=True, exist_ok=True)

    bridge_state_handle = None
    if args.bridge_state_file:
        bridge_state_path = Path(args.bridge_state_file)
        bridge_state_path.parent.mkdir(parents=True, exist_ok=True)
        bridge_state_handle = bridge_state_path.open("w", encoding="utf-8")

    proc, log_handle = start_combined_app(args, metadata_state_file)
    bridge = PX4SiyiBridge(args)
    target_pitch_deg = 0.0
    target_yaw_deg = 0.0
    previous_time = time.perf_counter()

    try:
        wait_for_metadata_file(metadata_state_file, proc, args.startup_timeout)
        bridge.connect()
        prime_live_control(bridge, args)

        with metadata_state_file.open("r", encoding="utf-8") as metadata_fp:
            while True:
                line = metadata_fp.readline()
                if not line:
                    if proc.poll() is not None:
                        break
                    time.sleep(args.poll_interval)
                    continue

                line = line.strip()
                if not line:
                    continue

                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue

                status = str(record.get("status", "searching"))
                command_status = str(record.get("command_status", "no_target"))
                visible_tracks = int(record.get("visible_tracks", 0) or 0)
                lost_frames = int(record.get("lost_frames", 0) or 0)
                pan_cmd = float(record.get("pan_cmd", 0.0) or 0.0)
                tilt_cmd = float(record.get("tilt_cmd", 0.0) or 0.0)
                if status in {"searching", "lost"}:
                    pan_cmd = 0.0
                    tilt_cmd = 0.0
                    command_status = status

                current_time = time.perf_counter()
                delta = min(max(current_time - previous_time, 1e-6), 0.1)
                previous_time = current_time

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
                send_live_command(
                    bridge=bridge,
                    args=args,
                    pitch_rate_dps=pitch_rate_dps,
                    yaw_rate_dps=yaw_rate_dps,
                    target_pitch_deg=target_pitch_deg,
                    target_yaw_deg=target_yaw_deg,
                )

                state = PrototypeV2BridgeState(
                    frame_idx=int(record.get("frame_idx", -1)),
                    timestamp=time.time(),
                    status=status,
                    command_status=command_status,
                    visible_tracks=visible_tracks,
                    lost_frames=lost_frames,
                    target_id=record.get("target_id"),
                    class_id=record.get("class_id"),
                    class_name=record.get("class_name"),
                    confidence=record.get("confidence"),
                    dx_norm=record.get("dx_norm"),
                    dy_norm=record.get("dy_norm"),
                    smooth_dx_norm=float(record.get("smooth_dx_norm", 0.0) or 0.0),
                    smooth_dy_norm=float(record.get("smooth_dy_norm", 0.0) or 0.0),
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
                    metadata_state_file=str(metadata_state_file),
                )
                emit_state(state, args.print_state, bridge_state_handle)
    finally:
        try:
            bridge.close()
        except Exception:
            pass
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

    return proc.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
