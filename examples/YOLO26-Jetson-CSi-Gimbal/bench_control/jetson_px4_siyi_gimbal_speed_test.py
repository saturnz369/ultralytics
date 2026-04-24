#!/usr/bin/env python3
# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = Path(__file__).resolve().parent
for path in (REPO_ROOT, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from pymavlink import mavutil

from px4_siyi_bench_bridge import PX4SiyiBenchBridge


FEEDBACK_MESSAGE_IDS = (158, 200)
FEEDBACK_TYPES = ("MOUNT_STATUS", "GIMBAL_REPORT", "GIMBAL_MANAGER_STATUS")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for a simple gimbal step-speed test."""
    parser = argparse.ArgumentParser(
        description="Send simple fixed gimbal angle step commands from Jetson -> PX4 -> SIYI and log movement feedback."
    )
    parser.add_argument("--start-pitch", type=float, default=0.0, help="Start pitch angle in degrees")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="Start yaw angle in degrees")
    parser.add_argument("--target-pitch", type=float, default=0.0, help="Target pitch angle in degrees")
    parser.add_argument("--target-yaw", type=float, default=0.0, help="Target yaw angle in degrees")
    parser.add_argument(
        "--second-target-pitch",
        type=float,
        default=math.nan,
        help="Optional second target pitch angle in degrees",
    )
    parser.add_argument(
        "--second-target-yaw",
        type=float,
        default=math.nan,
        help="Optional second target yaw angle in degrees",
    )
    parser.add_argument(
        "--third-target-pitch",
        type=float,
        default=math.nan,
        help="Optional third target pitch angle in degrees",
    )
    parser.add_argument(
        "--third-target-yaw",
        type=float,
        default=math.nan,
        help="Optional third target yaw angle in degrees",
    )
    parser.add_argument(
        "--start-hold-seconds",
        type=float,
        default=1.5,
        help="Seconds to hold the start pose before sending the target step",
    )
    parser.add_argument(
        "--target-hold-seconds",
        type=float,
        default=4.0,
        help="Seconds to hold the target pose while collecting feedback",
    )
    parser.add_argument(
        "--second-target-hold-seconds",
        type=float,
        default=0.0,
        help="Seconds to hold the optional second target pose while collecting feedback",
    )
    parser.add_argument(
        "--third-target-hold-seconds",
        type=float,
        default=0.0,
        help="Seconds to hold the optional third target pose while collecting feedback",
    )
    parser.add_argument(
        "--move-transit-seconds",
        type=float,
        default=0.0,
        help="Minimum travel window after each move command before sending the next target",
    )
    parser.add_argument(
        "--start-transit-seconds",
        type=float,
        default=math.nan,
        help="Optional travel window override for the start move",
    )
    parser.add_argument(
        "--target-transit-seconds",
        type=float,
        default=math.nan,
        help="Optional travel window override for the first target move",
    )
    parser.add_argument(
        "--second-target-transit-seconds",
        type=float,
        default=math.nan,
        help="Optional travel window override for the second target move",
    )
    parser.add_argument(
        "--third-target-transit-seconds",
        type=float,
        default=math.nan,
        help="Optional travel window override for the third target move",
    )
    parser.add_argument(
        "--return-transit-seconds",
        type=float,
        default=math.nan,
        help="Optional travel window override for the return move",
    )
    parser.add_argument(
        "--move-pitch-rate-dps",
        type=float,
        default=math.nan,
        help="Optional pitch rate hint in deg/s for each angle-target move",
    )
    parser.add_argument(
        "--move-yaw-rate-dps",
        type=float,
        default=math.nan,
        help="Optional yaw rate hint in deg/s for each angle-target move",
    )
    parser.add_argument(
        "--resend-hz",
        type=float,
        default=20.0,
        help="Re-send the same pose at this rate while holding",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=20.0,
        help="Loop rate for feedback collection and re-send scheduling",
    )
    parser.add_argument(
        "--feedback-request-hz",
        type=float,
        default=10.0,
        help="How often to request feedback messages during the hold windows",
    )
    parser.add_argument(
        "--settle-threshold-deg",
        type=float,
        default=3.0,
        help="Consider the gimbal settled when within this many degrees of the target",
    )
    parser.add_argument(
        "--state-file",
        default="/tmp/gimbal_speed_step_test.jsonl",
        help="JSONL log file for command and feedback events",
    )
    parser.add_argument(
        "--return-after",
        action="store_true",
        help="After the target hold window, return to the start pose",
    )
    parser.add_argument(
        "--return-hold-seconds",
        type=float,
        default=1.5,
        help="Seconds to hold the return-to-start pose if --return-after is used",
    )

    parser.add_argument("--serial-device", default="/dev/ttyUSB0", help="Jetson -> PX4 MAVLink serial device")
    parser.add_argument("--serial-baud", type=int, default=921600, help="Jetson -> PX4 MAVLink baud rate")
    parser.add_argument("--mav-source-system", type=int, default=42, help="MAVLink source system ID for the Jetson")
    parser.add_argument(
        "--mav-source-component",
        type=int,
        default=191,
        help="MAVLink source component ID for the Jetson",
    )
    parser.add_argument("--mav-target-system", type=int, default=1, help="PX4 target system ID")
    parser.add_argument(
        "--mav-target-component",
        type=int,
        default=154,
        help="PX4 gimbal manager component ID",
    )
    parser.add_argument("--gimbal-device-id", type=int, default=154, help="SIYI gimbal device component ID")
    parser.add_argument("--heartbeat-timeout", type=float, default=5.0, help="Seconds to wait for PX4 heartbeat")
    parser.add_argument("--ack-timeout", type=float, default=2.0, help="Seconds to wait for COMMAND_ACK")
    parser.add_argument(
        "--send-rate-hz",
        type=float,
        default=20.0,
        help="Internal lower-bound on command transmission rate for re-send loops",
    )
    parser.add_argument(
        "--control-api",
        default="attitude",
        choices=("attitude", "manual", "pitchyaw", "command", "message"),
        help="Send pitch/yaw using GIMBAL_MANAGER_SET_ATTITUDE, SET_MANUAL_CONTROL, SET_PITCHYAW, or MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW",
    )
    parser.add_argument("--yaw-lock", action="store_true", help="Use yaw-lock mode instead of vehicle-frame yaw")
    parser.add_argument("--pitch-lock", action="store_true", help="Set the pitch-lock flag in manager commands")
    parser.add_argument("--skip-configure", action="store_true", help="Skip gimbal manager configure step")
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
    parser.add_argument("--dry-run-mavlink", action="store_true", help="Print commands but do not send MAVLink")
    return parser.parse_args()


def angle_delta_deg(a_deg: float, b_deg: float) -> float:
    """Return the wrapped shortest angle delta a-b in degrees."""
    return ((a_deg - b_deg + 180.0) % 360.0) - 180.0


def write_jsonl(log_path: Path, payload: dict) -> None:
    """Append one JSON object to the JSONL log."""
    with log_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, sort_keys=True) + "\n")


def send_angle_command(
    bridge: PX4SiyiBenchBridge,
    pitch_deg: float,
    yaw_deg: float,
    force: bool,
    wait_ack: bool,
    pitch_rate_dps: float = math.nan,
    yaw_rate_dps: float = math.nan,
) -> str | None:
    """Send one angle command to PX4's gimbal manager."""
    return bridge.send_pitchyaw(
        pitch_angle_deg=pitch_deg,
        yaw_angle_deg=yaw_deg,
        pitch_rate_dps=pitch_rate_dps,
        yaw_rate_dps=yaw_rate_dps,
        force=force,
        wait_ack=wait_ack,
    )


def extract_feedback_row(msg, phase: str) -> dict:
    """Convert one feedback message into a JSON-serializable row."""
    row = {
        "event": "feedback",
        "phase": phase,
        "timestamp": time.time(),
        "msg_type": msg.get_type(),
    }
    for key, value in msg.to_dict().items():
        if isinstance(value, (int, float, str, bool)) or value is None:
            row[key] = value

    if row["msg_type"] == "MOUNT_STATUS":
        row["feedback_source"] = "MOUNT_STATUS"
        row["feedback_pitch_deg"] = float(row.get("pointing_a", 0.0)) / 100.0
        row["feedback_roll_deg"] = float(row.get("pointing_b", 0.0)) / 100.0
        row["feedback_yaw_deg"] = float(row.get("pointing_c", 0.0)) / 100.0
    elif row["msg_type"] == "GIMBAL_REPORT":
        row["feedback_source"] = "GIMBAL_REPORT"
        row["feedback_roll_deg"] = math.degrees(float(row.get("joint_roll", 0.0)))
        row["feedback_pitch_deg"] = math.degrees(float(row.get("joint_el", 0.0)))
        row["feedback_yaw_deg"] = math.degrees(float(row.get("joint_az", 0.0)))
    return row


def drain_feedback(bridge: PX4SiyiBenchBridge, phase: str, log_path: Path, rows: list[dict]) -> None:
    """Drain all currently pending feedback messages and log them."""
    while True:
        msg = bridge.recv_feedback(FEEDBACK_TYPES, timeout=0.0)
        if msg is None:
            break
        row = extract_feedback_row(msg, phase)
        rows.append(row)
        write_jsonl(log_path, row)


def hold_pose_and_collect(
    bridge: PX4SiyiBenchBridge,
    pitch_deg: float,
    yaw_deg: float,
    hold_seconds: float,
    phase: str,
    args: argparse.Namespace,
    log_path: Path,
    rows: list[dict],
) -> None:
    """Hold one pose, re-send it, and collect any available feedback."""
    if hold_seconds <= 0:
        return

    resend_interval = 0.0 if args.resend_hz <= 0 else 1.0 / args.resend_hz
    sample_interval = 0.05 if args.sample_hz <= 0 else 1.0 / args.sample_hz
    request_interval = 0.1 if args.feedback_request_hz <= 0 else 1.0 / args.feedback_request_hz

    deadline = time.monotonic() + hold_seconds
    next_resend = time.monotonic() + resend_interval if resend_interval > 0 else math.inf
    next_request = time.monotonic()

    while True:
        now = time.monotonic()
        if now >= deadline:
            break

        if resend_interval > 0 and now >= next_resend:
            send_angle_command(
                bridge,
                pitch_deg,
                yaw_deg,
                force=True,
                wait_ack=False,
                pitch_rate_dps=args.move_pitch_rate_dps,
                yaw_rate_dps=args.move_yaw_rate_dps,
            )
            next_resend += resend_interval

        if now >= next_request:
            for message_id in FEEDBACK_MESSAGE_IDS:
                bridge.request_message(message_id)
            next_request += request_interval

        msg = bridge.recv_feedback(FEEDBACK_TYPES, timeout=min(sample_interval, max(deadline - now, 0.0)))
        if msg is not None:
            row = extract_feedback_row(msg, phase)
            rows.append(row)
            write_jsonl(log_path, row)
            drain_feedback(bridge, phase, log_path, rows)


def collect_transit_window(
    bridge: PX4SiyiBenchBridge,
    pitch_deg: float,
    yaw_deg: float,
    hold_seconds: float,
    transit_seconds: float,
    phase: str,
    args: argparse.Namespace,
    log_path: Path,
    rows: list[dict],
) -> None:
    """Collect feedback for either the explicit hold or the configured transit window."""
    fallback_transit = args.move_transit_seconds if math.isnan(transit_seconds) else transit_seconds
    effective_seconds = max(hold_seconds, fallback_transit)
    if effective_seconds <= 0:
        return
    hold_pose_and_collect(
        bridge=bridge,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
        hold_seconds=effective_seconds,
        phase=phase,
        args=args,
        log_path=log_path,
        rows=rows,
    )


def choose_feedback_source(rows: list[dict], phase: str) -> str | None:
    """Choose the best available feedback source for summary statistics."""
    counts = {}
    for row in rows:
        if row.get("event") != "feedback" or row.get("phase") != phase:
            continue
        source = row.get("feedback_source")
        if source and "feedback_yaw_deg" in row:
            counts[source] = counts.get(source, 0) + 1
    if not counts:
        return None
    if "MOUNT_STATUS" in counts:
        return "MOUNT_STATUS"
    return max(counts, key=counts.get)


def summarize_axis_motion(
    rows: list[dict],
    phase: str,
    source: str,
    axis_key: str,
    target_deg: float,
    settle_threshold_deg: float,
) -> dict | None:
    """Compute a simple speed summary for one axis from feedback rows."""
    samples = [
        (float(row["timestamp"]), float(row[axis_key]))
        for row in rows
        if row.get("event") == "feedback"
        and row.get("phase") == phase
        and row.get("feedback_source") == source
        and axis_key in row
    ]
    if len(samples) < 2:
        return None

    peak_speed_dps = 0.0
    for (t0, a0), (t1, a1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        peak_speed_dps = max(peak_speed_dps, abs(angle_delta_deg(a1, a0)) / dt)

    start_t, start_a = samples[0]
    end_t, end_a = samples[-1]
    elapsed = max(end_t - start_t, 1e-6)
    avg_speed_dps = abs(angle_delta_deg(end_a, start_a)) / elapsed

    time_to_settle_sec = None
    for sample_t, sample_a in samples:
        if abs(angle_delta_deg(target_deg, sample_a)) <= settle_threshold_deg:
            time_to_settle_sec = sample_t - start_t
            break

    return {
        "feedback_source": source,
        "sample_count": len(samples),
        "start_deg": round(start_a, 3),
        "end_deg": round(end_a, 3),
        "target_deg": round(target_deg, 3),
        "peak_speed_dps": round(peak_speed_dps, 3),
        "avg_speed_dps": round(avg_speed_dps, 3),
        "time_to_settle_sec": None if time_to_settle_sec is None else round(time_to_settle_sec, 3),
    }


def print_summary(rows: list[dict], args: argparse.Namespace, log_path: Path) -> None:
    """Print a useful bench summary if we received usable feedback."""
    source = choose_feedback_source(rows, phase="target_hold")
    if source is None:
        print("No usable gimbal angle feedback was captured during the target hold.")
        print(f"Raw log saved to: {log_path}")
        return

    yaw_summary = summarize_axis_motion(
        rows,
        phase="target_hold",
        source=source,
        axis_key="feedback_yaw_deg",
        target_deg=args.target_yaw,
        settle_threshold_deg=args.settle_threshold_deg,
    )
    pitch_summary = summarize_axis_motion(
        rows,
        phase="target_hold",
        source=source,
        axis_key="feedback_pitch_deg",
        target_deg=args.target_pitch,
        settle_threshold_deg=args.settle_threshold_deg,
    )

    print(f"Feedback source used for summary: {source}")
    if yaw_summary is not None:
        print(
            "Yaw summary: "
            f"start={yaw_summary['start_deg']:+.2f}deg "
            f"end={yaw_summary['end_deg']:+.2f}deg "
            f"target={yaw_summary['target_deg']:+.2f}deg "
            f"peak={yaw_summary['peak_speed_dps']:.2f}deg/s "
            f"avg={yaw_summary['avg_speed_dps']:.2f}deg/s "
            f"settle={yaw_summary['time_to_settle_sec']}"
        )
    if pitch_summary is not None:
        print(
            "Pitch summary: "
            f"start={pitch_summary['start_deg']:+.2f}deg "
            f"end={pitch_summary['end_deg']:+.2f}deg "
            f"target={pitch_summary['target_deg']:+.2f}deg "
            f"peak={pitch_summary['peak_speed_dps']:.2f}deg/s "
            f"avg={pitch_summary['avg_speed_dps']:.2f}deg/s "
            f"settle={pitch_summary['time_to_settle_sec']}"
        )
    print(f"Raw log saved to: {log_path}")


def log_event(log_path: Path, **payload) -> None:
    """Write one non-feedback event row."""
    payload.setdefault("timestamp", time.time())
    write_jsonl(log_path, payload)


def execute_move(
    bridge: PX4SiyiBenchBridge,
    current_pitch_deg: float,
    current_yaw_deg: float,
    target_pitch_deg: float,
    target_yaw_deg: float,
    hold_seconds: float,
    transit_seconds: float,
    move_phase: str,
    hold_phase: str,
    human_label: str,
    args: argparse.Namespace,
    log_path: Path,
    feedback_rows: list[dict],
) -> None:
    """Execute one normal target move and collect the configured hold/transit window."""
    print(f"{human_label} pitch={target_pitch_deg:+.1f} yaw={target_yaw_deg:+.1f}")
    move_ack = send_angle_command(
        bridge,
        target_pitch_deg,
        target_yaw_deg,
        force=True,
        wait_ack=True,
        pitch_rate_dps=args.move_pitch_rate_dps,
        yaw_rate_dps=args.move_yaw_rate_dps,
    )
    log_event(
        log_path,
        event="command",
        phase=move_phase,
        pitch_deg=target_pitch_deg,
        yaw_deg=target_yaw_deg,
        pitch_rate_dps=None if math.isnan(args.move_pitch_rate_dps) else args.move_pitch_rate_dps,
        yaw_rate_dps=None if math.isnan(args.move_yaw_rate_dps) else args.move_yaw_rate_dps,
        ack=move_ack or "none",
    )
    collect_transit_window(
        bridge,
        target_pitch_deg,
        target_yaw_deg,
        hold_seconds=hold_seconds,
        transit_seconds=transit_seconds,
        phase=hold_phase,
        args=args,
        log_path=log_path,
        rows=feedback_rows,
    )


def run_test(args: argparse.Namespace) -> None:
    """Connect to PX4, run the step test, and save a JSONL log."""
    log_path = Path(args.state_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        log_path.unlink()

    bridge = PX4SiyiBenchBridge(args)
    feedback_rows: list[dict] = []

    print(
        f"Connecting to PX4 on {args.serial_device} @ {args.serial_baud} "
        f"(sys={args.mav_target_system}, comp={args.mav_target_component}, gimbal={args.gimbal_device_id}, "
        f"source={args.mav_source_system}/{args.mav_source_component})"
    )
    bridge.connect()
    print(f"Bridge connected. Configure ACK: {bridge.last_configure_ack}")
    log_event(
        log_path,
        event="connect",
        configure_ack=bridge.last_configure_ack,
        dry_run=bool(args.dry_run_mavlink),
    )

    try:
        print(f"Moving to start pose pitch={args.start_pitch:+.1f} yaw={args.start_yaw:+.1f}")
        start_ack = send_angle_command(
            bridge,
            args.start_pitch,
            args.start_yaw,
            force=True,
            wait_ack=True,
            pitch_rate_dps=args.move_pitch_rate_dps,
            yaw_rate_dps=args.move_yaw_rate_dps,
        )
        log_event(
            log_path,
            event="command",
            phase="start_move",
            pitch_deg=args.start_pitch,
            yaw_deg=args.start_yaw,
            pitch_rate_dps=None if math.isnan(args.move_pitch_rate_dps) else args.move_pitch_rate_dps,
            yaw_rate_dps=None if math.isnan(args.move_yaw_rate_dps) else args.move_yaw_rate_dps,
            ack=start_ack or "none",
        )
        collect_transit_window(
            bridge,
            args.start_pitch,
            args.start_yaw,
            args.start_hold_seconds,
            args.start_transit_seconds,
            phase="start_hold",
            args=args,
            log_path=log_path,
            rows=feedback_rows,
        )
        current_pitch_deg = args.start_pitch
        current_yaw_deg = args.start_yaw

        execute_move(
            bridge=bridge,
            current_pitch_deg=current_pitch_deg,
            current_yaw_deg=current_yaw_deg,
            target_pitch_deg=args.target_pitch,
            target_yaw_deg=args.target_yaw,
            hold_seconds=args.target_hold_seconds,
            transit_seconds=args.target_transit_seconds,
            move_phase="target_move",
            hold_phase="target_hold",
            human_label="Step to target pose",
            args=args,
            log_path=log_path,
            feedback_rows=feedback_rows,
        )
        current_pitch_deg = args.target_pitch
        current_yaw_deg = args.target_yaw

        if not math.isnan(args.second_target_pitch) and not math.isnan(args.second_target_yaw):
            execute_move(
                bridge=bridge,
                current_pitch_deg=current_pitch_deg,
                current_yaw_deg=current_yaw_deg,
                target_pitch_deg=args.second_target_pitch,
                target_yaw_deg=args.second_target_yaw,
                hold_seconds=args.second_target_hold_seconds,
                transit_seconds=args.second_target_transit_seconds,
                move_phase="second_target_move",
                hold_phase="second_target_hold",
                human_label="Step to second target pose",
                args=args,
                log_path=log_path,
                feedback_rows=feedback_rows,
            )
            current_pitch_deg = args.second_target_pitch
            current_yaw_deg = args.second_target_yaw

        if not math.isnan(args.third_target_pitch) and not math.isnan(args.third_target_yaw):
            execute_move(
                bridge=bridge,
                current_pitch_deg=current_pitch_deg,
                current_yaw_deg=current_yaw_deg,
                target_pitch_deg=args.third_target_pitch,
                target_yaw_deg=args.third_target_yaw,
                hold_seconds=args.third_target_hold_seconds,
                transit_seconds=args.third_target_transit_seconds,
                move_phase="third_target_move",
                hold_phase="third_target_hold",
                human_label="Step to third target pose",
                args=args,
                log_path=log_path,
                feedback_rows=feedback_rows,
            )
            current_pitch_deg = args.third_target_pitch
            current_yaw_deg = args.third_target_yaw

        if args.return_after:
            execute_move(
                bridge=bridge,
                current_pitch_deg=current_pitch_deg,
                current_yaw_deg=current_yaw_deg,
                target_pitch_deg=args.start_pitch,
                target_yaw_deg=args.start_yaw,
                hold_seconds=args.return_hold_seconds,
                transit_seconds=args.return_transit_seconds,
                move_phase="return_move",
                hold_phase="return_hold",
                human_label="Returning to start pose",
                args=args,
                log_path=log_path,
                feedback_rows=feedback_rows,
            )
    finally:
        bridge.close()
        log_event(log_path, event="close")
        print("Bridge closed.")

    print_summary(feedback_rows, args, log_path)


def main() -> None:
    """Run the simple PX4 -> SIYI gimbal step-speed bench test."""
    args = parse_args()
    run_test(args)


if __name__ == "__main__":
    main()
