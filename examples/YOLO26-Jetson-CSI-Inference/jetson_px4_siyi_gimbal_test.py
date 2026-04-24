#!/usr/bin/env python3
# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
for path in (REPO_ROOT, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from pymavlink import mavutil

from px4_siyi_bench_bridge import PX4SiyiBenchBridge


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for a simple PX4 -> SIYI bench test."""
    parser = argparse.ArgumentParser(
        description="Send simple fixed gimbal angle or rate commands from Jetson -> PX4 -> SIYI."
    )
    parser.add_argument(
        "--mode",
        default="angle",
        choices=("angle", "rate"),
        help="Use fixed angle command or fixed angular-rate command",
    )
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch angle in degrees for --mode angle")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees for --mode angle")
    parser.add_argument("--pitch-rate", type=float, default=0.0, help="Pitch rate in deg/s for --mode rate")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="Yaw rate in deg/s for --mode rate")
    parser.add_argument("--hold-seconds", type=float, default=2.0, help="Seconds to wait after the main command")
    parser.add_argument(
        "--resend-hz",
        type=float,
        default=20.0,
        help="Re-send the same command at this rate while holding",
    )
    parser.add_argument(
        "--neutral-after",
        action="store_true",
        help="After the hold period, send a neutral command before exit",
    )
    parser.add_argument("--neutral-pitch", type=float, default=0.0, help="Neutral pitch angle in degrees")
    parser.add_argument("--neutral-yaw", type=float, default=0.0, help="Neutral yaw angle in degrees")
    parser.add_argument(
        "--neutral-hold-seconds",
        type=float,
        default=1.0,
        help="Seconds to wait after sending the neutral command",
    )

    parser.add_argument("--serial-device", default="/dev/ttyUSB0", help="Jetson -> PX4 MAVLink serial device")
    parser.add_argument("--serial-baud", type=int, default=921600, help="Jetson -> PX4 MAVLink baud rate")
    parser.add_argument("--mav-source-system", type=int, default=1, help="MAVLink source system ID for the Jetson")
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


def describe_command(args: argparse.Namespace) -> str:
    """Return a readable summary of the configured test command."""
    if args.mode == "angle":
        return f"{args.control_api} angle pitch={args.pitch:+.1f}deg yaw={args.yaw:+.1f}deg"
    return f"{args.control_api} rate pitch_rate={args.pitch_rate:+.1f}deg/s yaw_rate={args.yaw_rate:+.1f}deg/s"


def send_main_command(bridge: PX4SiyiBenchBridge, args: argparse.Namespace, wait_ack: bool) -> str | None:
    """Send the main angle or rate command."""
    if args.mode == "angle":
        return bridge.send_pitchyaw(
            pitch_angle_deg=args.pitch,
            yaw_angle_deg=args.yaw,
            pitch_rate_dps=math.nan,
            yaw_rate_dps=math.nan,
            force=True,
            wait_ack=wait_ack,
        )
    return bridge.send_pitchyaw(
        pitch_angle_deg=math.nan,
        yaw_angle_deg=math.nan,
        pitch_rate_dps=args.pitch_rate,
        yaw_rate_dps=args.yaw_rate,
        force=True,
        wait_ack=wait_ack,
    )


def send_neutral_command(bridge: PX4SiyiBenchBridge, args: argparse.Namespace) -> str | None:
    """Send a neutral angle/rate command."""
    if args.mode == "angle":
        return bridge.send_pitchyaw(
            pitch_angle_deg=args.neutral_pitch,
            yaw_angle_deg=args.neutral_yaw,
            pitch_rate_dps=math.nan,
            yaw_rate_dps=math.nan,
            force=True,
            wait_ack=True,
        )
    return bridge.send_pitchyaw(
        pitch_angle_deg=math.nan,
        yaw_angle_deg=math.nan,
        pitch_rate_dps=0.0,
        yaw_rate_dps=0.0,
        force=True,
        wait_ack=True,
    )


def hold_neutral_command(bridge: PX4SiyiBenchBridge, args: argparse.Namespace) -> None:
    """Optionally keep re-sending the neutral command while holding."""
    if args.neutral_hold_seconds <= 0:
        return
    if args.resend_hz <= 0:
        time.sleep(args.neutral_hold_seconds)
        return

    interval = 1.0 / args.resend_hz
    deadline = time.monotonic() + args.neutral_hold_seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(interval, remaining))
        send_neutral_command(bridge, args)


def hold_command(bridge: PX4SiyiBenchBridge, args: argparse.Namespace) -> None:
    """Hold the current command, optionally re-sending it."""
    if args.hold_seconds <= 0:
        return
    if args.resend_hz <= 0:
        time.sleep(args.hold_seconds)
        return

    interval = 1.0 / args.resend_hz
    deadline = time.monotonic() + args.hold_seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(interval, remaining))
        send_main_command(bridge, args, wait_ack=False)


def run_test(args: argparse.Namespace) -> None:
    """Connect to PX4, send the requested command, and optionally neutralize."""
    bridge = PX4SiyiBenchBridge(args)
    print(
        f"Connecting to PX4 on {args.serial_device} @ {args.serial_baud} "
        f"(sys={args.mav_target_system}, comp={args.mav_target_component}, gimbal={args.gimbal_device_id}, "
        f"source={args.mav_source_system}/{args.mav_source_component})"
    )
    bridge.connect()
    print(f"Bridge connected. Configure ACK: {bridge.last_configure_ack}")
    if bridge.last_manager_status is not None:
        print(
            "Manager status: "
            f"primary={bridge.last_manager_status.get('primary_control_sysid')}/"
            f"{bridge.last_manager_status.get('primary_control_compid')} "
            f"device={bridge.last_manager_status.get('gimbal_device_id')}"
        )
    print(f"Sending {describe_command(args)}")

    try:
        ack = send_main_command(bridge, args, wait_ack=True)
        print(f"Pitch/yaw ACK: {ack or 'none'}")
        hold_command(bridge, args)

        if args.neutral_after:
            if args.mode == "angle":
                print(f"Sending neutral angle pitch={args.neutral_pitch:+.1f}deg yaw={args.neutral_yaw:+.1f}deg")
            else:
                print("Sending neutral rates pitch_rate=+0.0deg/s yaw_rate=+0.0deg/s")
            neutral_ack = send_neutral_command(bridge, args)
            print(f"Neutral ACK: {neutral_ack or 'none'}")
            hold_neutral_command(bridge, args)
    finally:
        bridge.close()
        print("Bridge closed.")


def main() -> None:
    """Run the simple PX4 -> SIYI gimbal bench test."""
    args = parse_args()
    run_test(args)


if __name__ == "__main__":
    main()
