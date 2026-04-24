#!/usr/bin/env python3
# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

from __future__ import annotations

import math
import os
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil
from serial import SerialException

mavutil.set_dialect("common")


def mav_result_name(result: int | None) -> str | None:
    """Return a readable MAV_RESULT name."""
    if result is None:
        return None
    enum_entry = mavutil.mavlink.enums.get("MAV_RESULT", {}).get(result)
    return None if enum_entry is None else enum_entry.name


def build_gimbal_manager_flags(args) -> tuple[int, str]:
    """Build the gimbal-manager flags for pitch/yaw commands."""
    flags = 0
    yaw_flag = "lock" if args.yaw_lock else "vehicle"
    flags |= (
        mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        if args.yaw_lock
        else mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
    )
    if args.pitch_lock:
        flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
    return flags, yaw_flag


def euler_pitchyaw_to_quaternion(pitch_rad: float, yaw_rad: float) -> tuple[float, float, float, float]:
    """Convert roll=0, pitch, yaw Euler angles into a quaternion."""
    half_pitch = pitch_rad * 0.5
    half_yaw = yaw_rad * 0.5
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    return (
        cp * cy,
        -sp * sy,
        sp * cy,
        cp * sy,
    )


class PX4SiyiBridge:
    """Live CSI bridge that sends target-control output to PX4 gimbal manager."""

    def __init__(self, args):
        self.args = args
        self.master = None
        self.connected = False
        self.last_configure_ack: str | None = None
        self.last_manager_status: dict | None = None
        self.last_pitchyaw_send: float | None = None
        self._last_send_monotonic = 0.0
        self._last_heartbeat_monotonic = 0.0
        self._send_interval = 0.0 if args.send_rate_hz <= 0 else 1.0 / args.send_rate_hz
        self._flags, self._yaw_flag = build_gimbal_manager_flags(args)

    @property
    def yaw_flag(self) -> str:
        """Return the human-readable yaw frame selection."""
        return self._yaw_flag

    def link_state(self) -> str:
        """Return a short MAVLink bridge status string."""
        if self.args.dry_run_mavlink:
            return "dry-run"
        if not self.connected:
            return "disconnected"
        if self.last_manager_status is not None:
            primary_sysid = self.last_manager_status.get("primary_control_sysid")
            primary_compid = self.last_manager_status.get("primary_control_compid")
            if primary_sysid == self.args.mav_source_system and primary_compid == self.args.mav_source_component:
                return f"ctrl:{primary_sysid}/{primary_compid}"
        if self.last_configure_ack:
            return f"cfg:{self.last_configure_ack}"
        return "connected"

    def connect(self) -> None:
        """Open the MAVLink connection and optionally claim gimbal control."""
        if self.args.dry_run_mavlink:
            self.connected = True
            self.last_configure_ack = "DRY_RUN"
            return

        try:
            self.master = mavutil.mavlink_connection(
                self.args.serial_device,
                baud=self.args.serial_baud,
                source_system=self.args.mav_source_system,
                source_component=self.args.mav_source_component,
                autoreconnect=False,
            )
        except SerialException as exc:
            raise RuntimeError(
                f"Failed to open {self.args.serial_device}: {exc}. "
                "If the port exists but access is denied, start a new shell with dialout permissions "
                "or use 'sg dialout -c ...' for the launcher."
            ) from exc

        heartbeat = self.master.wait_heartbeat(timeout=self.args.heartbeat_timeout)
        if heartbeat is None:
            raise RuntimeError(
                f"No PX4 heartbeat received on {self.args.serial_device} at {self.args.serial_baud} baud within "
                f"{self.args.heartbeat_timeout:.1f}s."
            )

        self.connected = True
        self._send_heartbeat(force=True)
        time.sleep(0.05)
        if not self.args.skip_configure:
            self.last_configure_ack = self.configure_control()
        else:
            self.last_configure_ack = "SKIPPED"

    def close(self) -> None:
        """Send a stop command and close the serial port."""
        try:
            if self.connected:
                self.send_rates(0.0, 0.0, force=True)
        finally:
            if self.master is not None:
                self.master.close()
            self.connected = False

    def wait_for_ack(self, command_id: int, timeout: float) -> str | None:
        """Wait for COMMAND_ACK for a particular MAV_CMD."""
        if self.master is None:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=max(deadline - time.monotonic(), 0.1))
            if msg is None:
                continue
            if getattr(msg, "command", None) != command_id:
                continue
            return mav_result_name(getattr(msg, "result", None)) or f"RESULT_{getattr(msg, 'result', None)}"
        return None

    def request_manager_status(self, timeout: float = 1.0) -> dict | None:
        """Request and return the current PX4 gimbal-manager status."""
        if self.master is None:
            return None
        self._send_heartbeat(force=True)
        self.master.mav.command_long_send(
            self.args.mav_target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type="GIMBAL_MANAGER_STATUS",
                blocking=True,
                timeout=max(deadline - time.monotonic(), 0.1),
            )
            if msg is None:
                continue
            status = msg.to_dict()
            self.last_manager_status = status
            return status
        return None

    def _send_heartbeat(self, force: bool = False) -> None:
        """Send a lightweight companion heartbeat so PX4 sees a live control component."""
        if self.master is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_heartbeat_monotonic) < 1.0:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self._last_heartbeat_monotonic = now

    def configure_control(self) -> str:
        """Claim primary control of the PX4 gimbal manager for the Jetson component."""
        if self.master is None:
            raise RuntimeError("MAVLink connection is not open.")
        last_ack = None
        for _ in range(3):
            self._send_heartbeat(force=True)
            self.master.mav.command_long_send(
                self.args.mav_target_system,
                mavutil.mavlink.MAV_COMP_ID_ALL,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                0,
                float(self.args.mav_source_system),
                float(self.args.mav_source_component),
                float(self.args.secondary_control_system),
                float(self.args.secondary_control_component),
                0.0,
                0.0,
                float(self.args.gimbal_device_id),
            )
            last_ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, self.args.ack_timeout)
            status = self.request_manager_status(timeout=max(self.args.ack_timeout, 0.8))
            if status is not None:
                if (
                    status.get("primary_control_sysid") == self.args.mav_source_system
                    and status.get("primary_control_compid") == self.args.mav_source_component
                ):
                    if last_ack:
                        return f"{last_ack} primary:{self.args.mav_source_system}/{self.args.mav_source_component}"
                    return f"PRIMARY:{self.args.mav_source_system}/{self.args.mav_source_component}"
            time.sleep(0.1)
        return last_ack or "NO_ACK"

    def send_pitchyaw(
        self,
        pitch_angle_deg: float = math.nan,
        yaw_angle_deg: float = math.nan,
        pitch_rate_dps: float = math.nan,
        yaw_rate_dps: float = math.nan,
        force: bool = False,
        wait_ack: bool = False,
    ) -> str | None:
        """Send a generic pitch/yaw command to PX4's gimbal manager."""
        if not self.connected or self.master is None:
            return None
        now = time.monotonic()
        if not force and self._send_interval > 0 and (now - self._last_send_monotonic) < self._send_interval:
            return None

        self._send_heartbeat()
        pitch_angle_rad = math.radians(pitch_angle_deg) if not math.isnan(pitch_angle_deg) else math.nan
        yaw_angle_rad = math.radians(yaw_angle_deg) if not math.isnan(yaw_angle_deg) else math.nan
        pitch_rate_rads = math.radians(pitch_rate_dps) if not math.isnan(pitch_rate_dps) else math.nan
        yaw_rate_rads = math.radians(yaw_rate_dps) if not math.isnan(yaw_rate_dps) else math.nan

        if self.args.control_api in {"attitude", "message"}:
            q = (
                euler_pitchyaw_to_quaternion(pitch_angle_rad, yaw_angle_rad)
                if not math.isnan(pitch_angle_rad) and not math.isnan(yaw_angle_rad)
                else (math.nan, math.nan, math.nan, math.nan)
            )
            self.master.mav.gimbal_manager_set_attitude_send(
                self.args.mav_target_system,
                self.args.mav_target_component,
                self._flags,
                self.args.gimbal_device_id,
                q,
                math.nan,
                pitch_rate_rads,
                yaw_rate_rads,
            )
        elif self.args.control_api == "manual":
            self.master.mav.gimbal_manager_set_manual_control_send(
                self.args.mav_target_system,
                self.args.mav_target_component,
                self._flags,
                self.args.gimbal_device_id,
                pitch_angle_rad if not math.isnan(pitch_angle_rad) else math.nan,
                yaw_angle_rad if not math.isnan(yaw_angle_rad) else math.nan,
                pitch_rate_dps if not math.isnan(pitch_rate_dps) else math.nan,
                yaw_rate_dps if not math.isnan(yaw_rate_dps) else math.nan,
            )
        elif self.args.control_api == "pitchyaw":
            self.master.mav.gimbal_manager_set_pitchyaw_send(
                self.args.mav_target_system,
                self.args.mav_target_component,
                self._flags,
                self.args.gimbal_device_id,
                pitch_angle_rad,
                yaw_angle_rad,
                pitch_rate_rads,
                yaw_rate_rads,
            )
        else:
            self.master.mav.command_long_send(
                self.args.mav_target_system,
                mavutil.mavlink.MAV_COMP_ID_ALL,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,
                float(pitch_angle_deg),
                float(yaw_angle_deg),
                float(pitch_rate_dps),
                float(yaw_rate_dps),
                float(self._flags),
                0.0,
                float(self.args.gimbal_device_id),
            )
        self._last_send_monotonic = now
        self.last_pitchyaw_send = time.time()
        if wait_ack:
            if self.args.control_api in {"attitude", "message", "manual", "pitchyaw"}:
                return "SENT"
            ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, self.args.ack_timeout)
            return ack or "NO_ACK"
        return None

    def send_rates(self, pitch_rate_dps: float, yaw_rate_dps: float, force: bool = False) -> None:
        """Send pitch/yaw rate commands to PX4's gimbal manager."""
        self.send_pitchyaw(
            pitch_rate_dps=pitch_rate_dps,
            yaw_rate_dps=yaw_rate_dps,
            force=force,
            wait_ack=False,
        )
