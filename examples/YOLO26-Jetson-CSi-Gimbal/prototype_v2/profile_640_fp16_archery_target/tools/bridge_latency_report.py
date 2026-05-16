#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

PROFILE_DIR = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize prototype_v2 bridge latency and drop metrics for one log session."
    )
    parser.add_argument(
        "--bridge-log",
        default="",
        help="Bridge JSONL log path; if omitted, runs/latest/detection_log.jsonl is preferred, then legacy /tmp logs",
    )
    parser.add_argument(
        "--session-index",
        type=int,
        default=-1,
        help="Session index after splitting append-only logs by frame/timestamp reset; default is latest",
    )
    return parser.parse_args()


def resolve_bridge_log(path_arg: str) -> Path:
    if path_arg:
        path = Path(path_arg)
        if not path.exists():
            raise SystemExit(f"Bridge log not found: {path}")
        return path

    candidates = [
        PROFILE_DIR / "runs/latest/detection_log.jsonl",
        Path("/tmp/profile640fp16_archery_target_mk15_live_bridge.jsonl"),
        Path("/tmp/profile640fp16_archery_target_live_bridge.jsonl"),
        Path("/tmp/profile640fp16_archery_target_bridge_dryrun.jsonl"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit(
        "Bridge log not found. Checked: "
        + ", ".join(str(candidate) for candidate in candidates)
    )


def load_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def split_sessions(rows: list[dict]) -> list[list[dict]]:
    sessions: list[list[dict]] = []
    cur: list[dict] = []
    last_ts = None
    last_frame = None

    for row in rows:
        ts = row.get("timestamp")
        frame = row.get("frame_idx")
        new_session = False
        if last_ts is not None:
            if isinstance(frame, int) and isinstance(last_frame, int) and frame < last_frame:
                new_session = True
            elif isinstance(ts, (int, float)) and isinstance(last_ts, (int, float)) and ts < last_ts - 1.0:
                new_session = True
        if new_session and cur:
            sessions.append(cur)
            cur = []
        cur.append(row)
        last_ts = ts
        last_frame = frame

    if cur:
        sessions.append(cur)
    return sessions


def numeric_values(rows: list[dict], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def average(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def maximum(values: list[float]) -> float | None:
    if not values:
        return None
    return max(values)


def format_float(value: float | None, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def main() -> int:
    args = parse_args()
    bridge_path = resolve_bridge_log(args.bridge_log)
    rows = load_rows(bridge_path)
    if not rows:
        raise SystemExit(f"No JSON rows found in: {bridge_path}")

    sessions = split_sessions(rows)
    try:
        session = sessions[args.session_index]
    except IndexError as exc:
        raise SystemExit(f"Session index out of range: {args.session_index}; sessions={len(sessions)}") from exc

    start_ts = float(session[0].get("timestamp", 0.0) or 0.0)
    end_ts = float(session[-1].get("timestamp", start_ts) or start_ts)
    duration_s = max(0.0, end_ts - start_ts)

    frame_values = [row.get("frame_idx") for row in session if isinstance(row.get("frame_idx"), int)]
    first_frame = frame_values[0] if frame_values else None
    last_frame = frame_values[-1] if frame_values else None

    vision_fps = None
    if duration_s > 0.0 and first_frame is not None and last_frame is not None and last_frame >= first_frame:
        vision_fps = ((last_frame - first_frame) + 1) / duration_s

    control_hz = (len(session) / duration_s) if duration_s > 0.0 else None
    mav_send_count = len(numeric_values(session, "mav_send_mono_ns"))
    mav_send_hz = (mav_send_count / duration_s) if duration_s > 0.0 else None

    vision_latency = numeric_values(session, "vision_latency_ms")
    metadata_age = numeric_values(session, "metadata_age_ms")
    mavlink_delay = numeric_values(session, "mavlink_delay_ms")
    feedback_delay = numeric_values(session, "feedback_delay_ms")

    dropped_metadata_frames = int(sum(int(row.get("metadata_gap_frames", 0) or 0) for row in session))
    dropped_video_frames = int(sum(int(row.get("video_frame_gap", 0) or 0) for row in session))

    frame_pts_count = sum(1 for row in session if isinstance(row.get("frame_pts_ns"), int))
    frame_ntp_count = sum(1 for row in session if isinstance(row.get("frame_ntp_ns"), int))

    print(f"bridge_log={bridge_path}")
    print(f"sessions_total={len(sessions)}")
    print(f"session_index={args.session_index}")
    print(f"session_rows={len(session)}")
    print(f"frame_range={first_frame}..{last_frame}")
    print(f"duration_s={duration_s:.3f}")
    print(f"Vision FPS: {format_float(vision_fps)}")
    print(f"Control Hz: {format_float(control_hz)}")
    print(f"Avg vision latency: {format_float(average(vision_latency))} ms")
    print(f"Max vision latency: {format_float(maximum(vision_latency))} ms")
    print(f"Avg metadata age: {format_float(average(metadata_age))} ms")
    print(f"Max metadata age: {format_float(maximum(metadata_age))} ms")
    print(f"Avg MAVLink delay: {format_float(average(mavlink_delay))} ms")
    print(f"Max MAVLink delay: {format_float(maximum(mavlink_delay))} ms")
    print(f"Avg feedback delay: {format_float(average(feedback_delay))} ms")
    print(f"Max feedback delay: {format_float(maximum(feedback_delay))} ms")
    print(f"MAVLink send Hz: {format_float(mav_send_hz)}")
    print(f"Dropped video frames: {dropped_video_frames}")
    print(f"Dropped metadata frames: {dropped_metadata_frames}")
    print(f"Frame PTS carried: {frame_pts_count}/{len(session)}")
    print(f"Frame NTP carried: {frame_ntp_count}/{len(session)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
