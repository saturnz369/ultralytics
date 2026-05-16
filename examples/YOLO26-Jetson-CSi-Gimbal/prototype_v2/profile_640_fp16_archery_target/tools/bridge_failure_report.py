#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

PROFILE_DIR = Path(__file__).resolve().parent.parent


@dataclass
class Segment:
    present: bool
    start_idx: int
    end_idx: int
    length: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze a prototype_v2 bridge JSONL log and report missed-detection windows."
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
    parser.add_argument(
        "--class-name",
        default="target_face",
        help="Class name treated as a successful target detection",
    )
    parser.add_argument(
        "--min-miss-frames",
        type=int,
        default=15,
        help="Only report miss windows with at least this many consecutive frames",
    )
    parser.add_argument(
        "--video",
        default="",
        help="Optional recorded video path; if set, emit ffmpeg extraction commands for miss windows",
    )
    parser.add_argument(
        "--output-dir",
        default="failure_extracts",
        help="Output directory name used in emitted ffmpeg commands",
    )
    parser.add_argument(
        "--pad-seconds",
        type=float,
        default=0.75,
        help="Padding around miss windows for emitted clip commands",
    )
    return parser.parse_args()


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


def build_segments(rows: list[dict], class_name: str) -> list[Segment]:
    present = [1 if row.get("class_name") == class_name else 0 for row in rows]
    if not present:
        return []

    segments: list[Segment] = []
    cur = present[0]
    start = 0
    length = 1

    for idx, value in enumerate(present[1:], start=1):
        if value == cur:
            length += 1
            continue
        segments.append(Segment(bool(cur), start, idx - 1, length))
        cur = value
        start = idx
        length = 1

    segments.append(Segment(bool(cur), start, len(present) - 1, length))
    return segments


def fmt_time(seconds: float) -> str:
    if seconds < 0:
        seconds = 0.0
    minutes, secs = divmod(seconds, 60.0)
    hours, minutes = divmod(int(minutes), 60)
    if hours > 0:
        return f"{hours:02d}:{minutes:02d}:{secs:06.3f}"
    return f"{minutes:02d}:{secs:06.3f}"


def main() -> int:
    args = parse_args()
    bridge_path = resolve_bridge_log(args.bridge_log)

    rows = load_rows(bridge_path)
    if not rows:
        raise SystemExit(f"No JSON rows found in: {bridge_path}")

    sessions = split_sessions(rows)
    session = sessions[args.session_index]
    start_ts = session[0].get("timestamp", 0.0)
    end_ts = session[-1].get("timestamp", 0.0)
    duration = max(0.0, end_ts - start_ts)

    print(f"bridge_log={bridge_path}")
    print(f"sessions_total={len(sessions)}")
    print(f"session_rows={len(session)}")
    print(f"frame_range={session[0].get('frame_idx')}..{session[-1].get('frame_idx')}")
    print(f"duration_s={duration:.3f}")

    segments = build_segments(session, args.class_name)
    miss_segments = [seg for seg in segments if not seg.present and seg.length >= args.min_miss_frames]
    hit_segments = [seg for seg in segments if seg.present]

    print(f"hit_segments={len(hit_segments)}")
    print(f"miss_segments_ge_{args.min_miss_frames}={len(miss_segments)}")
    print()

    if not miss_segments:
        print("No miss windows matched the current threshold.")
        return 0

    for idx, seg in enumerate(sorted(miss_segments, key=lambda s: s.length, reverse=True), start=1):
        start_row = session[seg.start_idx]
        end_row = session[seg.end_idx]
        start_rel = max(0.0, float(start_row.get("timestamp", 0.0)) - start_ts)
        end_rel = max(0.0, float(end_row.get("timestamp", 0.0)) - start_ts)
        duration_rel = max(0.0, end_rel - start_rel)
        start_frame = start_row.get("frame_idx")
        end_frame = end_row.get("frame_idx")
        print(
            f"[{idx}] miss frames={start_frame}..{end_frame} "
            f"len={seg.length} rel={fmt_time(start_rel)}..{fmt_time(end_rel)} dur={duration_rel:.3f}s"
        )

        if args.video:
            clip_start = max(0.0, start_rel - args.pad_seconds)
            clip_duration = duration_rel + (2.0 * args.pad_seconds)
            mid_time = start_rel + (duration_rel / 2.0)
            clip_name = f"{args.output_dir}/miss_{idx:02d}_{start_frame}_{end_frame}.mkv"
            frame_name = f"{args.output_dir}/miss_{idx:02d}_{start_frame}_{end_frame}_mid.jpg"
            print(
                f"  ffmpeg -y -ss {clip_start:.3f} -i '{args.video}' -t {clip_duration:.3f} "
                f"-c:v libx264 -crf 18 '{clip_name}'"
            )
            print(
                f"  ffmpeg -y -ss {mid_time:.3f} -i '{args.video}' -frames:v 1 '{frame_name}'"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
