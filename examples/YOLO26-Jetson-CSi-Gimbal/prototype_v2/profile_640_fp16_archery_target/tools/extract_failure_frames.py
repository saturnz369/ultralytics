#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import subprocess
import zipfile
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path


VIDEO_TS_RE = re.compile(r"(\d{8}-\d{6})")


@dataclass
class Segment:
    present: bool
    start_row: int
    end_row: int
    length: int


@dataclass
class ExtractedFrame:
    image_name: str
    segment_index: int
    frame_idx: int
    segment_length_frames: int
    segment_start_frame: int
    segment_end_frame: int
    bridge_rel_time_s: float
    video_time_s: float
    bridge_timestamp: float
    confidence: float | None
    status: str | None
    command_status: str | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract still images from missed-detection windows using the bridge log and a recorded RTSP video."
    )
    parser.add_argument(
        "--bridge-log",
        default="",
        help="Bridge JSONL log path; if omitted, the newest common archery-target bridge log is used",
    )
    parser.add_argument(
        "--video",
        required=True,
        help="Recorded RTSP video path",
    )
    parser.add_argument(
        "--class-name",
        default="target_face",
        help="Class name considered a successful detection",
    )
    parser.add_argument(
        "--min-miss-frames",
        type=int,
        default=10,
        help="Minimum consecutive missed frames for a segment to be extracted",
    )
    parser.add_argument(
        "--max-miss-frames",
        type=int,
        default=60,
        help="Maximum consecutive missed frames for a segment to be extracted",
    )
    parser.add_argument(
        "--frames-per-segment",
        type=int,
        default=3,
        help="How many stills to extract per selected miss segment",
    )
    parser.add_argument(
        "--session-index",
        type=int,
        default=None,
        help="Session index after splitting append-only logs; default auto-selects the session closest to the video start",
    )
    parser.add_argument(
        "--include-edge-misses",
        action="store_true",
        help="Also extract miss windows at the beginning or end of the selected session",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Optional extraction root; defaults to sibling folder under recordings/",
    )
    return parser.parse_args()


def resolve_bridge_log(path_arg: str) -> Path:
    if path_arg:
        path = Path(path_arg)
        if not path.exists():
            raise SystemExit(f"Bridge log not found: {path}")
        return path

    candidates = [
        Path("/tmp/profile640fp16_archery_target_mk15_live_bridge.jsonl"),
        Path("/tmp/profile640fp16_archery_target_live_bridge.jsonl"),
        Path("/tmp/profile640fp16_archery_target_bridge_dryrun.jsonl"),
    ]
    existing = [path for path in candidates if path.exists()]
    if not existing:
        raise SystemExit(
            "Bridge log not found. Checked: "
            + ", ".join(str(candidate) for candidate in candidates)
        )
    return max(existing, key=lambda path: path.stat().st_mtime)


def load_rows(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
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


def select_session(sessions: list[list[dict]], session_index: int | None, video_start_epoch: float) -> list[dict]:
    if not sessions:
        raise SystemExit("No sessions found in bridge log")
    if session_index is not None:
        try:
            return sessions[session_index]
        except IndexError as exc:
            raise SystemExit(f"Session index out of range: {session_index}; sessions={len(sessions)}") from exc

    def start_delta(session: list[dict]) -> float:
        ts = session[0].get("timestamp")
        if isinstance(ts, (int, float)):
            return abs(float(ts) - video_start_epoch)
        return math.inf

    return min(sessions, key=start_delta)


def build_segments(rows: list[dict], class_name: str) -> list[Segment]:
    present = [1 if row.get("class_name") == class_name else 0 for row in rows]
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


def ffprobe_duration(video: Path) -> float:
    commands = [
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "default=nw=1:nk=1",
            str(video),
        ],
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "stream=duration",
            "-of",
            "default=nw=1:nk=1",
            str(video),
        ],
    ]
    out = ""
    for cmd in commands:
        out = subprocess.check_output(cmd, text=True).strip()
        for line in reversed(out.splitlines()):
            line = line.strip()
            if not line or line == "N/A":
                continue
            try:
                return float(line)
            except ValueError:
                continue

    fps_cmd = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "stream=avg_frame_rate",
        "-of",
        "default=nw=1:nk=1",
        str(video),
    ]
    packet_cmd = [
        "ffprobe",
        "-v",
        "error",
        "-count_packets",
        "-show_entries",
        "stream=nb_read_packets",
        "-of",
        "default=nw=1:nk=1",
        str(video),
    ]
    fps_out = subprocess.check_output(fps_cmd, text=True).strip()
    packet_out = subprocess.check_output(packet_cmd, text=True).strip()
    fps = None
    for line in reversed(fps_out.splitlines()):
        line = line.strip()
        if not line:
            continue
        if "/" in line:
            num_str, den_str = line.split("/", 1)
            try:
                num = float(num_str)
                den = float(den_str)
                if den != 0.0:
                    fps = num / den
                    break
            except ValueError:
                continue
        else:
            try:
                fps = float(line)
                break
            except ValueError:
                continue

    packets = None
    for line in reversed(packet_out.splitlines()):
        line = line.strip()
        if not line or line == "N/A":
            continue
        try:
            packets = float(line)
            break
        except ValueError:
            continue

    if fps and packets:
        return packets / fps

    raise ValueError(f"Could not parse ffprobe duration output: {out!r}")


def infer_video_start_epoch(video: Path) -> float:
    ffprobe_cmd = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "format_tags=creation_time",
        "-of",
        "default=nw=1:nk=1",
        str(video),
    ]
    try:
        out = subprocess.check_output(ffprobe_cmd, text=True).strip()
        for line in reversed(out.splitlines()):
            line = line.strip()
            if not line:
                continue
            iso_value = line.replace("Z", "+00:00")
            try:
                return datetime.fromisoformat(iso_value).timestamp()
            except ValueError:
                continue
    except subprocess.CalledProcessError:
        pass

    match = VIDEO_TS_RE.search(video.name)
    if not match:
        raise ValueError(f"Could not infer start timestamp from video filename: {video.name}")
    dt = datetime.strptime(match.group(1), "%Y%m%d-%H%M%S")
    return dt.timestamp()


def choose_row_positions(length: int, frames_per_segment: int) -> list[int]:
    if frames_per_segment <= 1 or length <= 1:
        return [length // 2]
    fractions = [(idx + 1) / (frames_per_segment + 1) for idx in range(frames_per_segment)]
    positions = sorted({min(length - 1, max(0, int(round((length - 1) * frac)))) for frac in fractions})
    return positions


def extract_frame(video: Path, video_time_s: float, output_path: Path) -> None:
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-ss",
        f"{video_time_s:.3f}",
        "-i",
        str(video),
        "-frames:v",
        "1",
        "-q:v",
        "2",
        str(output_path),
    ]
    subprocess.run(cmd, check=True)


def main() -> int:
    args = parse_args()
    bridge_log = resolve_bridge_log(args.bridge_log)
    video = Path(args.video)
    if not video.exists():
        raise SystemExit(f"Video not found: {video}")

    rows = load_rows(bridge_log)
    if not rows:
        raise SystemExit("Bridge log is empty")

    video_duration_s = ffprobe_duration(video)
    video_start_epoch = infer_video_start_epoch(video)
    sessions = split_sessions(rows)
    session = select_session(sessions, args.session_index, video_start_epoch)
    session_start_epoch = float(session[0]["timestamp"])
    session_end_epoch = float(session[-1]["timestamp"])
    session_duration_s = session_end_epoch - session_start_epoch
    bridge_minus_video_start_s = session_start_epoch - video_start_epoch

    segments = build_segments(session, args.class_name)
    miss_segments = [
        seg
        for idx, seg in enumerate(segments)
        if not seg.present
        and (args.include_edge_misses or (idx > 0 and idx < len(segments) - 1))
        and args.min_miss_frames <= seg.length <= args.max_miss_frames
    ]

    if args.output_root:
        output_root = Path(args.output_root)
    else:
        output_root = video.parent / f"{video.stem}_failure_frames"
    output_root.mkdir(parents=True, exist_ok=True)

    extracted: list[ExtractedFrame] = []
    for seg_idx, seg in enumerate(miss_segments, start=1):
        positions = choose_row_positions(seg.length, args.frames_per_segment)
        for shot_idx, relative_pos in enumerate(positions, start=1):
            row_index = seg.start_row + relative_pos
            row = session[row_index]
            bridge_rel_time_s = float(row["timestamp"]) - session_start_epoch
            video_time_s = bridge_rel_time_s + bridge_minus_video_start_s
            if video_time_s < 0.0 or video_time_s > video_duration_s:
                continue
            image_name = f"miss_{seg_idx:03d}_f{row['frame_idx']}_p{shot_idx}.jpg"
            image_path = output_root / image_name
            extract_frame(video, video_time_s, image_path)
            extracted.append(
                ExtractedFrame(
                    image_name=image_name,
                    segment_index=seg_idx,
                    frame_idx=int(row["frame_idx"]),
                    segment_length_frames=seg.length,
                    segment_start_frame=int(session[seg.start_row]["frame_idx"]),
                    segment_end_frame=int(session[seg.end_row]["frame_idx"]),
                    bridge_rel_time_s=bridge_rel_time_s,
                    video_time_s=video_time_s,
                    bridge_timestamp=float(row["timestamp"]),
                    confidence=float(row["confidence"]) if isinstance(row.get("confidence"), (int, float)) else None,
                    status=row.get("status"),
                    command_status=row.get("command_status"),
                )
            )

    manifest_path = output_root / "manifest.jsonl"
    with manifest_path.open("w", encoding="utf-8") as handle:
        for item in extracted:
            handle.write(json.dumps(asdict(item), ensure_ascii=False) + "\n")

    summary = {
        "bridge_log": str(bridge_log),
        "video": str(video),
        "session_rows": len(session),
        "session_duration_s": session_duration_s,
        "video_duration_s": video_duration_s,
        "session_start_epoch": session_start_epoch,
        "video_start_epoch": video_start_epoch,
        "bridge_minus_video_start_s": bridge_minus_video_start_s,
        "sessions_total": len(sessions),
        "selected_session_index": sessions.index(session),
        "selected_miss_segments": len(miss_segments),
        "extracted_images": len(extracted),
        "min_miss_frames": args.min_miss_frames,
        "max_miss_frames": args.max_miss_frames,
        "frames_per_segment": args.frames_per_segment,
    }
    (output_root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    zip_path = output_root.with_suffix(".zip")
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(output_root.rglob("*")):
            if path.is_file():
                archive.write(path, arcname=path.relative_to(output_root.parent))

    print(json.dumps(summary, indent=2))
    print(f"output_dir={output_root}")
    print(f"zip_file={zip_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
