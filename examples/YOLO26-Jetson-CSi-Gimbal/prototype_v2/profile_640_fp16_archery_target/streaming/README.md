# MK15 Streaming README

This is the MK15 streaming note for:

- `/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target`

It uses the same clean streaming/runtime architecture as the main `profile_640_fp16` profile, but the YOLO modes in this folder are meant for the custom one-class `target_face` detector.

Primary MK15 URL:

```text
rtsp://192.168.144.100:8554/stream
```

## Current Status

- Camera-only RTSP launcher is ready now.
- YOLO RTSP launchers are wired to this profile now.
- The custom `target_face` model is now imported into this profile.
- Runtime model artifacts now exist:
  - `model/best.pt`
  - `model/target_face_v1_native_640.onnx`
  - `model/model_b1_gpu0_fp16.engine`
- A bounded local DeepStream sanity run already passed with the new engine.
- The full gimbal+RTSP wrapper now uses the fixed `/stream` server path again.
- Local `ffprobe` and `ffmpeg` decode checks are done against `/stream` from the exact full gimbal+stream launcher.
- The old blank-screen case on MK15 was traced to the RTSP wrapper and fixed in this profile.
- So:
  - camera-only RTSP can be used immediately
  - YOLO RTSP modes are now ready for real testing with the new model

## Plain Architecture

The monitoring/video branch is:

```text
CSI camera -> Jetson -> H.264 RTSP -> MK15 air unit -> MK15 handheld FPV
```

For the YOLO stream, it becomes:

```text
CSI camera -> DeepStream / YOLO / tracker / overlay -> H.264 RTSP -> MK15
```

The control branch stays separate:

```text
DeepStream tracker -> latest shared-memory metadata snapshot -> Python MAVLink bridge -> PX4 -> SIYI
```

So RTSP is still the monitoring path, not the control signal.

## Before YOLO Streaming

The model import and first engine build are already done for this profile.

If you replace the model later, the expected refresh path is still:

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/prepare_prototype_v2_model.sh
```

Then do one local DeepStream run so the engine is rebuilt.

## Output Settings

Current default RTSP output is now:

- `2028x1112`
- `60 fps`
- `H.264`

The `640` in this profile name is the YOLO inference size, not the MK15 stream size.

Current expected picture:

```text
pipeline capture: 2028x1112 @ 60
YOLO inference: 640x640
MK15 RTSP output: 2028x1112 @ 60
```

## Launch 1: Camera Only To MK15

This path does not need the `target_face` model at all.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_csi_rtsp.sh
```

On MK15:

1. Open the `FPV` app.
2. Set `Camera A` to `rtsp://192.168.144.100:8554/stream`.
3. Reopen the stream view.

## Launch 2: YOLO Overlay To MK15

The `target_face` model import, ONNX export, and first engine build are already done for this profile.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_rtsp.sh
```

Notes:

- `TARGET_CLASS_ID=0` is the custom `target_face` class in this profile.
- This is monitoring/video only. No PX4 or gimbal is required.
- MK15 URL: `rtsp://192.168.144.100:8554/stream`

If motion still looks too compressed on MK15, try a higher bitrate for a run:

```bash
cd /home/saturnzzz/ultralytics
BITRATE=10000000 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_rtsp.sh
```

## Engine Build Note

For this current `v1` bring-up, the first engine was built with a fast first-pass TensorRT build on the Jetson so the new profile could be validated quickly.

For a future more serious model version, such as `v2`, rebuild and keep a normal/final engine after the model is already accepted.

## Launch 3: YOLO + Gimbal + RTSP To MK15

Use this only after:

- the `target_face` model is working locally
- PX4 and SIYI hardware are connected
- `/dev/ttyUSB0` access is okay

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

This one-shot launcher already carries the working defaults for this profile:

- `SENSOR_ID=0`
- `CAMERA_WIDTH=2028`
- `CAMERA_HEIGHT=1112`
- `CAMERA_FPS_N=60`
- `CAMERA_FPS_D=1`
- `SHOW=1`
- `RTSP_ENABLE=1`
- `RTSP_MOUNT=/stream`
- `CONTROL_API=command`
- `LIVE_CONTROL_MODE=angle-target`
- `MAV_INVERT_PAN=0`
- `MAV_INVERT_TILT=0`
- the current working tuning block for `target_face`

For the MK15 handheld in this full mode, use:

- `rtsp://192.168.144.100:8554/stream`

This wrapper keeps the same practical bridge mode as the current working system:

- `CONTROL_API=command`
- `LIVE_CONTROL_MODE=angle-target`
- `MAV_INVERT_PAN=0`
- `MAV_INVERT_TILT=0`
- `SHOW=1` by default in full mode, so Jetson local preview and MK15 stream can both be on at the same time like the original working setup

It now also exports the same RTSP mount setting as the YOLO-only stream path, so the full wrapper and the monitoring-only wrapper use the same URL:

- `RTSP_MOUNT=/stream`

### Launch 3A: Full Explicit Working Setup

Use this when you want the full MK15 + gimbal launch with every important setting visible instead of relying on one-shot defaults.

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=1
export RTSP_PORT=8554
export RTSP_MOUNT='/stream'
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=60
export CAMERA_FPS_D=1
export TARGET_CLASS_ID=0
export SELECTION='center'
export PAN_GAIN=0.91
export TILT_GAIN=0.83
export DEADZONE=0.048
export SMOOTH_ALPHA=0.39
export FAST_SMOOTH_ALPHA=0.81
export FAST_ERROR_ZONE=0.15
export COMMAND_BOOST_ZONE=0.095
export MIN_ACTIVE_COMMAND=0.20
export RESPONSE_GAMMA=0.63
export PAN_FEEDFORWARD_GAIN=0.18
export TILT_FEEDFORWARD_GAIN=0.12
export FEEDFORWARD_ALPHA=0.40
export FEEDFORWARD_LIMIT=0.14
export FEEDFORWARD_ACTIVATION_ZONE=0.07
export MAX_YAW_RATE_DPS=95
export MAX_PITCH_RATE_DPS=60
export CONTROL_API=command
export LIVE_CONTROL_MODE=angle-target
export MAV_INVERT_PAN=0
export MAV_INVERT_TILT=0
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export BRIDGE_STATE_FILE='/tmp/profile640fp16_archery_target_live_bridge.jsonl'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

This explicit block is the same working setup as the one-shot launcher above, just written out fully.

If you want the higher-speed IMX412 camera mode later, change only this block before launching:

```bash
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=240
export CAMERA_FPS_D=1
```

### Launch 3B: Flight/Test Run With Clean Failure-Frame Recording

Use this when you want failure-frame extraction for training data during a real tracking test. It runs the same full MK15 + gimbal application, and adds a second local MKV from the camera-side branch before inference, tracker, `nvdsosd`, RTSP, and debug drawing, so the saved frames are clean dataset material.

```bash
cd /home/saturnzzz/ultralytics
RAW_RECORD_ENABLE=1 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

It uses the same current camera defaults unless you override them:

- `SENSOR_ID=0`
- `CAMERA_WIDTH=2028`
- `CAMERA_HEIGHT=1112`
- `CAMERA_FPS_N=60`
- `CAMERA_FPS_D=1`

If `RAW_RECORD_FILE` is not set, the launcher creates:

- `recordings/archery_target_clean_YYYYMMDD-HHMMSS.mkv`

After the run, extract failure frames from the newest clean recording and newest bridge JSONL log:

```bash
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh
```

The extractor writes:

- `recordings/archery_target_clean_YYYYMMDD-HHMMSS_failure_frames/`
- `recordings/archery_target_clean_YYYYMMDD-HHMMSS_failure_frames.zip`

Use that zip for labeling. Filenames keep the bridge `frame_idx`, for example `miss_001_f1234_p2.jpg`.

Optional extractor overrides:

```bash
MIN_MISS_FRAMES=10 FRAMES_PER_SEGMENT=5 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh
```

If you want to force a specific recording or log:

```bash
VIDEO=/path/to/archery_target_clean_YYYYMMDD-HHMMSS.mkv \
BRIDGE_LOG=/tmp/profile640fp16_archery_target_mk15_live_bridge.jsonl \
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh
```

If you want even more stream quality in full mode, raise bitrate for that run:

```bash
cd /home/saturnzzz/ultralytics
BITRATE=10000000 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

Default bridge log for this profile:

```text
/tmp/profile640fp16_archery_target_mk15_live_bridge.jsonl
```

## Rebuild

If you change the code in this profile:

```bash
cd /home/saturnzzz/ultralytics
make -C /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target clean all
```

## Stop

To stop running RTSP/DeepStream processes:

```bash
pkill -f deepstream_yolo26_rtsp_target_control
pkill -x csi_h264_rtsp_server
```

## Troubleshooting

If MK15 still does not show video correctly:

1. Stop the current launcher.
2. Start it again from this README.
3. Reopen `Camera A` in the FPV app with:
   - `rtsp://192.168.144.100:8554/stream`

If MK15 says `connected` but the screen is blank, first make sure you are using the current rebuilt binary from this profile. The fixed full launcher now publishes a real `/stream` RTSP payloader path, and that was the main blank-screen issue here.

If you want to check the stream locally on the Jetson:

```bash
ffprobe -rtsp_transport tcp -v error -select_streams v:0 -show_entries stream=codec_name,width,height,r_frame_rate -of default=nw=1 rtsp://127.0.0.1:8554/stream
```

Or extract one frame locally:

```bash
ffmpeg -y -rtsp_transport tcp -i rtsp://127.0.0.1:8554/stream -frames:v 1 /tmp/archery_rtsp_frame_check.jpg
```
