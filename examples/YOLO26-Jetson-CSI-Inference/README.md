# THORNG Jetson CSI + PX4 + SIYI Notes

This folder contains the working THORNG scripts for:
- Prototype 1: Ultralytics CSI tracking/follow/control
- Prototype 2: separate DeepStream-based path
- PX4 / SIYI gimbal control through:
  - `Jetson -> PX4 -> SIYI`

Prototype 1 and `prototype_v2` are intentionally kept separate.

## 1. Script Map

| File | What it does |
| --- | --- |
| `jetson_csi_inference.py` | basic object detection from CSI/video/webcam |
| `jetson_csi_tracking.py` | multi-object tracking |
| `jetson_csi_target_follow.py` | select one target and keep following it |
| `jetson_csi_target_control.py` | convert target error into virtual `pan_cmd` / `tilt_cmd` |
| `jetson_csi_px4_siyi_bridge.py` | prototype 1 live CSI bridge to `PX4 -> SIYI` |
| `px4_siyi_live_bridge.py` | isolated MAVLink backend for the prototype 1 live bridge |
| `jetson_px4_siyi_gimbal_test.py` | simple manual gimbal test without CSI/YOLO |
| `px4_siyi_bench_bridge.py` | isolated MAVLink backend for the simple bench test |
| `prototype_v2/` | separate DeepStream-based prototype |

Main launchers:
- `run_xavier_inference.sh`
- `run_xavier_tracking.sh`
- `run_xavier_target_follow.sh`
- `run_xavier_target_control.sh`
- `run_xavier_px4_siyi_bridge.sh`
- `run_xavier_px4_siyi_gimbal_test.sh`
- `prototype_v2/run_deepstream_yolo26_csi_app.sh`
- `prototype_v2/run_deepstream_yolo26_rtsp.sh`
- `prototype_v2/run_deepstream_yolo26_rtsp_preview.sh`
- `prototype_v2/run_deepstream_yolo26_metadata_dump.sh`
- `prototype_v2/run_deepstream_yolo26_target_control.sh`
- `prototype_v2/run_deepstream_yolo26_rtsp_target_control.sh`
- `prototype_v2/run_deepstream_yolo26_rtsp_target_control_preview.sh`
- `prototype_v2/run_deepstream_px4_siyi_bridge.sh`

## 2. Common Notes

- `SENSOR_ID=0` means `cam0`
- `DEVICE=0` means GPU
- `TRACK_CLASSES='0'` means person-only
- for PX4 / SIYI scripts, `/dev/ttyUSB0` must be free
- if `MAVProxy` is using `/dev/ttyUSB0`, stop it first
- if `/dev/ttyUSB0` says permission denied, use a shell with `dialout` access

## 3. Prototype 1: Ultralytics CSI Path

### 3.1 Object Detection

#### Basic

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

#### Faster Tuned Version

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export FRAMERATE=60
export CAPTURE_WIDTH=1920
export CAPTURE_HEIGHT=1080
export DISPLAY_WIDTH=1280
export DISPLAY_HEIGHT=720
export IMGSZ=512
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

Why this is faster:
- `FRAMERATE=60` uses the faster camera mode
- `IMGSZ=512` reduces model workload
- `DISPLAY_WIDTH=1280` and `DISPLAY_HEIGHT=720` reduce display overhead
- `DEVICE=0` keeps inference on GPU

### 3.2 Tracking

#### Live Tracking on `cam0`

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_tracking.sh
```

#### Person-Only Tracking

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_tracking.sh
```

#### Save to File Instead of Live Window

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=0
export SAVE=1
export TRACKER='bytetrack.yaml'
export MAX_FRAMES=300
export OUTPUT='/tmp/csi_tracking_run.mp4'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_tracking.sh
```

### 3.3 Follow Target

#### Live Person-Follow on `cam0`

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export STATE_FILE='/tmp/target_follow_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_follow.sh
```

#### Save Person-Follow Result

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=0
export SAVE=1
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export STATE_FILE='/tmp/target_follow_state.jsonl'
export OUTPUT='/tmp/target_follow_output.mp4'
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_follow.sh
```

#### Show All Tracks, But Highlight the Selected Target

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export SHOW_ALL_TRACKS=1
export STATE_FILE='/tmp/target_follow_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_follow.sh
```

#### Choose Largest Person

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='largest'
export STATE_FILE='/tmp/target_follow_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_follow.sh
```

#### Manual ID Follow

Example: follow `id:89`

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='manual-id'
export TARGET_ID=89
export STATE_FILE='/tmp/target_follow_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_follow.sh
```

### 3.4 Virtual Control Preview

#### Live Virtual `pan_cmd` / `tilt_cmd`

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACK_CLASSES='0'
export SELECTION='center'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_control.sh
```

#### Live Preview With JSONL Output

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACK_CLASSES='0'
export SELECTION='center'
export STATE_FILE='/tmp/target_control_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_target_control.sh
```

### 3.5 Simple PX4 -> SIYI Gimbal Test

This is for bench control without CSI or YOLO.

Defaults used by the launcher:
- `CONTROL_API='attitude'`
- `RESEND_HZ=20`
- `MAV_SOURCE_SYSTEM=42`
- `MAV_SOURCE_COMPONENT=191`
- `MAV_TARGET_COMPONENT=154`

#### Exact Angle Command, No Return

```bash
cd /home/aarl/ultralytics
export MODE='angle'
export PITCH=-10
export YAW=15
export HOLD_SECONDS=3
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_gimbal_test.sh
```

#### Exact Angle Command, Then Return to Neutral

```bash
cd /home/aarl/ultralytics
export MODE='angle'
export PITCH=-45
export YAW=45
export HOLD_SECONDS=3
export NEUTRAL_AFTER=1
export NEUTRAL_PITCH=0
export NEUTRAL_YAW=0
export NEUTRAL_HOLD_SECONDS=2
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_gimbal_test.sh
```

#### Experimental Rate Mode

```bash
cd /home/aarl/ultralytics
export MODE='rate'
export PITCH_RATE=-5
export YAW_RATE=10
export HOLD_SECONDS=2
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_gimbal_test.sh
```

Note:
- exact angle commands are the known-good bench path
- rate mode is for testing only

### 3.6 Live PX4 -> SIYI Bridge

This is the real Setup 3 path:

```text
Jetson -> PX4 -> SIYI
```

Current launcher defaults already use the working live bridge control identity:
- `MAV_SOURCE_SYSTEM=42`
- `MAV_SOURCE_COMPONENT=191`
- `MAV_TARGET_SYSTEM=1`
- `MAV_TARGET_COMPONENT=154`
- `GIMBAL_DEVICE_ID=154`
- `CONTROL_API='attitude'`
- `LIVE_CONTROL_MODE='angle-target'`

#### Live CSI Tracking + Real Gimbal Commands

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_bridge.sh
```

#### Same Bridge, But Save JSONL State Too

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export STATE_FILE='/tmp/px4_bridge_state.jsonl'
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_bridge.sh
```

#### Faster Live Response

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export PAN_GAIN=0.85
export TILT_GAIN=0.80
export DEADZONE=0.02
export SMOOTH_ALPHA=0.50
export MAX_YAW_RATE_DPS=90
export MAX_PITCH_RATE_DPS=60
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_bridge.sh
```

#### Aggressive

```bash
cd /home/aarl/ultralytics
export SOURCE='csi'
export SENSOR_ID=0
export DEVICE=0
export SHOW=1
export SAVE=0
export TRACKER='bytetrack.yaml'
export TRACK_CLASSES='0'
export SELECTION='center'
export PAN_GAIN=1.00
export TILT_GAIN=0.90
export DEADZONE=0.01
export SMOOTH_ALPHA=0.65
export MAX_YAW_RATE_DPS=120
export MAX_PITCH_RATE_DPS=80
unset OUTPUT
unset MAX_FRAMES
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/run_xavier_px4_siyi_bridge.sh
```

## 4. Prototype V2: DeepStream Path

This is the separate DeepStream-based prototype for the future optimized architecture.

Current `prototype_v2` state:
- separate from prototype 1
- local native `320x320` model artifacts under `prototype_v2/model/`
- RTSP/video branch
- metadata-only control branch
- combined RTSP + control-preview pipeline
- PX4 / SIYI bridge
- real physical SIYI motion already proven

Folder:
- `/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2`

### 4.1 Prepare Prototype V2 Model

```bash
cd /home/aarl/ultralytics
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/prepare_prototype_v2_model.sh
```

What this does:
- exports a local native `320x320` `yolo26n` ONNX for `prototype_v2`
- writes local labels into `prototype_v2/model/`
- clears stale local TensorRT engines so DeepStream can rebuild from the new ONNX

### 4.2 DeepStream CSI + YOLO26 Display

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=1
export RTSP_ENABLE=0
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_csi_app.sh
```

### 4.3 DeepStream CSI + YOLO26 Metadata Dump

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export MAX_FRAMES=300
export STATE_FILE='/tmp/deepstream_yolo26_metadata.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_metadata_dump.sh
```

### 4.4 DeepStream CSI + YOLO26 RTSP Server

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export RTSP_PORT=8554
export UDP_PORT=5400
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_rtsp.sh
```

RTSP URL:
- `rtsp://localhost:8554/ds-test`
- from another machine, replace `localhost` with the Jetson IP

### 4.5 DeepStream RTSP Local Preview

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export RTSP_PORT=8554
export RTSP_VIEWER_LATENCY_MS=80
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_rtsp_preview.sh
```

Notes:
- this starts the RTSP server and opens a low-latency local GStreamer viewer
- use this local preview flow instead of VLC on this machine

### 4.6 DeepStream Tracked-Metadata Control Preview

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export MAX_FRAMES=300
export STATE_FILE='/tmp/deepstream_yolo26_target_control.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_target_control.sh
```

What this does:
- uses DeepStream tracker metadata instead of exporting full frames to control
- selects one target from tracked metadata
- computes `dx_norm`, `dy_norm`, and virtual `pan_cmd` / `tilt_cmd`
- writes control-preview JSONL to `STATE_FILE`

### 4.7 DeepStream Combined RTSP + Control Preview

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export RTSP_PORT=8554
export UDP_PORT=5400
export MAX_FRAMES=300
export STATE_FILE='/tmp/deepstream_yolo26_rtsp_target_control.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_rtsp_target_control.sh
```

What this does:
- runs one `prototype_v2` DeepStream pipeline
- keeps the RTSP/video branch alive at the same time as the metadata-only control-preview branch
- writes target-control JSONL while the RTSP stream is live
- stream overlay shows:
  - selected target highlighted in green
  - center crosshair
  - error line
  - small control/status text

### 4.8 DeepStream Combined RTSP + Control Low-Latency Local Preview

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export RTSP_PORT=8554
export UDP_PORT=5400
export RTSP_VIEWER_LATENCY_MS=80
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_yolo26_rtsp_target_control_preview.sh
```

Notes:
- this is the preferred local stream check for the combined `prototype_v2` path
- for the closest V1-like feel, you can also run the combined app with:
  - `SHOW=1`

### 4.9 DeepStream PX4 -> SIYI Bridge

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export RTSP_PORT=8554
export UDP_PORT=5400
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export MAX_FRAMES=0
export BRIDGE_STATE_FILE='/tmp/prototype_v2_px4_bridge_state.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_px4_siyi_bridge.sh
```

What this does:
- starts the combined `prototype_v2` DeepStream app in the background
- consumes the metadata-only control JSONL from that app
- maps `pan_cmd` / `tilt_cmd` into the proven PX4 gimbal-manager path
- sends the result through:
  - `Jetson -> PX4 -> SIYI`

Notes:
- this is the `prototype_v2` bridge path, separate from prototype 1
- for a no-hardware-control smoke run, add:
  - `export DRY_RUN_MAVLINK=1`
- this path now physically moves the SIYI gimbal
- the control law is now shaped for:
  - fast continuous recentering when the target is clearly off-center
  - calmer settling near image center

### 4.10 Prototype V2 Saved Presets

These two shaped-controller launch commands are the only kept `prototype_v2` bridge presets now.
Older pre-shaped-control preset wording is removed.

#### Decisive Recenter Preset

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=1
export RTSP_ENABLE=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export PAN_GAIN=0.85
export TILT_GAIN=0.80
export DEADZONE=0.05
export SMOOTH_ALPHA=0.40
export FAST_SMOOTH_ALPHA=0.75
export FAST_ERROR_ZONE=0.18
export COMMAND_BOOST_ZONE=0.10
export MIN_ACTIVE_COMMAND=0.18
export RESPONSE_GAMMA=0.65
export MAX_YAW_RATE_DPS=90
export MAX_PITCH_RATE_DPS=60
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export MAX_FRAMES=0
export BRIDGE_STATE_FILE='/tmp/prototype_v2_px4_bridge_state.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_px4_siyi_bridge.sh
```

Notes:
- this is the stronger shaped-controller preset
- use this when you want the gimbal to commit harder when the target is clearly off-center
- this remains the better far-error response preset

#### Smoother Recenter Preset

```bash
cd /home/aarl/ultralytics
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export SHOW=1
export RTSP_ENABLE=0
export TARGET_CLASS_ID=0
export SELECTION='center'
export PAN_GAIN=0.80
export TILT_GAIN=0.75
export DEADZONE=0.06
export SMOOTH_ALPHA=0.32
export FAST_SMOOTH_ALPHA=0.68
export FAST_ERROR_ZONE=0.18
export COMMAND_BOOST_ZONE=0.12
export MIN_ACTIVE_COMMAND=0.14
export RESPONSE_GAMMA=0.72
export MAX_YAW_RATE_DPS=90
export MAX_PITCH_RATE_DPS=60
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export MAX_FRAMES=0
export BRIDGE_STATE_FILE='/tmp/prototype_v2_px4_bridge_state.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2/run_deepstream_px4_siyi_bridge.sh
```

Notes:
- this is the calmer shaped-controller preset
- smoother near center and in the transition zone
- weaker far-error pull than the decisive preset, so keep both

#### Control Shape Knobs

These are the new `prototype_v2` bridge/control-preview knobs used by the shaped controller:

- `FAST_SMOOTH_ALPHA`
  - faster smoothing used when error is clearly outside the center zone
- `FAST_ERROR_ZONE`
  - error magnitude where the controller starts behaving more aggressively
- `COMMAND_BOOST_ZONE`
  - small outer zone where the controller begins committing harder instead of creeping
- `MIN_ACTIVE_COMMAND`
  - minimum command used once the target is clearly off-center
- `RESPONSE_GAMMA`
  - response curve shaping; lower than `1.0` makes the controller react harder earlier

## 5. Quick Reminders

- for live CSI work, press `q` in the display window to stop
- for person-only workflows, keep `TRACK_CLASSES='0'`
- for manual follow mode, set `SELECTION='manual-id'` and `TARGET_ID=<id>`
- do not run multiple CSI launchers at the same time on the same camera

## 6. Project Progress

- Gimbal hardware baseline:
  - Setup 1 style `QGC -> PX4 -> SIYI` was already proven on bench
  - SIYI gimbal component / device id `154` is confirmed
  - SIYI control required `set_att_en=1`

- Jetson companion backbone:
  - Jetson companion serial link to PX4 is alive on `/dev/ttyUSB0`
  - MAVLink heartbeat, parameters, and traffic were confirmed from Jetson
  - current hardware backbone is:
    - `CSI camera -> Jetson`
    - `Jetson -> PX4`
    - `PX4 -> SIYI`

- Prototype 1 status:
  - CSI camera, detection, tracking, follow, and virtual control preview work
  - simple bench gimbal control works
  - live CSI bridge physically moves the SIYI gimbal
  - remaining prototype 1 work is mainly tuning quality

- Prototype 2 status:
  - separate `prototype_v2/` path exists and keeps prototype 1 untouched
  - uses local native `320x320` ONNX / TensorRT artifacts under `prototype_v2/model/`
  - detection, tracking, RTSP preview, metadata-only control preview, and combined RTSP + control preview all work
  - `prototype_v2` PX4 / SIYI bridge exists and physically moves the SIYI gimbal
  - current remaining `prototype_v2` work is tuning:
    - motion smoothness
    - center-zone twitching
    - final response-speed balance

## 7. Live Bridge Display Notes

Use the live bridge HUD to judge whether image-based control is behaving correctly.

- Main control indicators:
  - `tracked | target id:... person | tracks ... | lost ...`
    - confirms the target is visible, selected, and not currently lost
  - `vcmd pan ... tilt ...`
    - virtual control output after smoothing, deadzone, and clamp
    - positive `pan` means the target is to the right of center
    - negative `pan` means the target is to the left of center
  - `mav yaw ... pitch ...`
    - the actual gimbal angle target being sent through `Jetson -> PX4 -> SIYI`
  - `ctrl:42/191`
    - PX4 gimbal manager currently sees the Jetson component as the active primary controller
  - `err_norm x,y`
    - normalized target error from image center

- Visual overlay meaning:
  - green box = selected target box
  - green dot = selected target center
  - cyan crosshair = image center
  - yellow line = offset from image center to target center

- What it should look like when control is working properly:
  - `err_norm` moves toward `0,0`
  - `vcmd pan/tilt` gets smaller as the target gets centered
  - `mav yaw/pitch` moves in the correct direction, then settles
  - the yellow line shrinks
  - the target box stays closer to image center

- Warning signs:
  - `err_norm` stays large for a long time
  - `mav yaw` or `mav pitch` runs to a clamp/limit
  - the target remains far from center even though commands are active
  - the yellow line stays long or grows

## 8. Diagram vs Current Implementation

Current implementation:

```text
Prototype 1:
CSI -> GStreamer appsink -> Python frame -> YOLO -> tracking/follow -> control -> PX4 -> SIYI

Prototype 2:
CSI -> DeepStream -> YOLO/tracker metadata -> control branch + RTSP branch -> PX4 -> SIYI
```

Intended future optimized direction:

```text
CSI -> DeepStream YOLO -> tracker
                    -> RTSP/video branch
                    -> metadata-only control branch
```

Meaning:
- do not export full images just to control the gimbal
- keep image/video in the high-performance pipeline
- use bbox / tracker metadata for control
