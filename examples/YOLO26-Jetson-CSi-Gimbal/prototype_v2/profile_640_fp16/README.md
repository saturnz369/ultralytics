# Profile 640 FP16

This is the real `prototype_v2` `640x640` FP16 profile for the CSI camera + PX4/SIYI gimbal path:

- `/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16`

It is separate from the test-only `YOLO26-Jetson-CSI-Inference/prototype_v2` folder. Use this folder for the real gimbal profile.

## Current Status

- This profile has been ported from the old Xavier paths to this Orin NX:
  - old path base: `/home/aarl/...`
  - current path base: `/home/saturnzzz/...`
- This profile is configured for YOLO26n `640x640` FP16.
- Local model artifacts are present:
  - `model/yolo26n.pt`
  - `model/yolo26n_native_640.onnx`
  - `model/model_b1_gpu0_fp16.engine`
- The TensorRT engine was built locally on this Orin NX with DeepStream 7.1.
- DeepStream smoke run is verified:
  - cached engine deserializes successfully
  - CSI camera path runs
  - latest target metadata is published to the live shared-memory snapshot
- Local/RTSP preview is verified:
  - person detection works
  - tracking works
  - monitoring branch is live
  - default local RTSP URL is `rtsp://localhost:8554/ds-test`
- Bridge dry-run is verified:
  - latest shared-memory control snapshot is read by `deepstream_px4_siyi_bridge.py`
  - bridge state is written with `mavlink_link="dry-run"`
  - `pymavlink` and `pyserial` are installed in `/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys`
- Normal live control now uses a latest-only shared-memory handoff.
  - raw metadata JSONL is optional debug/audit logging, not a live dependency
- Real physical `PX4 -> SIYI` validation for this FP16 profile is verified.
- MK15 third-party RTSP path is verified:
  - camera-only stream works in the MK15 FPV app
  - YOLO/overlay stream works in the MK15 FPV app
  - operator steps are in `streaming/README.md`

Current practical meaning:

- Tracking-only tests do not need PX4 or the gimbal.
- Bridge dry-run does not need PX4 or the gimbal.
- Real bridge testing needs PX4/SIYI connected through the serial device, normally `/dev/ttyUSB0`.
- `profile_640_fp16` should be tuned independently from `profile_320` and `profile_640`.

## Current Hardware / Runtime

- Jetson: Orin NX
- JetPack/L4T used for the camera driver work: JetPack 6.2.1 / L4T 36.4.4
- DeepStream: `/opt/nvidia/deepstream/deepstream -> deepstream-7.1`
- CUDA: `/usr/local/cuda-12.6`
- Camera: e-con e-CAM121_CUONX / IMX412 CSI camera
- Camera connector used in the current working setup: CAM1
- FFC cable requirement from the working hardware setup: 22-pin, 0.5 mm pitch, Type A
- DeepStream-Yolo parser:
  - `/home/saturnzzz/DeepStream-Yolo/nvdsinfer_custom_impl_Yolo/libnvdsinfer_custom_impl_Yolo.so`
- Python runtime for model export and bridge:
  - `/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python`

## Quick Start

Run all commands from the repo root unless the command says otherwise:

```bash
cd /home/saturnzzz/ultralytics
```

### 1. Rebuild The App If Needed

The Makefile detects the installed DeepStream version from `/opt/nvidia/deepstream/deepstream`.

```bash
cd /home/saturnzzz/ultralytics
make -C /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16 clean all
```

### 2. Rebuild The ONNX / Engine If Needed

Only do this if the model or TensorRT engine is missing, or if you intentionally changed the model/config.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/prepare_prototype_v2_model.sh
```

Then launch DeepStream once to build `model/model_b1_gpu0_fp16.engine`:

```bash
cd /home/saturnzzz/ultralytics
export SHOW=0
export RTSP_ENABLE=0
export MAX_FRAMES=30
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_yolo26_rtsp_target_control.sh
```

The current Orin NX already has the engine built, so normal launches should deserialize it immediately.

### 3. Tracking / Preview Only

Use this before the gimbal is connected. It does not touch PX4 or SIYI.

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=1
export RTSP_PORT=8554
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export TARGET_CLASS_ID=0
export SELECTION='center'
export MAX_FRAMES=0
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_yolo26_rtsp_target_control.sh
```

Optional debug raw metadata log for this run:

```bash
export STATE_FILE='/tmp/profile640fp16_tracking_preview.jsonl'
```

RTSP URL:

```text
rtsp://localhost:8554/ds-test
```

### 4. Bridge Dry-Run

Use this before PX4/SIYI is connected. It starts DeepStream, reads the latest shared-memory metadata snapshot, computes MAVLink gimbal commands, but does not send them to hardware.

```bash
cd /home/saturnzzz/ultralytics
export DRY_RUN_MAVLINK=1
export PRINT_STATE=1
export SHOW=0
export RTSP_ENABLE=0
export MAX_FRAMES=90
export SENSOR_ID=0
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=720
export CAMERA_FPS_N=30
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
export BRIDGE_STATE_FILE='/tmp/profile640fp16_bridge_state_dryrun.jsonl'
export DEEPSTREAM_LOG_FILE='/tmp/profile640fp16_bridge_deepstream_dryrun.log'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_px4_siyi_bridge.sh
```

Optional raw metadata audit log for this run:

```bash
export METADATA_STATE_FILE='/tmp/profile640fp16_bridge_metadata_dryrun.jsonl'
```

Expected dry-run proof in `BRIDGE_STATE_FILE`:

```text
"mavlink_link":"dry-run"
"last_configure_ack":"DRY_RUN"
```

### 5. Real PX4 / SIYI Bridge

Use this only after PX4/SIYI hardware is connected and the serial device is correct.

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=0
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
export BRIDGE_STATE_FILE='/tmp/profile640fp16_live_bridge.jsonl'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_px4_siyi_bridge.sh
```

For real bridge testing, `RTSP_ENABLE=0` keeps the load lower. Change it to `RTSP_ENABLE=1` only when you also need the RTSP monitoring stream.

Optional raw metadata audit log for this run:

```bash
export METADATA_STATE_FILE='/tmp/profile640fp16_live_metadata.jsonl'
```

Current recommended bridge mode for the current gimbal + camera mount:

- `CONTROL_API=command`
- `LIVE_CONTROL_MODE=angle-target`
- `MAV_INVERT_PAN=0`
- `MAV_INVERT_TILT=0`

This keeps the old tuned tracking values, but uses the current working MAVLink bridge behavior for the new gimbal/mount setup.

### Runtime Timing Fields

When `BRIDGE_STATE_FILE` is enabled, the bridge log can show separation/latency metrics such as:

- `ds_to_py_ms`
  - live metadata handoff latency from DeepStream publish to Python read
- `mav_send_to_feedback_ms`
  - command send to observed PX4/gimbal feedback timing when feedback is available
- `display_frame_lag`
  - how many frames the preview branch trails the current control frame
- `rtsp_frame_lag`
  - how many frames the RTSP branch trails the current control frame

This is the main proof that the control branch can stay fresher than the monitoring branch.

## Current Clean Runtime Architecture

The current runtime is now much closer to the intended `prototype_v2` design.

Actual live flow:

```text
CSI camera
-> DeepStream / YOLO / tracker
-> tracker src pad-probe
-> latest-only shared-memory metadata snapshot
-> separate Python bridge loop
-> MAVLink / PX4 gimbal-manager
-> SIYI gimbal
```

At the same time, the monitoring branch stays on the video side:

```text
tracker output
-> leaky video queue
-> nvvidconv
-> nvosd
-> tee
-> display queue -> local preview
-> rtsp queue -> H.264 -> RTSP
```

### What Is Cleanly Separated Now

- Live gimbal commands are **not** sent inside the DeepStream callback.
- The Python bridge reads only the **latest** metadata snapshot.
- Slow preview or RTSP no longer causes the bridge to replay old metadata line by line.
- The display and RTSP paths sit behind leaky queues, so they are allowed to fall behind and drop frames.
- `RTSP_ENABLE=0` now really disables the RTSP encode/stream branch.

### What This Means In Practice

- The control branch can stay fresher than the monitoring branch.
- The MK15 stream or local preview may be a few frames behind, and that is acceptable.
- The important goal is that the gimbal follows fresh target metadata, not old display frames.

The timing fields above are the proof for this:

- `ds_to_py_ms` shows metadata handoff latency into the Python bridge.
- `display_frame_lag` shows how far preview trails the current control frame.
- `rtsp_frame_lag` shows how far RTSP trails the current control frame.
- `mav_send_to_feedback_ms` shows the observed command-to-feedback timing when PX4/gimbal feedback is available.

### Residual Weak Point

The architecture is cleaner now, but it is not the absolute final form yet.

The main remaining weak point is the DeepStream tracker callback:

- it still does target selection and raw target-state extraction in C
- the common video path still uses one shared OSD stage before the final display/RTSP sink split
- if debug metadata logging is enabled with flush, it can still do per-frame file sync work

So the system is now in a good practical state, but the most purist end-state would be:

- C tracker probe: extract/select latest target metadata, publish it, return immediately
- Python bridge loop: read latest snapshot, compute final smoothed control output, send MAVLink
- Video branch: preview / RTSP / recording only, with full sink isolation if a stable branch-local OSD path is available

## How This Profile Works

This profile follows the `prototype_v2` architecture from the diagram, but here it is written in engineering terms.

### System-Level Flow

```text
CSI camera
-> GStreamer / DeepStream video pipeline
-> primary detector (YOLO)
-> multi-object tracker
-> target-selection + raw metadata publish stage
-> split into 2 logical branches:
   1. video / monitoring branch
   2. metadata / control branch
```

### Detailed Pipeline Meaning

1. `CSI camera -> DeepStream pipeline`
   - the image enters through the Jetson CSI camera path
   - in practice this is the video source for the full pipeline
   - from the architecture point of view, this is the image path that should stay inside the high-performance pipeline

2. `DeepStream pipeline -> primary inference stage`
   - YOLO is the primary object detector
   - this stage produces detections such as:
     - class id
     - confidence
     - bounding box
   - in DeepStream terms, this is the detector / inference stage that feeds object metadata forward

3. `primary inference -> tracker`
   - tracking assigns persistent IDs to detections across frames
   - so the system is not only doing frame-by-frame detection
   - it is maintaining tracked targets over time
   - this is what makes single-target follow possible

4. `tracker -> target-selection + raw metadata stage`
   - one tracked target is selected for control
   - from that selected target, the profile computes the control-side quantities:
     - target center in image coordinates
     - normalized image-plane error:
       - `dx_norm`
       - `dy_norm`
   - that raw target state is published as the latest snapshot for the Python bridge
   - the final smoothed / shaped command is computed in the Python bridge, not in the DeepStream C callback

### The Important Branch Split

After detection, tracking, and target-control calculation, the profile is logically split into 2 branches.

```text
DeepStream detect/track/control source
-> Branch A: video / stream / monitoring
-> Branch B: metadata / control
```

#### Branch A: Video / Monitoring Branch

```text
DeepStream app
-> overlay / OSD / local preview / RTSP-style monitoring path
```

- this branch is for the operator / developer to see the result
- it is where you visually inspect:
  - detections
  - tracking IDs
  - selected target
  - crosshair / error line / control overlay
- this branch is **not** the control loop itself

#### Branch B: Metadata / Control Branch

```text
tracked target metadata
-> target position / bbox center
-> normalized image-center error
-> latest shared-memory snapshot
-> PX4 bridge
-> filtered / shaped pan_cmd / tilt_cmd
-> PX4 MAVLink gimbal-manager path
-> SIYI gimbal
```

- this is the real control branch
- this branch should use lightweight target metadata, not full exported video frames
- the important control variables are derived from tracked metadata inside the separate Python bridge loop, not from the RTSP stream and not from a second full-frame CPU image path

So the real control chain is:

```text
detect
-> track
-> choose target
-> compute dx_norm / dy_norm
-> publish latest metadata snapshot
-> Python bridge computes pan_cmd / tilt_cmd
-> map into MAVLink gimbal commands
-> send through PX4
-> move SIYI
```

### Why This Matches the Diagram

This is the key architectural idea from the earlier diagram discussion:

- the **image path** should stay in the DeepStream / GPU-oriented pipeline
- the **video branch** exists for preview / streaming / monitoring
- the **control branch** should consume only metadata-level information
- the gimbal should follow the metadata/control branch, not the preview stream itself

In other words:

- full-frame video is for display / streaming
- metadata is for control

That is why `prototype_v2` is the future optimized direction:

- it separates monitoring from control
- it avoids using full exported image frames as the main control signal
- it uses target metadata and image-plane error as the control input to `PX4 -> SIYI`

## What Each File Does

### Core Code

- `deepstream_yolo26_rtsp_target_control.c`
  - main DeepStream app for this profile
  - handles CSI input, YOLO detection, tracking, RTSP branch, and metadata-control output

- `deepstream_px4_siyi_bridge.py`
  - live bridge from the DeepStream control output to PX4 / SIYI
  - reads metadata-control state and converts it into MAVLink gimbal commands

- `px4_siyi_live_bridge.py`
  - low-level MAVLink helper used by the live bridge
  - handles PX4 connection, control claim, and gimbal pitch/yaw sends

### Launchers

- `run_deepstream_yolo26_rtsp_target_control.sh`
  - runs the main DeepStream app

- `run_deepstream_yolo26_rtsp_target_control_preview.sh`
  - runs the same main DeepStream app and opens local preview

- `run_deepstream_px4_siyi_bridge.sh`
  - runs the real live path:
    - `DeepStream -> metadata control -> PX4 -> SIYI`

### Model / Config

- `model/labels.txt`
  - class labels used by the model

- `model/yolo26n.pt`
  - local YOLO26n PyTorch weight used by `prepare_prototype_v2_model.sh`

- `model/yolo26n_native_640.onnx`
  - native `640x640` ONNX model for this profile

- `model/model_b1_gpu0_fp16.engine`
  - active TensorRT engine built on this Orin NX and used by this profile

- `config/config_infer_primary_yolo26.txt`
  - DeepStream infer config for this profile

- `config/tracker_config.txt`
  - main tracker config used by the app

- `config/config_tracker_NvDCF_perf.yml`
  - tracker support/tuning file referenced by the tracker config

### Build / Support

- `Makefile`
  - builds the DeepStream app

- `prepare_prototype_v2_model.sh`
  - rebuilds/prepares the local model artifacts for this profile

## Relationship Between Files

The main real path is:

```text
deepstream_yolo26_rtsp_target_control.c
-> run_deepstream_yolo26_rtsp_target_control.sh
-> deepstream_px4_siyi_bridge.py
-> px4_siyi_live_bridge.py
-> PX4
-> SIYI
```

For local viewing only:

```text
run_deepstream_yolo26_rtsp_target_control_preview.sh
-> run_deepstream_yolo26_rtsp_target_control.sh
-> deepstream_yolo26_rtsp_target_control.c
```

So the important idea is:

- the C file is the DeepStream detect/track/control source
- the first Python file is the live bridge
- the second Python file is the MAVLink backend
- the `.sh` files are only launch helpers around those pieces

## Controller Preset

The launch scripts already default to the current `640x640` FP16 feedforward recenter preset:

```bash
TARGET_CLASS_ID=0
SELECTION=center
PAN_GAIN=0.91
TILT_GAIN=0.83
DEADZONE=0.048
SMOOTH_ALPHA=0.39
FAST_SMOOTH_ALPHA=0.81
FAST_ERROR_ZONE=0.15
COMMAND_BOOST_ZONE=0.095
MIN_ACTIVE_COMMAND=0.20
RESPONSE_GAMMA=0.63
PAN_FEEDFORWARD_GAIN=0.18
TILT_FEEDFORWARD_GAIN=0.12
FEEDFORWARD_ALPHA=0.40
FEEDFORWARD_LIMIT=0.14
FEEDFORWARD_ACTIVATION_ZONE=0.07
MAX_YAW_RATE_DPS=95
MAX_PITCH_RATE_DPS=60
```

Use the Quick Start commands above first. Only export these variables manually when tuning or intentionally overriding the defaults.

## Tuning Guide

These variables fall into 4 groups.

### 1. Not tuning

These are runtime or link settings, not behavior tuning:

- `SHOW`
- `RTSP_ENABLE`
- `TARGET_CLASS_ID`
- `SELECTION`
- `SERIAL_DEVICE`
- `SERIAL_BAUD`
- `MAV_SOURCE_SYSTEM`
- `MAV_SOURCE_COMPONENT`
- `MAV_TARGET_SYSTEM`
- `MAV_TARGET_COMPONENT`
- `GIMBAL_DEVICE_ID`

### 2. Core controller tuning

These shape the base command from bbox error:

- `PAN_GAIN=0.91`
  Horizontal base gain. Bigger = stronger yaw response.
- `TILT_GAIN=0.83`
  Vertical base gain. Bigger = stronger pitch response.
- `DEADZONE=0.048`
  Ignore very small error near center. Bigger = calmer center, but slower to react.
- `SMOOTH_ALPHA=0.39`
  Base smoothing for small errors. Bigger = follows new error faster.
- `FAST_SMOOTH_ALPHA=0.81`
  Stronger smoothing factor used when error is larger. Bigger = reacts faster when off-center.
- `FAST_ERROR_ZONE=0.15`
  Error size where the controller transitions toward fast smoothing. Smaller = earlier fast reaction.
- `COMMAND_BOOST_ZONE=0.095`
  Below this, command is mostly linear. Above this, boost shaping starts.
- `MIN_ACTIVE_COMMAND=0.20`
  Minimum command once boost shaping is active. Helps prevent creeping.
- `RESPONSE_GAMMA=0.63`
  Shapes how aggressively command rises after boost starts. Lower = more aggressive earlier. Higher = softer.

### 3. Feedforward tuning

This is the extra “early pull” layer. It adds command when error is already growing in the same direction:

- `PAN_FEEDFORWARD_GAIN=0.18`
  Horizontal feedforward strength. Bigger = more early yaw pull.
- `TILT_FEEDFORWARD_GAIN=0.12`
  Vertical feedforward strength.
- `FEEDFORWARD_ALPHA=0.40`
  Smoothing on error-rate estimate. Bigger = reacts faster to motion, but can get noisier.
- `FEEDFORWARD_LIMIT=0.14`
  Maximum extra feedforward command. Bigger = more punch, but easier to overtune.
- `FEEDFORWARD_ACTIVATION_ZONE=0.07`
  Feedforward only activates when error magnitude is at least this big. Smaller = earlier activation, but can wake up near-center twitch.

### 4. Output caps

- `MAX_YAW_RATE_DPS=95`
  Max yaw rate the bridge will command.
- `MAX_PITCH_RATE_DPS=60`
  Max pitch rate the bridge will command.

These are caps, not the main “feel” knobs.

### Real tuning elements

The main behavior knobs are:

- `PAN_GAIN`
- `TILT_GAIN`
- `DEADZONE`
- `SMOOTH_ALPHA`
- `FAST_SMOOTH_ALPHA`
- `FAST_ERROR_ZONE`
- `COMMAND_BOOST_ZONE`
- `MIN_ACTIVE_COMMAND`
- `RESPONSE_GAMMA`
- `PAN_FEEDFORWARD_GAIN`
- `TILT_FEEDFORWARD_GAIN`
- `FEEDFORWARD_ALPHA`
- `FEEDFORWARD_LIMIT`
- `FEEDFORWARD_ACTIVATION_ZONE`
- `MAX_YAW_RATE_DPS`
- `MAX_PITCH_RATE_DPS`

### Best tuning order

If tuning later, adjust in this order:

1. `PAN_FEEDFORWARD_GAIN`
2. `FEEDFORWARD_ACTIVATION_ZONE`
3. `FEEDFORWARD_LIMIT`
4. `PAN_GAIN`
5. `DEADZONE`
6. `FAST_SMOOTH_ALPHA`
7. `FAST_ERROR_ZONE`

This order is best for the current goal:

- earlier reaction
- stronger pull
- still calm near center

## Code Proof Map

This is the current code-level shape of the real runtime.

`CSI camera -> DeepStream pipeline`
- The camera source is `nvarguscamerasrc` inside `deepstream_yolo26_rtsp_target_control.c`.
- The current practical common video chain is:
  - `nvarguscamerasrc -> capsfilter -> nvvideoconvert -> nvstreammux -> nvinfer -> nvtracker -> video_queue -> nvvideoconvert -> nvdsosd -> tee`

`DeepStream tracker -> raw latest metadata`
- The tracker output probe in `deepstream_yolo26_rtsp_target_control.c` reads `NvDsObjectMeta`, selects one target, and computes raw `dx_norm / dy_norm`.
- That probe now publishes the latest target snapshot through shared memory.
- Optional metadata JSONL is only debug/audit output now; it is not the live control dependency.

`Latest metadata -> Python bridge -> final control output`
- `deepstream_px4_siyi_bridge.py` reads only the newest shared-memory snapshot.
- The Python bridge now owns the filtered/smoothed control shaping:
  - deadzone
  - smoothing
  - command boost / gamma shaping
  - feedforward
- That bridge then maps the final `pan_cmd / tilt_cmd` into yaw/pitch motion and sends MAVLink through `px4_siyi_live_bridge.py`.

`Video / monitoring branch`
- The video branch stays in DeepStream.
- Overlay work is video-side only.
- After the common OSD stage, a real `tee` splits to:
  - local preview
  - RTSP encode / pay / sink

So the real live control chain is now:

```text
NvDsObjectMeta
-> target selection
-> dx_norm / dy_norm
-> latest shared-memory snapshot
-> Python bridge computes pan_cmd / tilt_cmd
-> MAVLink
-> PX4
-> SIYI
```

That is the current code-level proof that:

- the gimbal does not follow the RTSP video
- the bridge does not replay old metadata line by line
- the final control shaping is outside the DeepStream callback
