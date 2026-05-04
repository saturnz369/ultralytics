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
  - JSONL metadata/control output is written
- Local/RTSP preview is verified:
  - person detection works
  - tracking works
  - control metadata is live
  - default local RTSP URL is `rtsp://localhost:8554/ds-test`
- Bridge dry-run is verified:
  - metadata/control output is read by `deepstream_px4_siyi_bridge.py`
  - bridge state is written with `mavlink_link="dry-run"`
  - `pymavlink` and `pyserial` are installed in `/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys`
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
export STATE_FILE='/tmp/profile640fp16_tracking_preview.jsonl'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_yolo26_rtsp_target_control.sh
```

RTSP URL:

```text
rtsp://localhost:8554/ds-test
```

### 4. Bridge Dry-Run

Use this before PX4/SIYI is connected. It starts DeepStream, reads the metadata/control JSONL, computes MAVLink gimbal commands, but does not send them to hardware.

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
export METADATA_STATE_FILE='/tmp/profile640fp16_bridge_metadata_dryrun.jsonl'
export BRIDGE_STATE_FILE='/tmp/profile640fp16_bridge_state_dryrun.jsonl'
export DEEPSTREAM_LOG_FILE='/tmp/profile640fp16_bridge_deepstream_dryrun.log'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_px4_siyi_bridge.sh
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
export METADATA_STATE_FILE='/tmp/profile640fp16_live_metadata.jsonl'
export BRIDGE_STATE_FILE='/tmp/profile640fp16_live_bridge.jsonl'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_px4_siyi_bridge.sh
```

For real bridge testing, `RTSP_ENABLE=0` keeps the load lower. Change it to `RTSP_ENABLE=1` only when you also need the RTSP monitoring stream.

Current recommended bridge mode for the current gimbal + camera mount:

- `CONTROL_API=command`
- `LIVE_CONTROL_MODE=angle-target`
- `MAV_INVERT_PAN=0`
- `MAV_INVERT_TILT=0`

This keeps the old tuned tracking values, but uses the current working MAVLink bridge behavior for the new gimbal/mount setup.

## How This Profile Works

This profile follows the `prototype_v2` architecture from the diagram, but here it is written in engineering terms.

### System-Level Flow

```text
CSI camera
-> GStreamer / DeepStream video pipeline
-> primary detector (YOLO)
-> multi-object tracker
-> target-selection + control-law stage
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

4. `tracker -> target-selection + control stage`
   - one tracked target is selected for control
   - from that selected target, the profile computes the control-side quantities:
     - target center in image coordinates
     - normalized image-plane error:
       - `dx_norm`
       - `dy_norm`
     - filtered / shaped control output:
       - `pan_cmd`
       - `tilt_cmd`

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
-> virtual pan_cmd / tilt_cmd
-> PX4 bridge
-> PX4 MAVLink gimbal-manager path
-> SIYI gimbal
```

- this is the real control branch
- this branch should use lightweight target metadata, not full exported video frames
- the important control variables are derived from tracked metadata, not from a second full-frame CPU-side image-processing path

So the real control chain is:

```text
detect
-> track
-> choose target
-> compute dx_norm / dy_norm
-> compute pan_cmd / tilt_cmd
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

`CSI camera -> DeepStream pipeline`
- The camera source is `nvarguscamerasrc` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1112).

- The main pipeline chain is built as:
  - `nvarguscamerasrc -> capsfilter -> nvvideoconvert -> nvstreammux -> nvinfer -> nvtracker -> nvvideoconvert -> nvdsosd -> tee`

- You can see that chain in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1213) and [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1225).



`DeepStream -> YOLO detection`
- The detector is `nvinfer` as `pgie` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1116).

- Its config file is loaded with `config-file-path` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1167).



`YOLO detection -> tracker`
- The tracker is `nvtracker` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1117).

- Tracker config is loaded in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1168).



`Tracked metadata -> selected target`
- The app reads `NvDsObjectMeta` directly from DeepStream metadata in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:530).

- It extracts bbox center `cx/cy` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:548).

- It selects one target in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:691) and [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:901).



`Selected target -> dx_norm/dy_norm -> pan_cmd/tilt_cmd`
- Normalized image-plane error is computed in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:913).

- The shaped control output `pan_cmd/tilt_cmd` is computed in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:951).



**Where the diagram’s two branches appear in code**

`Branch A: video / stream / monitoring`
- After OSD, the code creates a real `tee` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1120).

- One branch goes to local display in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1230).

- Another branch goes to RTSP encode/pay/sink in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1235) and [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1248).



`Branch B: metadata / control`
- This branch is not a GStreamer `tee`; it is implemented by a pad probe on the tracker output.

- The probe is attached at `tracker src pad` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1273) and [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1276).

- Inside that probe, the code reads `NvDsBatchMeta / NvDsFrameMeta / NvDsObjectMeta`, selects a target, computes error, computes commands, and writes JSONL in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:878), [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:913), and [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:778).



That is the strongest proof that `prototype_v2` matches your diagram:
- video branch stays in DeepStream
- control branch uses metadata from the tracker path
- no separate full-frame CPU image-processing control loop is used here

**Proof that gimbal control uses metadata, not the stream**
- The C app writes only metadata/control values like `dx_norm`, `dy_norm`, `pan_cmd`, `tilt_cmd` in [deepstream_yolo26_rtsp_target_control.c](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:799).

- The Python bridge reads that JSONL file line by line in [deepstream_px4_siyi_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:327).

- It reads `pan_cmd/tilt_cmd` in [deepstream_px4_siyi_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:366).

- It maps them into yaw/pitch motion in [deepstream_px4_siyi_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:377).

- It sends live gimbal commands in [deepstream_px4_siyi_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:409).

- The low-level MAVLink send is in [px4_siyi_live_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:98), [px4_siyi_live_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:199), and [px4_siyi_live_bridge.py](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:232).

So the real control chain in code is:

```text
NvDsObjectMeta
-> bbox center
-> dx_norm / dy_norm
-> pan_cmd / tilt_cmd
-> JSONL metadata/control output
-> Python bridge
-> MAVLink
-> PX4
-> SIYI
```

That is exactly the code-level proof that `prototype_v2` follows the diagram idea.
