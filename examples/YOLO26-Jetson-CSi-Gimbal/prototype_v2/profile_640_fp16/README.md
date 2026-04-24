# Profile 640 FP16

This is the clean `prototype_v2` `640x640` profile copy under:

- `/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16`

It is kept separate from the old working tree so the structure can be organized cleanly.

## Current Status

- `profile_640_fp16` is now a separate runnable sibling of `profile_320` and `profile_640`
- the local profile engine now lives at:
  - `model/model_b1_gpu0_fp16.engine`
- this profile is configured for FP16
- the active local engine was installed from a matching local `640` FP16 engine artifact after an earlier FP16 engine candidate proved incompatible with this profile
- DeepStream smoke run is verified from this profile:
  - engine deserializes successfully
  - camera path runs
  - JSONL metadata/control output is written
- first real local preview is also verified from this profile:
  - person detection works
  - tracking works
  - control metadata is live
- bridge dry-run is also verified from this profile:
  - metadata/control output is read
  - bridge state is written with `mavlink_link="dry-run"`
- first real physical `PX4 -> SIYI` validation for this FP16 profile is still pending
- this profile is separate from the FP32 640 profile and should be validated independently
- live `DeepStream -> PX4 -> SIYI` launch path is copied in place for this profile

Current practical meaning:

- `profile_320` remains the known-good tuned baseline
- `profile_640_fp16` is now ready for separate preview, real bridge, and tuning tests
- `profile_640_fp16` should be treated as its own profile and tuned independently from `profile_320` and `profile_640`

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

- `model/yolo26n_native_640.onnx`
  - native `640x640` ONNX model for this profile

- `model/model_b1_gpu0_fp16.engine`
  - active TensorRT engine used by this profile

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

## Bridge Presets

This is the single kept preset for `profile_640_fp16`.
It is the current main `640x640` FP16 feedforward preset.

### Main Feedforward Recenter Preset

```bash
cd /home/aarl/ultralytics
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
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export BRIDGE_STATE_FILE='/tmp/profile640fp16_main_feedforward_recenter.jsonl'
bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/run_deepstream_px4_siyi_bridge.sh
```

Use this as the default `profile_640_fp16` live preset. It keeps the feedforward controller, improves mid/far pull over the older balanced preset, and stays calm near center without the over-aggressive behavior from the later failed tries.

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
- The camera source is `nvarguscamerasrc` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:881).

- The main pipeline chain is built as:
  - `nvarguscamerasrc -> capsfilter -> nvvideoconvert -> nvstreammux -> nvinfer -> nvtracker -> nvvideoconvert -> nvdsosd -> tee`

- You can see that chain in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:977) and [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:994).



`DeepStream -> YOLO detection`
- The detector is `nvinfer` as `pgie` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:885).

- Its config file is loaded with `config-file-path` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:936).



`YOLO detection -> tracker`
- The tracker is `nvtracker` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:886).

- Tracker config is loaded in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:937).



`Tracked metadata -> selected target`
- The app reads `NvDsObjectMeta` directly from DeepStream metadata in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:443).

- It extracts bbox center `cx/cy` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:462).

- It selects one target in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:605) and [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:733).



`Selected target -> dx_norm/dy_norm -> pan_cmd/tilt_cmd`
- Normalized image-plane error is computed in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:746).

- The shaped control output `pan_cmd/tilt_cmd` is computed in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:757).



**Where the diagram’s two branches appear in code**

`Branch A: video / stream / monitoring`
- After OSD, the code creates a real `tee` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:889).

- One branch goes to local display in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:999).

- Another branch goes to RTSP encode/pay/sink in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1004) and [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1017).



`Branch B: metadata / control`
- This branch is not a GStreamer `tee`; it is implemented by a pad probe on the tracker output.

- The probe is attached at `tracker src pad` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1023) and [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:1028).

- Inside that probe, the code reads `NvDsBatchMeta / NvDsFrameMeta / NvDsObjectMeta`, selects a target, computes error, computes commands, and writes JSONL in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:693), [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:807), and [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:647).



That is the strongest proof that `prototype_v2` matches your diagram:
- video branch stays in DeepStream
- control branch uses metadata from the tracker path
- no separate full-frame CPU image-processing control loop is used here

**Proof that gimbal control uses metadata, not the stream**
- The C app writes only metadata/control values like `dx_norm`, `dy_norm`, `pan_cmd`, `tilt_cmd` in [deepstream_yolo26_rtsp_target_control.c](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_yolo26_rtsp_target_control.c:661).

- The Python bridge reads that JSONL file line by line in [deepstream_px4_siyi_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:273).

- It reads `pan_cmd/tilt_cmd` in [deepstream_px4_siyi_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:295).

- It maps them into yaw/pitch motion in [deepstream_px4_siyi_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:306).

- It sends live gimbal commands in [deepstream_px4_siyi_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/deepstream_px4_siyi_bridge.py:316).

- The low-level MAVLink send is in [px4_siyi_live_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:98), [px4_siyi_live_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:199), and [px4_siyi_live_bridge.py](/home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/px4_siyi_live_bridge.py:232).

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


