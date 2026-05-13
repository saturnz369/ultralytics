# Profile 640 FP16 Archery Target

This is a separate sibling profile for a custom single-class `target_face` detector:

- `/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target`

It is based on the current clean `profile_640_fp16` runtime, but it is wired for a custom one-class model instead of the original COCO/person model.

## Current Status

- The runtime architecture is copied from the clean working `profile_640_fp16` profile.
- The control / streaming separation is preserved:
  - latest-only shared-memory metadata handoff
  - Python MAVLink bridge outside the DeepStream callback
  - monitoring branch behind the video-side queue / OSD / tee path
- The new infer config is already set to:
  - `num-detected-classes=1`
  - `TARGET_CLASS_ID=0`
  - `model/labels.txt = target_face`
- The custom model is now imported into this Jetson profile.
- Runtime model artifacts now exist:
  - `model/best.pt`
  - `model/target_face_v1_native_640.onnx`
  - `model/model_b1_gpu0_fp16.engine`
- The serious FP16 TensorRT engine is already rebuilt and deserializes cleanly.
- Local `target_face` detection is working on the Jetson display.
- Real PX4 / SIYI gimbal control is working again on the old board.
- The MK15 `/stream` RTSP path was fixed in the full gimbal+stream wrapper and now decodes cleanly again from the exact full launcher path.
- At this point the main runtime bring-up is done. Remaining work is mostly model-quality iteration and future `v2` training improvements.

Current practical meaning:

- tracking-only tests do not need PX4 or the gimbal
- bridge dry-run does not need PX4 or the gimbal
- real bridge testing needs PX4/SIYI connected through the serial device, normally `/dev/ttyUSB0`
- the recording workflow is the same main application launch plus `RAW_RECORD_ENABLE=1`

## Current Hardware / Runtime

- Jetson: Orin NX
- JetPack/L4T used for the camera driver work: JetPack 6.2.1 / L4T 36.4.4
- DeepStream: `/opt/nvidia/deepstream/deepstream -> deepstream-7.1`
- CUDA: `/usr/local/cuda-12.6`
- Camera: e-con e-CAM121_CUONX / IMX412 CSI camera
- Camera connector used in the current working setup: CAM1
- FFC cable requirement from the working hardware setup: 22-pin, 0.5 mm pitch, Type A
- Main camera/application default for this profile: `2028x1112 @ 60`
- DeepStream-Yolo parser:
  - `/home/saturnzzz/DeepStream-Yolo/nvdsinfer_custom_impl_Yolo/libnvdsinfer_custom_impl_Yolo.so`
- Python runtime for model export and bridge:
  - `/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python`

## Prototype Workflow

This profile is not split into separate systems. It is one runtime workflow with a few launch modes:

- normal application launch
  - full MK15 + gimbal + RTSP
  - low-latency control path stays separate from the monitoring path
- dataset launch
  - same runtime as the normal launch
  - adds `RAW_RECORD_ENABLE=1`
  - records a clean pre-OSD MKV for later frame extraction
- model iteration launch
  - rebuild ONNX / TensorRT from `model/best.pt`
  - keep the same runtime settings when comparing `v1` vs `v2`

The latency-sensitive design is intentional:

- tracker metadata is published as latest-only state
- Python control consumes only the newest metadata snapshot
- video/display/RTSP use leaky queues so slow monitoring does not block control
- raw recording is separate and optional, not part of the control path

Use `2028x1112 @ 60` as the main profile default. Use `1280x720 @ 60` only if you explicitly want a lighter fallback run.

## Expected Model Files

This profile expects:

- input weight:
  - `model/best.pt`
- generated ONNX:
  - `model/target_face_v1_native_640.onnx`
- generated TensorRT engine:
  - `model/model_b1_gpu0_fp16.engine`
- labels:
  - `model/labels.txt`

The current default label file already contains:

```text
target_face
```

If your real class name should be different, edit [labels.txt](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/labels.txt).

## Import The Trained Weight

Your PC-side trained weight is:

```text
/home/saturn/ultralytics/examples/custom_training/model_v1/runs/yolo26n_target_face_v1/weights/best.pt
```

Put that file into this Jetson profile as:

```text
/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/best.pt
```

Once that file exists, this profile can generate its ONNX and rebuild the engine locally.

Original imported training artifacts are also preserved here:

- [training_weight/best.pt](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/training_weight/best.pt)
- [training_weight/best.onnx](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/training_weight/best.onnx)

## Quick Start

Run all commands from the repo root:

```bash
cd /home/saturnzzz/ultralytics
```

### 1. Build The App

```bash
make -C /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target clean all
```

### 2. Generate The ONNX From `best.pt`

After `model/best.pt` is copied into this folder:

```bash
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/prepare_prototype_v2_model.sh
```

This will:

- export ONNX from `model/best.pt`
- write `model/target_face_v1_native_640.onnx`
- refresh `model/labels.txt`
- delete any old engine so DeepStream rebuilds it

Current exported runtime ONNX:

- input: `1x3x640x640`
- output: `1x300x6`
- opset: `17`

Optional overrides:

```bash
export WEIGHTS='/some/other/path/best.pt'
export DEFAULT_LABEL='target_face'
export LABELS_SOURCE='/some/other/labels.txt'
```

### 3. First Detection / Preview Test

This is the first test to run after the ONNX exists.

```bash
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=0
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=60
export CAMERA_FPS_D=1
export TARGET_CLASS_ID=0
export SELECTION='center'
export MAX_FRAMES=0
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh
```

If you want to switch this profile to the higher-speed IMX412 mode later, change only the camera block to:

```bash
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=240
export CAMERA_FPS_D=1
```

Notes:

- `TARGET_CLASS_ID=0` is correct for this one-class model.
- The first run will also build `model/model_b1_gpu0_fp16.engine`.

## Engine Build Workflow

Use the engine workflow below when you want to bring up a new model version cleanly.

Important:

- use the command as **one copy-paste command**
- do **not** break the ONNX or engine path across lines in the middle of the path
- terminal visual wrapping is okay, but do not insert a real newline inside the path

### Stage 1: Fast First-Pass Build

Use this first when you want a quick runtime-valid engine for inspection and testing:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/target_face_v1_native_640.onnx --saveEngine=/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine --fp16 --memPoolSize=workspace:256 --avgTiming=1 --builderOptimizationLevel=0 --skipInference
```

What this does:

- rebuilds the engine from the exported `640x640` ONNX
- keeps the engine in `FP16`
- reduces TensorRT tactic search effort so engine creation finishes sooner

Optional backup if you want to preserve that quick engine before rebuilding seriously later:

```bash
cp /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.fastpass.engine
```

Current fast first-pass backup path:

- [model_b1_gpu0_fp16.fastpass.engine](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.fastpass.engine)

### Stage 1 Check

1. `trtexec` returns to the shell without a failure message.
2. The engine file timestamp changes:

```bash
stat /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine
```

3. Run a short DeepStream sanity test:

```bash
cd /home/saturnzzz/ultralytics
SHOW=0 RTSP_ENABLE=0 MAX_FRAMES=60 TARGET_CLASS_ID=0 SELECTION=center bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh
```

4. In that DeepStream output, you want to see:

- `deserialized trt engine from`
- `Use deserialized engine model`

### Stage 1 Real Test

After the sanity run passes, do your real local preview or MK15 test with this first-pass engine.

Local preview:

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=0
export TARGET_CLASS_ID=0
export SELECTION='center'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh
```

MK15 YOLO stream:

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_rtsp.sh
```

### Stage 2: Serious Final Rebuild

After the first-pass engine is validated, rebuild the stronger final-style engine:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/target_face_v1_native_640.onnx --saveEngine=/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine --fp16 --memPoolSize=workspace:1024 --builderOptimizationLevel=5 --skipInference
```

What this does:

- rebuilds the same `640x640` model into a stronger final-style engine
- keeps the engine in `FP16`
- uses a larger workspace than the fast first-pass build
- uses a higher TensorRT optimization effort than the fast first-pass build

### Stage 2 Check

1. `trtexec` returns to the shell without a failure message.
2. The engine file timestamp changes:

```bash
stat /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine
```

3. Run a short DeepStream sanity test:

```bash
cd /home/saturnzzz/ultralytics
SHOW=0 RTSP_ENABLE=0 MAX_FRAMES=60 TARGET_CLASS_ID=0 SELECTION=center bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh
```

4. In that DeepStream output, you want to see:

- `deserialized trt engine from`
- `Use deserialized engine model`

You do **not** want to see it trying to rebuild the engine again.

### Stage 2 Real Test

After the serious engine passes the sanity test, repeat the same real preview or MK15 test again. That is the runtime baseline you should use when deciding what `v2` needs to improve.

### If You Want To Force A Fresh Rebuild

Delete the current engine first:

```bash
rm -f /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/model_b1_gpu0_fp16.engine
```

Then rerun:

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=1
export RTSP_PORT=8554
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=60
export CAMERA_FPS_D=1
export TARGET_CLASS_ID=0
export SELECTION='center'
export MAX_FRAMES=0
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh
```

### 4. Bridge Dry-Run

Use this before PX4/SIYI hardware is involved.

```bash
cd /home/saturnzzz/ultralytics
export DRY_RUN_MAVLINK=1
export PRINT_STATE=1
export SHOW=0
export RTSP_ENABLE=0
export MAX_FRAMES=90
export TARGET_CLASS_ID=0
export SELECTION='center'
export CONTROL_API=command
export LIVE_CONTROL_MODE=angle-target
export MAV_INVERT_PAN=0
export MAV_INVERT_TILT=0
export BRIDGE_STATE_FILE='/tmp/profile640fp16_archery_target_bridge_dryrun.jsonl'
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_px4_siyi_bridge.sh
```

### 5. Real PX4 / SIYI Run

Use this only after the `target_face` detector is already working locally.

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=0
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
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_px4_siyi_bridge.sh
```

To switch this full-control block to the higher-speed camera mode later, change only:

```bash
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=240
export CAMERA_FPS_D=1
```

## Streaming / MK15

This profile also has the same MK15 streaming layer as the main profile:

- [streaming/README.md](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/README.md)

That means you can later run:

- camera-only RTSP
- YOLO overlay RTSP
- YOLO + gimbal + RTSP
- YOLO + gimbal + RTSP + clean pre-OSD recording for dataset extraction

For flight/test dataset collection, use the full MK15 launcher with clean raw recording enabled:

```bash
cd /home/saturnzzz/ultralytics
RAW_RECORD_ENABLE=1 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

After stopping the run, extract the missed-detection frames from the newest clean recording:

```bash
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh
```

The output zip under `recordings/` is the one to move to the PC and label. The clean recording is taken from the camera-side branch before inference, tracker, OSD, RTSP, and debug drawing.

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
-> nvvideoconvert
-> nvdsosd
-> tee
-> display queue -> local preview
-> rtsp queue -> H.264 -> RTSP
```

### What Is Cleanly Separated Now

- live gimbal commands are not sent inside the DeepStream callback
- the Python bridge reads only the latest metadata snapshot
- slow preview or RTSP no longer causes the bridge to replay old metadata line by line
- the display and RTSP paths sit behind leaky queues, so they are allowed to fall behind and drop frames
- `RTSP_ENABLE=0` now really disables the RTSP encode/stream branch

### What This Means In Practice

- the control branch can stay fresher than the monitoring branch
- the MK15 stream or local preview may be a few frames behind, and that is acceptable
- the important goal is that the gimbal follows fresh target metadata, not old display frames

The timing fields below are the proof for this:

- `ds_to_py_ms` shows metadata handoff latency into the Python bridge
- `display_frame_lag` shows how far preview trails the current control frame
- `rtsp_frame_lag` shows how far RTSP trails the current control frame
- `mav_send_to_feedback_ms` shows the observed command-to-feedback timing when PX4/gimbal feedback is available

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
   - this is the detector / inference stage that feeds object metadata forward

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

After detection, tracking, and target-control calculation, the profile splits into 2 branches:

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
- this branch is not the control loop itself

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

- the image path should stay in the DeepStream / GPU-oriented pipeline
- the video branch exists for preview / streaming / monitoring
- the control branch should consume only metadata-level information
- the gimbal should follow the metadata/control branch, not the preview stream itself

In other words:

- full-frame video is for display / streaming
- metadata is for control

That is why `prototype_v2` is the future optimized direction:

- it separates monitoring from control
- it avoids using full exported image frames as the main control signal
- it uses target metadata and image-plane error as the control input to `PX4 -> SIYI`

Live control path in the current implementation:

```text
CSI camera
-> DeepStream / YOLO / tracker
-> tracker src pad-probe
-> latest-only shared-memory metadata snapshot
-> separate Python bridge loop
-> MAVLink / PX4 gimbal-manager
-> SIYI gimbal
```

Monitoring path in the current implementation:

```text
tracker output
-> leaky video queue
-> nvvideoconvert
-> nvdsosd
-> tee
-> display queue -> local preview
-> rtsp queue -> H.264 -> RTSP
```

Dataset recording path when `RAW_RECORD_ENABLE=1`:

```text
camera-side source tee
-> leaky raw-record queue
-> hardware H.264 encoder
-> local clean MKV under recordings/
```

Important meaning:

- the gimbal does not follow the RTSP video
- the bridge does not replay old metadata line by line
- the final control shaping happens in Python, not inside the DeepStream callback
- clean dataset recording is separate from OSD/RTSP and does not contain overlays
- the control branch should use lightweight metadata only, not full video frames

## Runtime Timing Fields

When `BRIDGE_STATE_FILE` is enabled, the bridge log can show separation/latency metrics such as:

- `ds_to_py_ms`
  - live metadata handoff latency from DeepStream publish to Python read
- `mav_send_to_feedback_ms`
  - command send to observed PX4/gimbal feedback timing when feedback is available
- `display_frame_lag`
  - how many frames the preview branch trails the current control frame
- `rtsp_frame_lag`
  - how many frames the RTSP branch trails the current control frame

## Files That Matter

- [config/config_infer_primary_yolo26.txt](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/config/config_infer_primary_yolo26.txt)
  - points to the `target_face` ONNX / engine / labels
  - one class only
- [prepare_prototype_v2_model.sh](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/prepare_prototype_v2_model.sh)
  - exports the ONNX from `model/best.pt`
- [run_deepstream_yolo26_rtsp_target_control.sh](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_yolo26_rtsp_target_control.sh)
  - launches the DeepStream app
- [run_deepstream_px4_siyi_bridge.sh](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/run_deepstream_px4_siyi_bridge.sh)
  - launches the full live bridge path
- [tools/extract_latest_failure_frames.sh](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh)
  - extracts labelable failure frames from the newest clean recording and bridge log

## Next Iteration Targets

The runtime side is now in a usable state. The next work should be about model quality, not basic bring-up.

Useful next checks:

1. evaluate `target_face` stability on real scenes, distance changes, and partial views
2. decide whether `v2` needs better data coverage, tighter labels, or harder negatives
3. compare `v1` serious-engine behavior against any future `v2` training run under the same runtime settings
