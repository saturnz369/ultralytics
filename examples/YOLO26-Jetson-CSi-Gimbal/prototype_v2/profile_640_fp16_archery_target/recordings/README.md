# Recording Workflow

This folder stores clean dataset recordings and extracted failure-frame zips for the `profile_640_fp16_archery_target` profile.

## Flight/Test Launch With Clean Dataset Recording

Use this when you want failure frames for labeling. It is the same main MK15 + gimbal application launch, but with `RAW_RECORD_ENABLE=1` added so the clean raw recording branch is enabled too. It still uses one terminal for the main run.

```bash
cd /home/saturnzzz/ultralytics
export RAW_RECORD_ENABLE=1
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

The clean recording is created here:

```text
recordings/archery_target_clean_YYYYMMDD-HHMMSS.mkv
```

Stop the run with `Ctrl+C`.

## Extract Failure Frames After The Run

After stopping the test, run this in a terminal:

```bash
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_latest_failure_frames.sh
```

The extractor creates:

```text
recordings/archery_target_clean_YYYYMMDD-HHMMSS_failure_frames/
recordings/archery_target_clean_YYYYMMDD-HHMMSS_failure_frames.zip
```

Use the `.zip` for labeling on the PC.

## Manual Fallback Extractor For A Specific Recording

If the live bridge-based extractor is not available for that run, use the fallback detector-miss extractor on one recording at a time.

Recommended: use the CPU-safe version below on Jetson. It is slower, but it is more stable than the GPU replay path for this offline extractor.

General command:

```bash
cd /home/saturnzzz/ultralytics

/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python \
/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_detector_miss_frames_from_video.py \
--video /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/recordings/archery_target_clean_YYYYMMDD-HHMMSS.mkv \
--weights /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/best.pt \
--class-id 0 \
--conf 0.10 \
--imgsz 640 \
--min-miss-frames 6 \
--max-miss-frames 600 \
--frames-per-segment 4 \
--progress-every 300 \
--device cpu
```

Current two recordings:

```bash
cd /home/saturnzzz/ultralytics

/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python \
/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_detector_miss_frames_from_video.py \
--video /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/recordings/archery_target_clean_20260513-154912.mkv \
--weights /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/best.pt \
--class-id 0 \
--conf 0.10 \
--imgsz 640 \
--min-miss-frames 6 \
--max-miss-frames 600 \
--frames-per-segment 4 \
--progress-every 300 \
--device cpu
```

```bash
cd /home/saturnzzz/ultralytics

/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python \
/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/tools/extract_detector_miss_frames_from_video.py \
--video /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/recordings/archery_target_clean_20260513-155348.mkv \
--weights /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/model/best.pt \
--class-id 0 \
--conf 0.10 \
--imgsz 640 \
--min-miss-frames 6 \
--max-miss-frames 600 \
--frames-per-segment 4 \
--progress-every 300 \
--device cpu
```

This manual fallback creates:

```text
recordings/archery_target_clean_YYYYMMDD-HHMMSS_detector_miss_frames/
recordings/archery_target_clean_YYYYMMDD-HHMMSS_detector_miss_frames.zip
```

The fallback command above already uses:

```bash
--device cpu
```

Keep that flag on Jetson unless you intentionally want to test the faster GPU replay path again.

## Terminal Count

For the real flight/test launch, use one terminal for the main command.

After the run is stopped, use the same terminal or a new terminal to run the extractor. Do not run the extractor while the main tracking command is still recording.

For fallback extraction, run only one video at a time and wait for the first zip to finish before starting the second.
