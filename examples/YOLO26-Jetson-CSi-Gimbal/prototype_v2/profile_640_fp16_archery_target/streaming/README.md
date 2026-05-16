# MK15 Streaming README

This file is the stream-only operator note for:

- `/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target`

Use this README for:

- MK15 video streaming
- full MK15 + gimbal application launch
- stream-side troubleshooting
- clean stream-side dataset recording

Use the main profile README for the full system architecture and non-streaming topics:

- [README.md](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/README.md)

Use the recording README for deeper post-run extraction workflow:

- [recordings/README.md](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/recordings/README.md)

## Current Streaming Target

Current primary MK15 URL:

```text
rtsp://192.168.144.100:8554/stream
```

Current default stream side:

- camera capture: `2028x1112 @ 60`
- YOLO inference: `640x640`
- RTSP output: `2028x1112 @ 60`
- codec: `H.264`

Current stream-related defaults in this profile:

- `RTSP_ENABLE=1`
- `RTSP_PORT=8554`
- `RTSP_MOUNT=/stream`
- `SHOW=1` in the one-shot full launcher
- `RUN_ARTIFACTS_ENABLE=1`

## Stream Architecture

The MK15 stream is the monitoring branch, not the control signal.

Plain stream path:

```text
CSI camera -> Jetson -> H.264 RTSP -> MK15 air unit -> MK15 handheld FPV
```

YOLO stream path:

```text
CSI camera -> DeepStream / YOLO / tracker / overlay -> H.264 RTSP -> MK15
```

Control path stays separate:

```text
DeepStream tracker -> latest shared-memory metadata snapshot -> Python MAVLink bridge -> PX4 -> SIYI
```

Important meaning:

- RTSP is the operator view
- local preview is the operator view
- neither one is the real control loop
- slow RTSP or preview is allowed to lag behind the control branch

Current runtime hardening that matters to streaming:

- video branch queues are leaky
- RTSP can be disabled independently with `RTSP_ENABLE=0`
- monitoring branch errors are non-fatal by default
- latest metadata/control stays outside the RTSP path

## Current Hardware Map For Streaming Use

Main streaming camera:

- e-con IMX412 on `CAM1`
- normal camera mode: `2028x1112 @ 60`

Current PX4 serial map around the stream/gimbal system:

- `TELEM1 / MK15 air unit`
  - `57600 8N1`
- `TELEM2 / Jetson bridge`
  - `921600 8N1`
- `TELEM3 / SIYI gimbal`
  - `115200 8N1`

Current Jetson bridge side:

- serial device: `/dev/ttyUSB0`
- USB-UART type: CP2102 / Silicon Labs style
- bridge baud: `921600`

Current network side:

- Jetson Ethernet goes to the MK15 air unit network side
- MK15 stream URL stays:
  - `rtsp://192.168.144.100:8554/stream`

## Before Streaming

This streaming folder assumes:

- the model is already prepared
- the engine already exists or can deserialize/build correctly
- the main profile binary is already built

If you replace the model later:

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/prepare_prototype_v2_model.sh
```

Then do one local DeepStream run so the engine is rebuilt or verified.

## Launch 1: Camera Only To MK15

Use this when you only want the camera stream and no YOLO overlay.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_csi_rtsp.sh
```

On MK15:

1. Open `FPV`.
2. Set `Camera A` to `rtsp://192.168.144.100:8554/stream`.
3. Reopen the stream view.

## Launch 2: YOLO Overlay To MK15

Use this when you want the stream with detection/tracking overlay, but no PX4/gimbal control.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_rtsp.sh
```

Notes:

- `TARGET_CLASS_ID=0` is the custom `target_face` class
- no PX4 or gimbal is required
- MK15 URL stays:
  - `rtsp://192.168.144.100:8554/stream`

If you want a higher stream bitrate for a run:

```bash
cd /home/saturnzzz/ultralytics
BITRATE=10000000 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_rtsp.sh
```

## Launch 3: Real Full Application

This is the main real application path for MK15 + gimbal use.

Use it only after:

- the `target_face` model is already working locally
- PX4 and SIYI wiring are ready
- `/dev/ttyUSB0` is correct
- MK15 air unit networking is connected

One-shot launcher:

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

This wrapper already carries the current working defaults:

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
- current accepted tuning block for `target_face`
- `RAW_RECORD_ENABLE=0`

MK15 URL in this full mode:

- `rtsp://192.168.144.100:8554/stream`

## Launch 3A: Full Explicit Real Application Launch

This is the recommended explicit block for real application use when you want every important stream/control setting visible in one place.

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
export MAX_FRAMES=0
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
export INITIAL_YAW_DEG=0.0
export INITIAL_PITCH_DEG=0.0
export YAW_LOCK=0
export PITCH_LOCK=0
export MAV_INVERT_PAN=0
export MAV_INVERT_TILT=0
export SERIAL_DEVICE='/dev/ttyUSB0'
export SERIAL_BAUD=921600
export MAV_SOURCE_SYSTEM=42
export MAV_SOURCE_COMPONENT=191
export MAV_TARGET_SYSTEM=1
export MAV_TARGET_COMPONENT=154
export GIMBAL_DEVICE_ID=154
export RAW_RECORD_ENABLE=0
export METADATA_MAX_AGE_MS=150
export MONITORING_ERRORS_FATAL=0
export PRINT_HEALTH=1
export HEALTH_PRINT_INTERVAL_SEC=1.0
export RUN_ARTIFACTS_ENABLE=1
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

Preflight hold-angle note:

- `INITIAL_PITCH_DEG` and `INITIAL_YAW_DEG` set the first angle-target hold setpoint before tracking starts
- example for a slight downward camera angle before takeoff:
  - `export INITIAL_PITCH_DEG=-12.0`
- `PITCH_LOCK=1` and `YAW_LOCK=1` are optional lock-mode flags; keep them at `0` unless you intentionally want lock behavior

What this gives you:

- Jetson local preview
- MK15 RTSP stream
- live PX4/SIYI control
- health print
- run bundle under `runs/<tag>/`

Normal meaning:

- this is the real full application launch
- it does **not** record by default
- the current default is `RAW_RECORD_ENABLE=0`

If you want the same full application but with less local display load:

```bash
cd /home/saturnzzz/ultralytics
SHOW=0 RTSP_ENABLE=1 bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

That keeps MK15 streaming and control, but turns off the Jetson local preview window.

If you want the high-speed IMX412 candidate mode later, change only:

```bash
export SENSOR_ID=0
export CAMERA_WIDTH=2028
export CAMERA_HEIGHT=1112
export CAMERA_FPS_N=240
export CAMERA_FPS_D=1
```

## Launch 3B: Full Application With Clean Recording

Do not use this streaming note as the source-of-truth for the record-enabled variant.

Use the dedicated recording operator note:

- [recordings/README.md](/home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target/recordings/README.md)

That file carries:

- the full recording-enabled launch block
- run bundle recording behavior
- failure-frame extraction
- detector-miss fallback extraction

## Stream-Side Notes About The 12 Cleanups

The streaming operator does not need every internal detail, but these current behaviors matter:

- stream and preview are monitoring only
- latest metadata/control is no longer a per-frame JSON replay path
- stale metadata is age-gated in the Python bridge
- monitoring queues are leaky
- `RTSP_ENABLE=0` really disables the RTSP encode branch
- monitoring-branch errors are non-fatal by default
- run artifact bundling is on by default

Practical meaning:

- MK15 disconnects do not define control latency
- local preview lag does not automatically mean gimbal lag
- recording is separate from the overlay/RTSP branch

## Rebuild

If you changed code in this profile:

```bash
cd /home/saturnzzz/ultralytics
make -C /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16_archery_target clean all
```

## Stop

To stop running RTSP / DeepStream processes:

```bash
pkill -f deepstream_yolo26_rtsp_target_control
pkill -x csi_h264_rtsp_server
```

## Troubleshooting

### MK15 Says Connected But Screen Is Blank

1. Stop the current launcher.
2. Launch again from this README.
3. Reopen `Camera A` in the FPV app with:
   - `rtsp://192.168.144.100:8554/stream`

This profile previously had a blank-screen problem because the full wrapper did not publish the right RTSP payloader path. The current fixed path is `/stream`.

### Check The RTSP Stream Locally On Jetson

```bash
ffprobe -rtsp_transport tcp -v error -select_streams v:0 -show_entries stream=codec_name,width,height,r_frame_rate -of default=nw=1 rtsp://127.0.0.1:8554/stream
```

Extract one frame locally:

```bash
ffmpeg -y -rtsp_transport tcp -i rtsp://127.0.0.1:8554/stream -frames:v 1 /tmp/archery_rtsp_frame_check.jpg
```

### Check The Jetson Bridge Serial Device

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id 2>/dev/null
```

If the Jetson bridge is missing, the full gimbal launch may stream video but control will not move the gimbal.

### If You Need Strict Monitoring Failure Behavior

Default runtime behavior is tolerant:

- `MONITORING_ERRORS_FATAL=0`

If you want the app to fail fast on monitoring-branch errors for debugging:

```bash
export MONITORING_ERRORS_FATAL=1
```

## Backup Camera Plan: ArduCam IMX477

Main deployment camera for this profile is still the e-con IMX412 setup. Use the ArduCam IMX477 only as a backup plan if the IMX412 path is unavailable.

Do not assume stock NVIDIA `imx477-A` or `imx477-C` overlays are enough here. The working path on this Jetson was ArduCam's own installer:

```bash
cd ~
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m imx477
sudo depmod -a
sudo reboot
```

Verified backup mode on this Jetson:

- `1920x1080 @ 60`

For more detail on the backup camera path, use the main profile README instead of extending this streaming note further.
