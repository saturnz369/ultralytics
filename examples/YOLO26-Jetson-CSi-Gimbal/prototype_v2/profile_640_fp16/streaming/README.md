# MK15 Streaming README

This file is the final operator note for streaming Jetson video into the SIYI MK15 FPV app.

Verified working paths:

- `CSI camera -> H.264 RTSP -> MK15 FPV`
- `CSI camera -> YOLO/overlay -> H.264 RTSP -> MK15 FPV`

Primary MK15 URL:

```text
rtsp://192.168.144.100:8554/stream
```

Use `Camera A` in the MK15 FPV app. For these video-only tests, `QGroundControl` is not needed.

## Plain Explanation

This streaming system turns the Jetson into a third-party IP camera source for the SIYI MK15 system.

In simple words:

- The CSI camera sends live video into the Jetson.
- The Jetson either:
  - sends the raw live camera video out as RTSP, or
  - runs YOLO/tracking first, draws the overlay, then sends that processed video out as RTSP.
- The Jetson encodes the video as `H.264`.
- The RTSP stream leaves the Jetson through Ethernet into the `MK15 air unit` LAN port.
- The MK15 air unit sends that video wirelessly to the MK15 handheld.
- The MK15 handheld `FPV` app opens the Jetson RTSP URL and shows the video.

So the full idea is:

```text
CSI camera -> Jetson -> H.264 RTSP -> MK15 air unit -> MK15 handheld FPV app
```

For the YOLO display case, it becomes:

```text
CSI camera -> Jetson -> DeepStream / YOLO / tracker / overlay -> H.264 RTSP -> MK15 air unit -> MK15 handheld FPV app
```

Important separation:

- The `RTSP stream` is the video/monitoring path.
- The `gimbal control` path is separate and uses metadata/MAVLink, not the RTSP video itself.

## Output Settings

Current default RTSP output to the MK15 handheld is:

- `1280x720`
- `30 fps`
- `H.264`

Important:

- `profile_640_fp16` does **not** mean the streamed video is `640x640`
- the `640` part is the YOLO inference profile, not the MK15 video output size

Main variables:

- `CAMERA_WIDTH`
- `CAMERA_HEIGHT`
- `CAMERA_FPS_N`
- `CAMERA_FPS_D`
- `BITRATE`

Examples:

- `CAMERA_WIDTH=1280`
- `CAMERA_HEIGHT=720`
- `CAMERA_FPS_N=30`
- `CAMERA_FPS_D=1`
- `BITRATE=3000000` for camera-only RTSP
- `BITRATE=4000000` for YOLO / full combined RTSP

Temporary override example:

```bash
cd /home/saturnzzz/ultralytics
export CAMERA_WIDTH=1920
export CAMERA_HEIGHT=1080
export CAMERA_FPS_N=30
export CAMERA_FPS_D=1
export BITRATE=4000000
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/streaming/run_mk15_yolo_rtsp.sh
```

If you want to verify the actual outgoing RTSP stream from the Jetson while it is running:

```bash
ffprobe -v error -show_streams rtsp://127.0.0.1:8554/stream
```

## Three Different Sizes

It is important not to mix up these four things:

1. `Sensor mode`
2. `Camera capture size into the pipeline`
3. `YOLO inference size`
4. `RTSP output size to MK15`

In the current working setup:

- The CSI pipeline requests `1280x720 @ 30 fps`
- YOLO uses the `profile_640_fp16` inference profile, meaning `640x640`
- The RTSP output to MK15 is `1280x720 @ 30 fps`

Simple explanation:

- The camera sensor can run at its own native mode internally.
- Jetson then requests a working video size for the pipeline.
- DeepStream resizes for YOLO inference.
- The RTSP branch sends the video out again at the selected stream size.

So do not read `profile_640_fp16` as “the MK15 video is 640x640”.

The practical current picture is:

```text
sensor mode: internal camera mode chosen by Argus
pipeline capture: 1280x720 @ 30
YOLO inference: 640x640
MK15 RTSP output: 1280x720 @ 30
```

## Hardware / Network

- Jetson internet/Codex link should stay on Wi-Fi.
- Jetson Ethernet `enP8p1s0` should go to the MK15 air unit LAN port.
- MK15 air unit and handheld must be powered and linked.
- CSI camera must already be working on Jetson.
- For the full gimbal mode later, the user running the launcher must have read/write access to `/dev/ttyUSB0`.

Jetson Ethernet static IP:

```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name mk15 \
  ipv4.method manual ipv4.addresses 192.168.144.100/24 \
  ipv4.never-default yes ipv6.method ignore
sudo nmcli con up mk15
```

Quick check:

```bash
ip -br addr show enP8p1s0
ping -I enP8p1s0 -c 2 192.168.144.11
```

Expected Jetson Ethernet address:

```text
192.168.144.100/24
```

## Launch 1: Camera Only To MK15

This is the simplest working video path. No YOLO, no PX4, no gimbal.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/streaming/run_mk15_csi_rtsp.sh
```

What to do on MK15:

1. Open the `FPV` app.
2. Set `Camera A` to `rtsp://192.168.144.100:8554/stream`.
3. Reopen the stream view.

Result:

- You should see the real live CSI camera video on the MK15 handheld.

## Launch 2: YOLO Overlay To MK15

This is the working `prototype_v2` video branch:

```text
CSI camera -> DeepStream -> YOLO -> tracker -> overlay -> H.264 RTSP -> MK15 FPV
```

This is video/monitoring only. It does not need PX4 or the gimbal powered.

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/streaming/run_mk15_yolo_rtsp.sh
```

What to do on MK15:

1. Keep `Camera A` as `rtsp://192.168.144.100:8554/stream`.
2. Reopen the FPV stream.
3. Stand in front of the camera if you want to see `person` detection/tracking.

Result:

- You should see live camera video plus the YOLO/DeepStream overlay on the MK15 handheld.

## Launch 3: YOLO + Gimbal + RTSP To MK15

Use this only after PX4, SIYI gimbal, and serial/MAVLink wiring are ready.

This is a single combined runtime, not two separate terminals.

Meaning:

- the existing tuned `profile_640_fp16` control logic stays the same
- the gimbal still follows metadata/MAVLink
- the MK15 video is just the added RTSP video branch in the same run

Flow:

```text
CSI camera -> DeepStream / YOLO / tracker / control metadata -> PX4 / SIYI gimbal
                                         -> H.264 RTSP -> MK15 FPV
```

### Full Run: Simple

This is the normal one-shot command.

It already uses the tuned `profile_640_fp16` defaults for:

- target class / selection
- control gains
- feedforward values
- MAVLink serial settings
- MK15 RTSP settings
- the current working gimbal bridge mode for this hardware:
  - `CONTROL_API=command`
  - `LIVE_CONTROL_MODE=angle-target`
  - `MAV_INVERT_PAN=0`
  - `MAV_INVERT_TILT=0`

```bash
cd /home/saturnzzz/ultralytics
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

### Attitude vs Command

The full gimbal bridge supports multiple MAVLink control APIs.

- `attitude`
  - Sends `gimbal_manager_set_attitude`.
  - This is the more theory-clean attitude-style path.
  - It can be more sensitive to mount direction, frame conventions, and how PX4/SIYI interpret the message.
- `command`
  - Sends `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`.
  - This is a more direct pitch/yaw command path.
  - For this current Jetson + PX4 + SIYI + camera-mount setup, this is the working recommended mode.

Current recommended mode in this folder:

```text
CONTROL_API=command
LIVE_CONTROL_MODE=angle-target
```

Why this is what we use now:

- The old tracking tune is still the same.
- Only the MAVLink bridge behavior changed.
- The camera is now mounted in the same forward direction as the gimbal.
- With this hardware/mount setup, `command` behaved more consistently than `attitude`.

If the camera/gimbal mounting changes again later, the direction-related variables to re-check first are:

- `MAV_INVERT_PAN`
- `MAV_INVERT_TILT`

### Full Run: Advanced / Tuning

If you want to tune or temporarily override settings, keep using the same wrapper, but set environment variables before launch.

Example:

```bash
cd /home/saturnzzz/ultralytics
export DISPLAY=:1
export SHOW=1
export RTSP_ENABLE=1
export RTSP_PORT=8554
export RTSP_MOUNT='/stream'
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
bash /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16/streaming/run_mk15_yolo_gimbal_rtsp.sh
```

Use this advanced style when you want to change things like:

- `PAN_GAIN`, `TILT_GAIN`, `DEADZONE`
- `MAX_YAW_RATE_DPS`, `MAX_PITCH_RATE_DPS`
- `MAX_YAW_ANGLE_DEG`, `MIN_PITCH_ANGLE_DEG`, `MAX_PITCH_ANGLE_DEG`
- `TARGET_CLASS_ID`, `SELECTION`
- `SHOW`

## Stop / Restart

Stop the current stream from the same terminal with `Ctrl-C`.

If port `8554` is stuck:

```bash
pgrep -af 'csi_h264_rtsp_server|deepstream_yolo26_rtsp_target_control'
pkill -f csi_h264_rtsp_server
pkill -f deepstream_yolo26_rtsp_target_control
```

## Rebuild Note

If this profile is copied to another Jetson or the C files changed, rebuild once before testing:

```bash
cd /home/saturnzzz/ultralytics
make -C /home/saturnzzz/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/prototype_v2/profile_640_fp16 clean all
```

The MK15 RTSP compatibility fix is compiled into:

- `csi_h264_rtsp_server`
- `deepstream_yolo26_rtsp_target_control`
