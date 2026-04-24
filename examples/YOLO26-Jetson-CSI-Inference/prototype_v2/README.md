# Prototype V2

Separate DeepStream-based prototype for the future optimized architecture.

This path is intentionally separate from prototype 1.

## Current Status

- CSI camera input through DeepStream works
- local native `320x320` model artifacts live under `prototype_v2/model/`
- RTSP/video branch works
- low-latency local RTSP preview helper works
- metadata dump works
- tracked-metadata control preview works
- combined RTSP + tracked-metadata control-preview pipeline works
- PX4 / SIYI bridge exists and real physical SIYI motion is proven
- shaped control law now exists for:
  - stronger continuous motion when the target is clearly off-center
  - calmer settle behavior near center

## Main Files

- `run_deepstream_yolo26_csi_app.sh`
- `run_deepstream_yolo26_rtsp.sh`
- `run_deepstream_yolo26_rtsp_preview.sh`
- `run_deepstream_yolo26_rtsp_target_control_preview.sh`
- `run_deepstream_yolo26_metadata_dump.sh`
- `run_deepstream_yolo26_target_control.sh`
- `run_deepstream_yolo26_rtsp_target_control.sh`
- `run_deepstream_px4_siyi_bridge.sh`
- `prepare_prototype_v2_model.sh`
- `deepstream_yolo26_metadata_dump.c`
- `deepstream_yolo26_target_control.c`
- `deepstream_yolo26_rtsp_target_control.c`
- `deepstream_px4_siyi_bridge.py`
- `px4_siyi_live_bridge.py`
- `model/`
- `config/config_infer_primary_yolo26.txt`
- `config/deepstream_app_yolo26_csi.template.txt`
- `config/tracker_config.txt`
- `config/config_tracker_NvDCF_perf.yml`

## Saved Bridge Presets

These are the two kept `prototype_v2` bridge presets now.
Older pre-shaped-control preset wording is removed.

### Decisive Recenter Preset

- `PAN_GAIN=0.85`
- `TILT_GAIN=0.80`
- `DEADZONE=0.05`
- `SMOOTH_ALPHA=0.40`
- `FAST_SMOOTH_ALPHA=0.75`
- `FAST_ERROR_ZONE=0.18`
- `COMMAND_BOOST_ZONE=0.10`
- `MIN_ACTIVE_COMMAND=0.18`
- `RESPONSE_GAMMA=0.65`
- `MAX_YAW_RATE_DPS=90`
- `MAX_PITCH_RATE_DPS=60`

Use this when you want the stronger far-error pull.

### Smoother Recenter Preset

- `PAN_GAIN=0.80`
- `TILT_GAIN=0.75`
- `DEADZONE=0.06`
- `SMOOTH_ALPHA=0.32`
- `FAST_SMOOTH_ALPHA=0.68`
- `FAST_ERROR_ZONE=0.18`
- `COMMAND_BOOST_ZONE=0.12`
- `MIN_ACTIVE_COMMAND=0.14`
- `RESPONSE_GAMMA=0.72`
- `MAX_YAW_RATE_DPS=90`
- `MAX_PITCH_RATE_DPS=60`

Use this when you want a calmer, smoother follow test, but keep in mind it pulls less strongly when the error is large.

## Control Shape Knobs

- `FAST_SMOOTH_ALPHA`
  - faster smoothing used outside the small-error settle zone
- `FAST_ERROR_ZONE`
  - error magnitude where the controller shifts into more decisive response
- `COMMAND_BOOST_ZONE`
  - outer zone where the controller starts to commit instead of creeping
- `MIN_ACTIVE_COMMAND`
  - minimum nonzero command once the target is clearly off-center
- `RESPONSE_GAMMA`
  - response curve shaping; lower than `1.0` reacts harder earlier

## Current Focus

- keep prototype 1 untouched
- tune `prototype_v2` bridge smoothness and response quality on top of the new shaped controller
- later, if needed, move to a larger valid model path than `320x320` for cleaner target-center signals
