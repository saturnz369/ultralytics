# Bench Control

This folder is for one scripted `Jetson -> UART -> PX4 -> MAVLink gimbal v2 -> SIYI` bench test only.

Current kept files:
- `px4_siyi_bench_bridge.py`
  - low-level bench bridge to PX4 gimbal manager
- `jetson_px4_siyi_gimbal_speed_test.py`
  - scripted angle-step bench test with optional multi-step sequence and rate hints
- `run_px4_siyi_gimbal_speed_test.sh`
  - launcher for the scripted angle-step test

Main saved command right now:

```bash
cd /home/aarl/ultralytics

export START_PITCH=0
export START_YAW=0
export START_HOLD_SECONDS=0.2

export TARGET_PITCH=0
export TARGET_YAW=135
export TARGET_HOLD_SECONDS=3.0

export SECOND_TARGET_PITCH=nan
export SECOND_TARGET_YAW=nan
export SECOND_TARGET_HOLD_SECONDS=0.0

export THIRD_TARGET_PITCH=nan
export THIRD_TARGET_YAW=nan
export THIRD_TARGET_HOLD_SECONDS=0.0

export RETURN_AFTER=1
export RETURN_HOLD_SECONDS=0.0

export MOVE_TRANSIT_SECONDS=0.0
export START_TRANSIT_SECONDS=0.2
export TARGET_TRANSIT_SECONDS=0.0
export RETURN_TRANSIT_SECONDS=1.0

export MOVE_PITCH_RATE_DPS=0
export MOVE_YAW_RATE_DPS=1500

export STATE_FILE='/tmp/gimbal_speed_simple_135_hold3_return0.jsonl'

bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/bench_control/run_px4_siyi_gimbal_speed_test.sh
```

Main speed-related value:
- `MOVE_YAW_RATE_DPS`
  - this is the main yaw speed hint on the angle-target path
- `MOVE_PITCH_RATE_DPS`
  - this is the main pitch speed hint on the angle-target path

Other timing values:
- `TARGET_HOLD_SECONDS`
  - how long to stay at the target before returning
- `START_TRANSIT_SECONDS`
- `TARGET_TRANSIT_SECONDS`
- `RETURN_TRANSIT_SECONDS`
  - these control how long the script allows a move leg before sending the next target

Notes:
- exact angle commands are the known-good path
- pure rate-mode bench tests were not useful on this setup
- usable gimbal angle feedback is still not coming back on this setup, so this script is best treated as a sequence runner, not a true automatic speed meter

Simple pitch test command:

```bash
cd /home/aarl/ultralytics

export START_PITCH=0
export START_YAW=0
export START_HOLD_SECONDS=0.2

export TARGET_PITCH=-45
export TARGET_YAW=0
export TARGET_HOLD_SECONDS=3.0

export SECOND_TARGET_PITCH=nan
export SECOND_TARGET_YAW=nan
export SECOND_TARGET_HOLD_SECONDS=0.0

export THIRD_TARGET_PITCH=nan
export THIRD_TARGET_YAW=nan
export THIRD_TARGET_HOLD_SECONDS=0.0

export RETURN_AFTER=1
export RETURN_HOLD_SECONDS=0.0

export MOVE_TRANSIT_SECONDS=0.0
export START_TRANSIT_SECONDS=0.2
export TARGET_TRANSIT_SECONDS=0.0
export RETURN_TRANSIT_SECONDS=1.0

export MOVE_PITCH_RATE_DPS=1500
export MOVE_YAW_RATE_DPS=0

export STATE_FILE='/tmp/gimbal_pitch_minus45_hold3_return0.jsonl'

bash /home/aarl/ultralytics/examples/YOLO26-Jetson-CSi-Gimbal/bench_control/run_px4_siyi_gimbal_speed_test.sh
```
