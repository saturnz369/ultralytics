# Jetson AGX Xavier Notes

This note narrows the example to a practical Jetson AGX Xavier starting point.

## Recommended First Pass

- Model: `yolo26n.pt`
- Device: `0`
- Input size: `640`
- CSI source: `sensor-id=0`
- Camera mode: `1280x720 @ 30 FPS`
- Precision: `FP16` enabled

These are conservative defaults intended to get the pipeline running cleanly before tuning for speed or quality.

## Why Start Here

- `yolo26n` keeps latency lower than larger variants.
- `640` is usually a reasonable first balance for embedded inference.
- `1280x720 @ 30` is a practical CSI bring-up mode on Jetson systems.
- FP16 is typically the right next step once GPU inference is working.

## Suggested Tuning Order

1. Make sure `.pt` inference works on CSI.
2. Adjust camera orientation with `FLIP_METHOD`.
3. Adjust confidence threshold with `CONF`.
4. If needed, reduce `IMGSZ` to `512` or `416` for lower latency.
5. Export TensorRT on the Xavier and switch to `.engine`.

## Launcher Examples

Local video test:

```bash
SOURCE=/tmp/jetson_test_video.mp4 DEVICE=cpu SHOW=0 SAVE=1 \
OUTPUT=/tmp/xavier_test_output.mp4 \
bash examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

GPU validation on this Xavier should use:

```bash
SOURCE=/tmp/jetson_test_video.mp4 DEVICE=0 SHOW=0 SAVE=1 \
OUTPUT=/tmp/xavier_test_output_gpu.mp4 \
bash examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

CSI bring-up:

```bash
bash examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

CSI with a different sensor and rotation:

```bash
SENSOR_ID=1 FLIP_METHOD=2 \
bash examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```

TensorRT later:

```bash
MODEL_PATH=yolo26n.engine \
bash examples/YOLO26-Jetson-CSI-Inference/run_xavier_inference.sh
```
