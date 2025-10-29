# Deep Features

---

## SuperPoint

- PyTorch to ONNX
- ONNX to RKNN

## Results

| Model | Platform | Input Size | Exe Time (ms) |
| --- |----------|---------|---------------|
| SuperPoint | RK3588   | 640x480 | 100           |
| SuperPoint | RK3576   | 640x480 | 250~300       |


## NPU

```shell
watch -n 1 cat /sys/kernel/debug/rknpu/load
```
