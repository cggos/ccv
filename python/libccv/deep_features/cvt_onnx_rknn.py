import os
import sys

from rknn.api import RKNN

CG_DM_ROOT = os.getenv("CG_DM_ROOT")

if __name__ == "__main__":
    sp_model_root = f"{CG_DM_ROOT}/dataset_ml/superpoint"
    model_path = f"{sp_model_root}/super_point.onnx"
    output_path = f"{sp_model_root}/super_point.rknn"

    platform = "rk3576"

    rknn = RKNN(verbose=False)

    print("--> Config model")
    rknn.config(
        target_platform=platform,
        compress_weight=True,
        model_pruning=True,
        single_core_mode=True,
        dynamic_input=[
            [[1, 1, 480, 640]],
        ],
    )
    print("done")

    print("--> Loading model")
    ret = rknn.load_onnx(
        model=model_path,
        inputs=["input"],
        input_size_list=[[1, 1, 480, 640]],
        outputs=["output", "52"],
    )
    if ret != 0:
        print("Load model failed!")
        exit(ret)
    print("done")

    print("--> Building model")
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print("Build model failed!")
        exit(ret)
    print("done")

    print("--> Export rknn model")
    ret = rknn.export_rknn(output_path)
    if ret != 0:
        print("Export rknn model failed!")
        exit(ret)
    print("done")

    rknn.release()
