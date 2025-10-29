#!/usr/bin/env python
import os

import torch
import torch.onnx
import torch.nn as nn

from demo_superpoint import SuperPointNet

CG_DM_ROOT = os.getenv("CG_DM_ROOT")

_CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
_WEIGHTS_PATH = os.path.join(
    _CURRENT_DIR, "SuperPointPretrainedNetwork/superpoint_v1.pth"
)

sp_model_root = f"{CG_DM_ROOT}/dataset_ml/superpoint"
model_path = f"{sp_model_root}/superpoint_v1.pth"
output_path = f"{sp_model_root}/super_point.onnx"


def replace_relu(module):
    for name, child in module.named_children():
        if isinstance(child, nn.ReLU):
            setattr(module, name, nn.SiLU(inplace=child.inplace))
        else:
            replace_relu(child)


def main():
    assert os.path.isfile(model_path)
    model = SuperPointNet()
    model.load_state_dict(torch.load(model_path))
    model.eval()

    # replace_relu(model)

    batch_size = 1
    height = 16
    width = 16
    x = torch.randn(batch_size, 1, height, width)

    torch.onnx.export(
        model,
        x,
        output_path,
        export_params=True,
        opset_version=10,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={
            "input": {0: "batch_size", 2: "height", 3: "width"},
            "output": {0: "batch_size"},
        },
    )
    print(f"\nonnx model is saved to: {output_path}")


if __name__ == "__main__":
    main()
