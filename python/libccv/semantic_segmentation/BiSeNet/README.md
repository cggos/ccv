# BiSeNet

---

## Install

```shell
pip install -r requirements.txt
```

## Dataset

将数据集存放到相应路径下

```
datasets
└── SUPS
    ├── data
    │   ├── images
    │   ├── labels
    │   └── labels_vis
    ├── test.csv
    ├── train.csv
    └── val.csv
```

## Train

```shell
# 节点数根据可用gpu数设置，单卡训练--nproc_per_node=1
torchrun --nproc_per_node=4 tools/train_amp.py --config ./configs/bisenetv1_SUPS.py
```

## Infer

```shell
python tools/demo.py --config configs/bisenetv1_SUPS.py --weight-path res/model_final.pth --img-path ./datasets/SUPS/data/images/1647073999.942919.png
```
