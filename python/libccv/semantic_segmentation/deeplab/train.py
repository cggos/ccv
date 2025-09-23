#!/usr/bin/env python
"""
@Project : dl_with_pytorch
@File    : train_seg_net.py
@Site    : Vector-LabPics dataset: https://zenodo.org/record/3697452
@Author  : Gavin Gao
@Date    : 12/24/22 10:19 PM
"""

import os
import numpy as np
import cv2
import torch
import torchvision.transforms as tf
import argparse
from libccv.semantic_segmentation import SegmentationModelLoader, SegmentationTransforms


# os.environ["CUDA_VISIBLE_DEVICES"] = "3"

torch.cuda.empty_cache()


def ReadRandomImage(TrainFolder, ListImages, transformImg, transformAnn):
    idx = np.random.randint(0, len(ListImages))  # Pick random image
    Img = cv2.imread(os.path.join(TrainFolder, "Image", ListImages[idx]))

    Filled = cv2.imread(
        os.path.join(
            TrainFolder, "Semantic/16_Filled", ListImages[idx].replace("jpg", "png")
        ),
        0,
    )
    Vessel = cv2.imread(
        os.path.join(
            TrainFolder, "Semantic/1_Vessel", ListImages[idx].replace("jpg", "png")
        ),
        0,
    )

    AnnMap = np.zeros(Img.shape[0:2], np.float32)  # Segmentation map
    if Vessel is not None:
        AnnMap[Vessel == 1] = 1
    if Filled is not None:
        AnnMap[Filled == 1] = 2
    Img = transformImg(Img)
    AnnMap = transformAnn(AnnMap)
    return Img, AnnMap


def LoadBatch(
    TrainFolder, ListImages, transformImg, transformAnn, batchSize, height, width
):  # Load batch of images
    images = torch.zeros([batchSize, 3, height, width])
    ann = torch.zeros([batchSize, height, width])

    for i in range(batchSize):
        images[i], ann[i] = ReadRandomImage(
            TrainFolder, ListImages, transformImg, transformAnn
        )

    return images, ann


def main():
    parser = argparse.ArgumentParser(
        description="Train semantic segmentation model with refactored code"
    )
    parser.add_argument(
        "--train_folder",
        type=str,
        default="/dev_sdb/datasets/ml/LabPicsV1/Simple/Train/",
        help="Path to training data folder",
    )
    parser.add_argument(
        "--learning_rate", type=float, default=1e-5, help="Learning rate for training"
    )
    parser.add_argument(
        "--height", type=int, default=800, help="Image height for training"
    )
    parser.add_argument(
        "--width", type=int, default=800, help="Image width for training"
    )
    parser.add_argument(
        "--batch_size", type=int, default=1, help="Batch size for training"
    )
    parser.add_argument(
        "--num_classes", type=int, default=3, help="Number of classes for the model"
    )
    parser.add_argument(
        "--max_iterations",
        type=int,
        default=10000,
        help="Maximum number of training iterations",
    )
    parser.add_argument(
        "--save_interval",
        type=int,
        default=1000,
        help="Interval to save model checkpoints",
    )
    parser.add_argument(
        "--model_name",
        type=str,
        default="deeplabv3_resnet50",
        help="Model name to use for training",
    )

    args = parser.parse_args()

    Learning_Rate = args.learning_rate
    height = args.height
    width = args.width
    batchSize = args.batch_size
    num_classes = args.num_classes
    max_iterations = args.max_iterations
    save_interval = args.save_interval
    model_name = args.model_name

    TrainFolder = args.train_folder
    if not os.path.exists(TrainFolder):
        raise FileNotFoundError(f"Training folder not found: {TrainFolder}")

    ListImages = os.listdir(os.path.join(TrainFolder, "Image"))

    transformImg = SegmentationTransforms.get_preprocessing_transform((height, width))
    transformAnn = SegmentationTransforms.get_annotation_transform((height, width))

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    print(device)

    # Load model using the new model loader
    Net = SegmentationModelLoader.load_pretrained(model_name, num_classes=num_classes)
    Net = Net.to(device)

    optimizer = torch.optim.Adam(
        params=Net.parameters(), lr=Learning_Rate
    )  # Create adam optimizer

    for itr in range(max_iterations):  # Training loop
        images, ann = LoadBatch(
            TrainFolder,
            ListImages,
            transformImg,
            transformAnn,
            batchSize,
            height,
            width,
        )
        images = torch.autograd.Variable(images, requires_grad=False).to(device)
        ann = torch.autograd.Variable(ann, requires_grad=False).to(device)
        Pred = Net(images)["out"]  # make prediction
        Net.zero_grad()
        criterion = torch.nn.CrossEntropyLoss()  # Set loss function
        Loss = criterion(Pred, ann.long())  # Calculate cross entropy loss
        Loss.backward()  # Backpropogate loss
        optimizer.step()  # Apply gradient descent change to weight
        seg = torch.argmax(Pred[0], 0).cpu().detach().numpy()  # Get prediction classes
        print(itr, ") Loss=", Loss.data.cpu().numpy())
        if itr % save_interval == 0 and itr > 0:
            print("Saving Model" + str(itr) + ".torch")
            torch.save(Net.state_dict(), str(itr) + ".torch")


if __name__ == "__main__":
    main()
