import torch.nn.functional as F
from torch import nn


import torch


class Net00(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 3, 1, padding=1)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 64, 3, 1, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, 10, 3, 1, padding=1)
        self.bn3 = nn.BatchNorm2d(10)

    def forward(self, x):
        # Validate inputs
        if x is None:
            raise ValueError("Input tensor cannot be None")
        if not isinstance(x, torch.Tensor):
            raise TypeError("Input must be a torch.Tensor")

        x = self.bn1(self.conv1(x))
        x = F.relu(x)
        x = self.bn2(self.conv2(x))
        x = F.relu(x)
        x = self.bn3(self.conv3(x))
        return F.relu(x)
