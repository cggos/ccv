import torch
from libccv.common.utils import LayerActivations
from lenet import LeNet5_00

net = LeNet5_00()
x = torch.randn(2, 1, 32, 32)

la = LayerActivations(net.C3)
_ = net(x)
la.remove()
