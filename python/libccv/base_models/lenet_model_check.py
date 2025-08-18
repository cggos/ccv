import torch
from libccv.common import utils

from lenet import LeNet5_00

net = LeNet5_00()


for _key, _value in net.state_dict().items():
    pass

data_in = torch.rand(1, 1, 32, 32)
data_out = net(data_in)

utils.output_model_params(net, data_in.shape)
