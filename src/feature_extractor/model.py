import torch.nn.functional as F
import torch.nn as nn
import torch
from torchvision.models import mobilenet_v2
import numpy as np


class FeatureDetectorNet(nn.Module):
    def __init__(self, backbone, bottleneck_input_size):
        super(FeatureDetectorNet, self).__init__()
        self.backbone = backbone
        self.bottleneck = nn.Linear(bottleneck_input_size, 128)

    def forward(self, x):
        x = self.backbone(x)
        x = x.view(-1, num_flat_features(x))
        return self.bottleneck(x)


def num_flat_features(x):
    size = x.size()[1:]  # all dimensions except the batch dimension
    num_features = 1
    for s in size:
        num_features *= s
    return num_features


def calculate_padding(kernel_size):
    return (kernel_size - 1) // 2


