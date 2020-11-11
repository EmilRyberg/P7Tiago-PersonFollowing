import torch.nn.functional as F
import torch.nn as nn
import torch
from torchvision.models import mobilenet_v2
import numpy as np


class FeatureDetectorNet(nn.Module):
    def __init__(self, backbone=None, bottleneck_input_size=1280 * 11 * 4, use_classifier=False, num_classes=6):
        super(FeatureDetectorNet, self).__init__()
        self.backbone = None
        if backbone:
            self.backbone = backbone
        else:
            mn = mobilenet_v2(True)
            self.backbone = mn.features
        self.bottleneck = nn.Linear(bottleneck_input_size, 128)
        self.classifier = nn.Linear(128, num_classes)
        self.use_classifier = use_classifier

    def forward(self, x):
        x = self.backbone(x)
        x = x.view(-1, num_flat_features(x))
        x = self.bottleneck(x)
        if self.use_classifier:
            x = self.classifier(x)
        return x


def num_flat_features(x):
    size = x.size()[1:]  # all dimensions except the batch dimension
    num_features = 1
    for s in size:
        num_features *= s
    return num_features


def calculate_padding(kernel_size):
    return (kernel_size - 1) // 2


