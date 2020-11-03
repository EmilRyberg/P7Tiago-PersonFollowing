import torch
import numpy as np
from torchvision.models import mobilenet_v2
from model import FeatureDetectorNet
from datasets import FeatureExtractorDataset

if __name__ == "__main__":
    height, width = (336, 120)
    test_tensor = torch.randn((1, 3, height, width))
    dataset = FeatureExtractorDataset("dataset")

    # mobile_net = mobilenet_v2(True)
    # backbone = mobile_net.features
    # detector = FeatureDetectorNet(backbone, 1280 * 11 * 4)  # when using 120x336 img
    # res = detector(test_tensor)
