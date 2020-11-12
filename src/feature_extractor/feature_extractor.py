from model import FeatureExtractorNet
import torch
from torchvision.transforms import transforms
import numpy as np


class FeatureExtractor:
    def __init__(self, weights_dir, on_gpu=True):
        self.model = FeatureExtractorNet()
        self.on_gpu = on_gpu
        self.model.load_state_dict(torch.load(weights_dir))
        self.model.eval()
        if on_gpu:
            self.model = self.model.cuda()
        self.data_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ])

    def get_features(self, cropped_person_img):
        img_transformed = self.data_transform(cropped_person_img)
        if self.on_gpu:
            img_transformed = img_transformed.cuda()
        features = self.model(img_transformed)
        features_np = features.detach().cpu().data.numpy()
        print(f"features: {features_np}")
        return features_np

    def embedding_distance(self, features_1, features_2):
        return np.linalg.norm(features_1 - features_2)

    def is_same_person(self, features_1, features_2, threshold=0.05):
        return self.embedding_distance(features_1, features_2) < threshold