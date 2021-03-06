from person_detector.feature_extractor.model import FeatureExtractorNet
import torch
import torch.nn.functional as F
from torchvision.transforms import transforms
import numpy as np


def embedding_distance(features_1, features_2):
    return np.linalg.norm(features_1 - features_2)


def is_same_person(features_1, features_2, threshold=1):
    return embedding_distance(features_1, features_2) < threshold


class FeatureExtractor:
    def __init__(self, weights_dir, on_gpu=True, image_size=(336, 120)):
        self.model = FeatureExtractorNet()
        self.on_gpu = on_gpu
        self.image_size = image_size
        if on_gpu:
            self.model.load_state_dict(torch.load(weights_dir))
        else:
            self.model.load_state_dict(torch.load(weights_dir, map_location=torch.device('cpu')))
        self.model.eval()
        if on_gpu:
            self.model = self.model.cuda()
        self.data_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ])

    def get_features(self, cropped_person_img):
        img_transformed = self.data_transform(cropped_person_img)
        img_transformed = F.interpolate(img_transformed.unsqueeze(0), self.image_size)
        #img_transformed = img_transformed.unsqueeze(0)
        if self.on_gpu:
            img_transformed = img_transformed.cuda()
        features = self.model(img_transformed)
        features = F.normalize(features, p=2)
        features_np = features.detach().cpu().data.numpy()
        return features_np