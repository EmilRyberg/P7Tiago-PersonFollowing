import torch
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
from glob import glob
import os
from PIL import Image
import numpy as np


class ClassificationDataset(Dataset):
    def __init__(self, dataset_dir, image_size=(336, 120), transforms=None):
        self.image_folders = glob(dataset_dir + "/*/")
        self.images_with_class = []
        self.transforms = transforms
        self.image_size = image_size
        class_id = 0
        for folder in self.image_folders:
            png_paths = glob(f"{folder}*.png")
            jpg_paths = glob(f"{folder}*.jpg")
            all_images = []
            all_images.extend(png_paths)
            all_images.extend(jpg_paths)
            for img_path in all_images:
                self.images_with_class.append((img_path, class_id))
            class_id += 1

    def __len__(self):
        return len(self.images_with_class)

    def __getitem__(self, idx):
        img_path, class_id = self.images_with_class[idx]
        image = np.array(Image.open(img_path))
        image = torch.from_numpy(image)
        image = F.interpolate(image, self.image_size)
        if self.transforms:
            image = self.transforms(image)
        return image, class_id


class FeatureExtractorDataset(Dataset):
    def __init__(self, dataset_dir):
        self.image_folders = glob(dataset_dir + "/*/")
        self.class_with_images = []
        self.flattened_class_with_images = []
        class_id = 0
        for folder in self.image_folders:
            png_paths = glob(f"{folder}*.png")
            jpg_paths = glob(f"{folder}*.jpg")
            all_images = []
            all_images.extend(png_paths)
            all_images.extend(jpg_paths)
            #self.class_with_images.append((class_id, all_images))
            for img_path in all_images:
                self.flattened_class_with_images.append((class_id, img_path))
            class_id += 1

    def __getitem__(self, idx):
        return self.flattened_class_with_images[idx]
