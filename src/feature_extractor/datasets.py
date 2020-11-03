import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
from glob import glob
import os


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
