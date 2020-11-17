import torch
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
from glob import glob
import os
from PIL import Image
import numpy as np
import cv2 as cv


class ClassificationDataset(Dataset):
    def __init__(self, dataset_dir, image_size=(336, 120), data_transform=None):
        self.image_folders = glob(dataset_dir + "/*/")
        self.images_with_class = []
        self.data_transform = data_transform
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
        image = Image.open(img_path)
        if self.data_transform:
            image = self.data_transform(image)
        else:
            image = transforms.ToTensor()(image)
        image = F.interpolate(image.unsqueeze(0), self.image_size).squeeze(0)
        return image, class_id


class TripletDataset(Dataset):
    def __init__(self, dataset_dir, images_per_class=200, image_size=(336, 120), data_transform=None):
        self.image_folders = glob(dataset_dir + "/*/")
        self.images_grouped_by_class = []
        self.flattened_class_with_images = []
        self.images_per_class = images_per_class
        self.image_size = image_size
        self.data_transform = data_transform
        self.triplets = []
        class_id = 0
        self.num_classes = len(self.image_folders)
        for folder in self.image_folders:
            png_paths = glob(f"{folder}*.png")
            jpg_paths = glob(f"{folder}*.jpg")
            all_images = []
            all_images.extend(png_paths)
            all_images.extend(jpg_paths)
            for img_path in all_images:
                self.flattened_class_with_images.append((img_path, class_id))
            self.images_grouped_by_class.append((class_id, all_images))
            class_id += 1
        self.sample_triplets()

    def sample_triplets(self):
        self.triplets = []
        for id in range(self.num_classes):
            cid, class_images = self.images_grouped_by_class[id]
            class_images = np.array(class_images).copy()
            other_images = [r[1] for r in self.images_grouped_by_class if r[0] != cid]
            other_images = [item for sub_list in other_images for item in sub_list]
            other_images = np.array(other_images)
            np.random.shuffle(other_images)
            np.random.shuffle(class_images)
            for i in range(self.images_per_class):
                anchor = class_images[0]
                class_images = class_images[1:]
                positive = np.random.choice(class_images, 1)[0]
                #print(f"a: {anchor}, p: {positive}, n: {negative}")
                self.triplets.append((cid, anchor, positive))

    def __len__(self):
        return len(self.triplets)

    def __getitem__(self, idx):
        cid, a_path, p_path = self.triplets[idx]
        a_img = Image.open(a_path)
        p_img = Image.open(p_path)

        if self.data_transform:
            a_img = self.data_transform(a_img)
            p_img = self.data_transform(p_img)
        else:
            a_img = transforms.ToTensor()(a_img)
            p_img = transforms.ToTensor()(p_img)

        a_img = F.interpolate(a_img.unsqueeze(0), self.image_size).squeeze(0)
        p_img = F.interpolate(p_img.unsqueeze(0), self.image_size).squeeze(0)

        return cid, a_img, p_img
