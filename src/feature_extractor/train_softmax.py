import numpy as np
import torch
import torch.nn as nn
from model import FeatureDetectorNet
from datasets import ClassificationDataset
from torch.utils.data import DataLoader
import torch.optim as optim
from torchvision import transforms, datasets
import os


def train_softmax(dataset_dir, image_size, num_classes=5, epochs=30, on_gpu=True, checkpoint_dir="checkpoints"):
    dataset = ClassificationDataset(dataset_dir)
    gpu = torch.device("cuda")
    dataloader = DataLoader(dataset, shuffle=True, batch_size=12, num_workers=4)
    model = FeatureDetectorNet(use_classifier=True)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001) # 0.001 default

    if on_gpu:
        model = model.cuda()

    for epoch in range(epochs):
        running_loss = 0.0
        last_loss = 0.0
        for i, data in enumerate(dataloader, 0):
            inputs, labels = data
            if on_gpu:
                inputs = inputs.cuda()
                labels = inputs.cuda()

            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            if i % 10 == 9:
                last_loss = running_loss/10
                print(f"[{epoch + 1}, {i + 1}] loss: {last_loss}")
                running_loss = 0.0

        checkpoint_name = f"{epoch}-{last_loss}.pth"
        checkpoint_full_name = os.path.join(checkpoint_dir, checkpoint_name)
        print(f"[{epoch + 1}] Saving checkpoint as {checkpoint_full_name}")
        torch.save(model.state_dict(), checkpoint_full_name)
    print("Finished training")
