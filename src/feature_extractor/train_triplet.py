import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
from model import FeatureDetectorNet
from datasets import TripletDataset
from torch.utils.data import DataLoader
import torch.optim as optim
from torchvision import transforms, datasets
import os
import math
from torch.utils.tensorboard import SummaryWriter
import matplotlib.pyplot as plt


def train_triplet(dataset_dir, weights_dir=None, run_name="run1", image_size=None, epochs=30, on_gpu=True, checkpoint_dir="checkpoints", batch_size=24):
    dataset = TripletDataset(dataset_dir)
    writer = SummaryWriter(f"runs/triplet_{run_name}")
    data_transform = transforms.Compose([
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])
    dataset = TripletDataset(dataset_dir, data_transform=data_transform) if not image_size else \
        TripletDataset(dataset_dir, image_size, data_transform=data_transform)
    dataset_length = len(dataset)
    dataloader = DataLoader(dataset, shuffle=True, batch_size=batch_size, num_workers=4)
    model = FeatureDetectorNet(use_classifier=False)
    if weights_dir:
        print(f"Continuing training using weights {weights_dir}")
        model.load_state_dict(torch.load(weights_dir))
    example_input = None
    for data in dataloader:
        example_input, _, _ = data
        break
    writer.add_graph(model, example_input)
    for param in model.backbone.parameters():
        param.requires_grad = False
    criterion = nn.TripletMarginLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.0005, weight_decay=0.01)  # 0.001 default

    #print(f"Training with {train_length} train images, and {test_length} test images")
    if on_gpu:
        model = model.cuda()
    running_loss = 0.0
    last_loss = 0.0
    mini_batches = 0
    for epoch in range(epochs):
        model.train()
        for i, data in enumerate(dataloader, 0):
            anchor, positive, negative = data
            if on_gpu:
                anchor = anchor.cuda()
                positive = positive.cuda()
                negative = negative.cuda()

            optimizer.zero_grad()
            anchor_embeddings = model(anchor)
            anchor_embeddings = F.normalize(anchor_embeddings, p=2) # L2 normalization so embeddings live inside unit hyper-sphere
            positive_embeddings = model(positive)
            positive_embeddings = F.normalize(positive_embeddings, p=2)
            negative_embeddings = model(negative)
            negative_embeddings = F.normalize(negative_embeddings, p=2)

            # TODO: Choose triplets based on embeddings

            loss = criterion(anchor_embeddings, positive_embeddings, negative_embeddings)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()

            if i % 5 == 4:
                loss = running_loss / 5
                last_loss = loss
                print(f"[{epoch + 1}, {i + 1}] loss: {loss:.6f}")
                running_loss = 0.0
                writer.add_scalar("training loss", loss, mini_batches)
            mini_batches += 1

        checkpoint_name = f"triplet-epoch-{epoch}-loss-{last_loss:.5f}.pth"
        checkpoint_full_name = os.path.join(checkpoint_dir, checkpoint_name)
        print(f"[{epoch + 1}] Saving checkpoint as {checkpoint_full_name}")
        torch.save(model.state_dict(), checkpoint_full_name)
        dataset.sample_triplets() # get new triplets
    print("Finished training")
    writer.close()