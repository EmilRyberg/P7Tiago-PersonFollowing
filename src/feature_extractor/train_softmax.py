import numpy as np
import torch
import torch.nn as nn
import torchvision

from model import FeatureDetectorNet
from datasets import ClassificationDataset
from torch.utils.data import DataLoader
import torch.optim as optim
from torchvision import transforms, datasets
import os
import math
from torch.utils.tensorboard import SummaryWriter
import matplotlib.pyplot as plt


def matplotlib_imshow(img, one_channel=False):
    if one_channel:
        img = img.mean(dim=0)
    img = (img / 2 + 0.5)*255     # unnormalize
    npimg = img.numpy()
    if one_channel:
        plt.imshow(npimg, cmap="Greys")
    else:
        plt.imshow(np.transpose(npimg, (1, 2, 0)))


def train_softmax(dataset_dir, run_name="run1", image_size=None, epochs=30, on_gpu=True, checkpoint_dir="checkpoints", batch_size=24):
    writer = SummaryWriter(f"runs/{run_name}")
    data_transform = transforms.Compose([
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])
    dataset = ClassificationDataset(dataset_dir, data_transform=data_transform) if not image_size else \
        ClassificationDataset(dataset_dir, image_size, data_transform=data_transform)
    dataset_length = len(dataset)
    train_length = int(math.ceil(dataset_length * 0.7))
    test_length = dataset_length - train_length
    train_set, test_set = torch.utils.data.random_split(dataset, [train_length, test_length])
    dataloader = DataLoader(train_set, shuffle=True, batch_size=batch_size, num_workers=4)
    test_dataloader = DataLoader(test_set, shuffle=False, batch_size=batch_size, num_workers=4)
    model = FeatureDetectorNet(use_classifier=True)
    example_input = None
    for data in dataloader:
        images, labels = data
        example_input = images
        break
    writer.add_graph(model, example_input)
    # freeze_index = 0
    # for param in model.backbone.parameters():
    #     if freeze_index >= 6:
    #         break
    #     param.requires_grad = False
    #     freeze_index += 1
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.0005) # 0.001 default

    print(f"Training with {train_length} train images, and {test_length} test images")
    if on_gpu:
        model = model.cuda()
    running_loss = 0.0
    for epoch in range(epochs):
        model.train()
        for i, data in enumerate(dataloader, 0):
            inputs, labels = data
            if on_gpu:
                inputs = inputs.cuda()
                labels = labels.cuda()

            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            if i % 5 == 4:
                loss = running_loss/5
                print(f"[{epoch + 1}, {i + 1}] loss: {loss:.6f}")
                running_loss = 0.0
                writer.add_scalar("training loss", loss, epoch * len(dataloader) + i)

        test_loss = 0
        test_correct = 0
        total_img = 0
        for i, data in enumerate(test_dataloader, 0):
            model.eval()
            with torch.no_grad():
                inputs, labels = data
                if on_gpu:
                    inputs = inputs.cuda()
                    labels = labels.cuda()
                outputs = model(inputs)
                loss = criterion(outputs, labels)
                test_loss += loss
                softmax_output = torch.nn.functional.softmax(outputs, dim=0)
                output_np = softmax_output.cpu().data.numpy()
                predicted_ids = output_np.argmax(1)
                np_labels = labels.cpu().data.numpy()
                total_img += np_labels.shape[0]
                correct_labels = np_labels == predicted_ids
                sum_correct_labels = correct_labels.sum()
                test_correct += sum_correct_labels
        test_loss = test_loss / batch_size
        test_acc = (test_correct / test_length) * 100.0
        print(f"[{epoch}] Test loss: {test_loss:.5f}, test acc.: {test_acc:.3f}%")
        writer.add_scalar("test loss", test_loss, epoch + 1)
        writer.add_scalar("test accuracy", test_acc, epoch + 1)

        checkpoint_name = f"epoch-{epoch}-loss-{test_loss:.5f}-acc-{test_acc:.2f}.pth"
        checkpoint_full_name = os.path.join(checkpoint_dir, checkpoint_name)
        print(f"[{epoch + 1}] Saving checkpoint as {checkpoint_full_name}")
        torch.save(model.state_dict(), checkpoint_full_name)
    print("Finished training")
    writer.close()
