import numpy as np
import torch
import torch.nn as nn

from person_detector.feature_extractor.model import FeatureExtractorNet
from person_detector.feature_extractor.datasets import ClassificationDataset
from torch.utils.data import DataLoader
from torchvision import transforms
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


# function for first step in training, train a classifier
def train_softmax(dataset_dir, weights_dir=None, run_name="run1", image_size=None, epochs=30,
                  on_gpu=True, checkpoint_dir="checkpoints", batch_size=24, print_interval=50, num_classes=41):
    # Just a writer that writes for TensorBoard
    writer = SummaryWriter(f"runs/{run_name}")
    # Transforms applied to images
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
    model = FeatureExtractorNet(use_classifier=True, num_classes=num_classes)
    if not os.path.isdir(checkpoint_dir):
        os.makedirs(checkpoint_dir)
    if weights_dir:
        print(f"Continuing training using weights {weights_dir}")
        model.load_state_dict(torch.load(weights_dir))
    example_input = None
    for data in dataloader:
        images, super_class_labels, labels = data
        example_input = images
        break
    writer.add_graph(model, example_input)
    # freeze_index = 0
    # for param in model.backbone.parameters():
    #     if freeze_index >= 10:
    #         break
    #     param.requires_grad = False
    #     freeze_index += 1
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.0005, weight_decay=0.01) # lr 0.001 default

    print(f"Training with {train_length} train images, and {test_length} test images")
    if on_gpu:
        model = model.cuda()
    running_loss = 0.0
    # here we start training
    for epoch in range(epochs):
        model.train()
        for i, data in enumerate(dataloader, 0):
            inputs, super_class_labels, labels = data
            if on_gpu:
                inputs = inputs.cuda()
                super_class_labels = super_class_labels.cuda()
                labels = labels.cuda()

            optimizer.zero_grad()
            outputs = model(inputs)
            # split outputs so we can compute super-class loss and sub-class loss
            super_class_outputs = outputs[:, :6]
            specific_class_outputs = outputs[:, 6:]
            super_class_loss = criterion(super_class_outputs, super_class_labels)
            sub_class_loss = criterion(specific_class_outputs, labels)
            total_loss = super_class_loss + sub_class_loss
            total_loss.backward()
            optimizer.step()
            running_loss += total_loss.item()
            if i % print_interval == print_interval-1:
                loss = running_loss/print_interval
                print(f"[{epoch + 1}, {i + 1}] loss: {loss:.6f}")
                running_loss = 0.0
                writer.add_scalar("training loss", loss, epoch * len(dataloader) + i)

        test_loss = 0
        super_class_test_correct = 0
        sub_class_test_correct = 0
        total_test_correct = 0
        total_img = 0
        total_runs = 0
        print("Running on test set")
        for i, data in enumerate(test_dataloader, 0):
            model.eval()
            with torch.no_grad():
                inputs, super_class_labels, labels = data
                if on_gpu:
                    inputs = inputs.cuda()
                    super_class_labels = super_class_labels.cuda()
                    labels = labels.cuda()
                outputs = model(inputs)
                super_class_outputs = outputs[:, :6]
                specific_class_outputs = outputs[:, 6:]
                super_class_loss = criterion(super_class_outputs, super_class_labels)
                sub_class_loss = criterion(specific_class_outputs, labels)
                total_loss = super_class_loss + sub_class_loss
                test_loss += total_loss.item()

                # super class acc
                super_class_softmax_output = torch.nn.functional.softmax(super_class_outputs, dim=0)
                super_class_output_np = super_class_softmax_output.cpu().data.numpy()
                super_class_predicted_ids = super_class_output_np.argmax(1)
                super_class_labels_np = super_class_labels.cpu().data.numpy()
                total_img += super_class_labels_np.shape[0]
                super_class_correct_labels = super_class_labels_np == super_class_predicted_ids
                super_class_sum_correct_labels = super_class_correct_labels.sum()
                super_class_test_correct += super_class_sum_correct_labels

                # sub class acc
                sub_class_softmax_output = torch.nn.functional.softmax(specific_class_outputs, dim=0)
                sub_class_output_np = sub_class_softmax_output.cpu().data.numpy()
                sub_class_predicted_ids = sub_class_output_np.argmax(1)
                sub_class_labels_np = labels.cpu().data.numpy()
                sub_class_correct_labels = sub_class_labels_np == sub_class_predicted_ids
                sub_class_sum_correct_labels = sub_class_correct_labels.sum()
                sub_class_test_correct += sub_class_sum_correct_labels

                # total acc
                total_correct = np.logical_and(super_class_correct_labels, sub_class_correct_labels)
                total_correct_sum = total_correct.sum()
                total_test_correct += total_correct_sum

                total_runs += 1
        avg_test_loss = test_loss / total_runs
        super_class_test_acc = (super_class_test_correct / test_length) * 100.0
        sub_class_test_acc = (sub_class_test_correct / test_length) * 100.0
        overall_test_acc = (total_test_correct / test_length) * 100.0
        print(f"[{epoch + 1}] Test loss: {avg_test_loss:.5f}")
        print(f"[{epoch + 1}] Super class test acc.: {super_class_test_acc:.3f}%")
        print(f"[{epoch + 1}] Sub class test acc.: {sub_class_test_acc:.3f}%")
        print(f"[{epoch + 1}] Overall test acc.: {overall_test_acc:.3f}%")
        writer.add_scalar("test loss", avg_test_loss, epoch + 1)
        writer.add_scalar("super class test accuracy", super_class_test_acc, epoch + 1)
        writer.add_scalar("sub class test accuracy", sub_class_test_acc, epoch + 1)
        writer.add_scalar("overall test accuracy", overall_test_acc, epoch + 1)

        checkpoint_name = f"epoch-{epoch + 1}-loss-{avg_test_loss:.5f}-{super_class_test_acc:.2f}-{sub_class_test_acc:.2f}-{overall_test_acc:.2f}.pth"
        checkpoint_full_name = os.path.join(checkpoint_dir, checkpoint_name)
        print(f"[{epoch + 1}] Saving checkpoint as {checkpoint_full_name}")
        torch.save(model.state_dict(), checkpoint_full_name)
    print("Finished training")
    writer.close()
