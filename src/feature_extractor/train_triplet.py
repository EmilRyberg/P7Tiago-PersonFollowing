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
import random


def train_triplet(dataset_dir, weights_dir=None, run_name="run1", image_size=None, epochs=30, on_gpu=True, checkpoint_dir="checkpoints_triplet", batch_size=50):
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
    if not os.path.isdir(checkpoint_dir):
        os.makedirs(checkpoint_dir)
    for data in dataloader:
        _, example_input, _ = data
        break
    writer.add_graph(model, example_input)
    for param in model.backbone.parameters():
        param.requires_grad = False
    criterion = nn.TripletMarginLoss(margin=0.2)
    optimizer = torch.optim.SGD(model.parameters(), lr=0.25, weight_decay=0.001, momentum=0.9)

    #print(f"Training with {train_length} train images, and {test_length} test images")
    if on_gpu:
        model = model.cuda()
    running_loss = 0.0
    last_loss = 0.0
    mini_batches = 0
    epoch_loss = 0.0
    epoch_mini_batches = 0
    for epoch in range(epochs):
        model.train()
        for i, data in enumerate(dataloader, 0):
            class_ids, anchor, positive = data
            if on_gpu:
                anchor = anchor.cuda()
                positive = positive.cuda()
                #negative = negative.cuda()

            print("running model")
            optimizer.zero_grad()
            anchor_embeddings = model(anchor)
            anchor_embeddings = F.normalize(anchor_embeddings, p=2) # L2 normalization so embeddings live inside unit hyper-sphere
            positive_embeddings = model(positive)
            positive_embeddings = F.normalize(positive_embeddings, p=2)
            #negative_embeddings = model(negative)
            #negative_embeddings = F.normalize(negative_embeddings, p=2)

            # resample triplets based on embeddings to train on the hardest negative embeddings
            class_ids_np = class_ids.detach().cpu().data.numpy()
            anchor_embeddings_np = anchor_embeddings.detach().cpu().data.numpy()
            positive_embeddings_np = positive_embeddings.detach().cpu().data.numpy()
            #negative_embeddings_np = negative_embeddings.detach().cpu().data.numpy()
            print("Resampling embeddings")
            #negative_np = negative.detach().cpu().data.numpy()
            all_images = torch.cat((anchor, positive), dim=0)
            print(f"all images: {all_images.shape}")
            all_images_np = all_images.detach().cpu().data.numpy()
            new_negatives = resample_triplets(class_ids_np, anchor_embeddings_np, positive_embeddings_np, all_images_np)
            if on_gpu:
                new_negatives = new_negatives.cuda()
            new_negative_embeddings = model(new_negatives)
            new_negative_embeddings = F.normalize(new_negative_embeddings, p=2)

            loss = criterion(anchor_embeddings, positive_embeddings, new_negative_embeddings)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            epoch_loss += loss.item()

            avg_loss = running_loss
            last_loss = avg_loss
            print(f"[{epoch + 1}, {i + 1}] loss: {avg_loss:.10f}")
            running_loss = 0.0
            mini_batches += 1
            epoch_mini_batches += 1
        avg_epoch_loss = epoch_loss / epoch_mini_batches
        epoch_mini_batches = 0
        print(f"[{epoch + 1}] loss: {avg_epoch_loss:.10f}")
        writer.add_scalar("training loss", avg_epoch_loss, mini_batches)

        checkpoint_name = f"triplet-epoch-{epoch}-loss-{avg_epoch_loss:.5f}.pth"
        checkpoint_full_name = os.path.join(checkpoint_dir, checkpoint_name)
        print(f"[{epoch + 1}] Saving checkpoint as {checkpoint_full_name}")
        torch.save(model.state_dict(), checkpoint_full_name)
        dataset.sample_triplets()  # get new triplets
        epoch_loss = 0
    print("Finished training")
    writer.close()


def resample_triplets(class_id_np, anchor_emb, positive_emb, all_images_np, alpha=0.2):
    new_negatives = []
    class_id_np_2x = np.tile(np.expand_dims(class_id_np, axis=1), (2, 1))
    all_embeddings_np = np.concatenate((anchor_emb, positive_emb), axis=0)
    for i, (cid, a_emb, p_emb) in enumerate(zip(class_id_np, anchor_emb, positive_emb)):
        pos_dist = np.linalg.norm(a_emb - p_emb)
        negatives_np, negative_emb = get_all_other_images_and_embeddings(class_id_np_2x,
                                                                         all_images_np, all_embeddings_np, cid)
        a_repeat_emb = np.tile(a_emb, (negative_emb.shape[0], 1))
        neg_dist = np.linalg.norm(a_repeat_emb - negative_emb, axis=1).reshape((-1, 1))
        pos_dist_repeat = np.tile(pos_dist, (negative_emb.shape[0], 1))
        neg_indices, _ = np.nonzero(neg_dist - pos_dist_repeat < alpha)
        if neg_indices.shape[0] == 0:
            #print("WARNING: No embedding distance below alpha, picking random triplets")
            batch_len = negatives_np.shape[0]
            r_idx = random.randint(0, batch_len-1)
            new_negatives.append(negatives_np[r_idx])
        else:
            np.random.shuffle(neg_indices)
            neg_index = neg_indices[0]
            new_negatives.append(negatives_np[neg_index])

    return torch.from_numpy(np.array(new_negatives))


def get_all_other_images_and_embeddings(class_ids_np, all_images_np, all_embeddings_np, current_class_id):
    images = []
    embeddings = []
    for cid, image, emb in zip(class_ids_np, all_images_np, all_embeddings_np):
        if cid != current_class_id:
            embeddings.append(emb)
            images.append(image)
    return np.array(images), np.array(embeddings)