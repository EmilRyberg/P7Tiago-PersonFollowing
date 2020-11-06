import torch
import numpy as np
from train_softmax import train_softmax
from train_triplet import train_triplet

if __name__ == "__main__":
    #train_softmax("dataset", epochs=50)
    train_triplet("dataset", weights_dir="checkpoints/epoch-49-loss-0.03688-acc-69.49.pth")
