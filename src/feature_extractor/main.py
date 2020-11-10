import torch
import numpy as np
from train_softmax import train_softmax
from train_triplet import train_triplet

if __name__ == "__main__":
    #train_softmax("dataset", checkpoint_dir="checkpoints_ir_2", epochs=50, run_name="run2_inceptionresnetv2", on_gpu=True, batch_size=10)
    train_triplet("dataset", weights_dir="checkpoints_d2/epoch-47-loss-0.16744-acc-95.76.pth", epochs=50, checkpoint_dir="checkpoints_triplet")
