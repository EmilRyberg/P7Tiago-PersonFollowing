from person_detector.feature_extractor.train_triplet import train_triplet
from person_detector.feature_extractor.train_softmax import train_softmax

if __name__ == "__main__":
    #train_softmax("dataset", checkpoint_dir="checkpoints_6_days_4_emb",
    #             epochs=60, run_name="run_6_days_4_emb", on_gpu=True, batch_size=64, print_interval=50, num_classes=41)
    train_triplet("dataset", weights_dir="checkpoints_6_days_4_emb/epoch-60-loss-0.12512-97.47-80.69-79.29.pth", epochs=20,
                   checkpoint_dir="checkpoints_triplet_6_days_4_emb_0.4mg", run_name="run1_6_days_4_emb_0.4mg", on_gpu=True, batch_size=150)
