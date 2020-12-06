from person_detector.feature_extractor.train_triplet import train_triplet
from person_detector.feature_extractor.train_softmax import train_softmax

if __name__ == "__main__":
    #train_softmax("dataset", checkpoint_dir="checkpoints_5_days_sub_class_2",
    #             epochs=50, run_name="run_5_days_sub_class_2", on_gpu=True, batch_size=64, print_interval=25, num_classes=34)
    train_triplet("dataset", weights_dir="checkpoints_5_days_sub_class_2/epoch-42-loss-0.04277-99.58-92.20-92.35.pth", epochs=10,
                   checkpoint_dir="checkpoints_triplet_5_days", run_name="run1_5_days", on_gpu=True, batch_size=150)
