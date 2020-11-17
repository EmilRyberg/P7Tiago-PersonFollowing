from person_detector.feature_extractor.train_triplet import train_triplet
from person_detector.feature_extractor.train_softmax import train_softmax

if __name__ == "__main__":
    #train_softmax("dataset", checkpoint_dir="checkpoints_ir_2", epochs=50, run_name="run2_inceptionresnetv2", on_gpu=True, batch_size=10)
    train_triplet("dataset", weights_dir="triplet-epoch-49-loss-0.01593.pth", epochs=50,
                  checkpoint_dir="checkpoints_triplet", run_name="run1", on_gpu=True)
