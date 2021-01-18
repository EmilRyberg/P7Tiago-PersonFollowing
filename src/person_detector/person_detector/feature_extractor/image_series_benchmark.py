import numpy as np
from person_detector.person_finder.person_finder import PersonFinder
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from glob import glob
import cv2 as cv
from PIL import Image
import torch
from torchvision import transforms
from torch import Tensor
from person_detector.feature_extractor.utils import plot_person_detections
import os


class Benchmark:
    def __init__(self, image_folder, feature_weights, yolo_weights):
        self.class_folders = glob(image_folder + "/*/")
        self.images_with_class = []
        self.feature_extractor = FeatureExtractor(feature_weights)
        self.person_detector = PersonFinder(yolo_weights)
        self.save_folder = "detections"
        class_id = 0
        for folder in self.class_folders:
            png_paths = glob(f"{folder}*.png")
            jpg_paths = glob(f"{folder}*.jpg")
            all_images = []
            all_images.extend(png_paths)
            all_images.extend(jpg_paths)
            self.images_with_class.append((class_id, all_images))
            class_id += 1

    def run(self):
        batch_size = 20
        person_feature_mapping = []
        person_id = 0
        for cid, img_paths in self.images_with_class:
            for i in range(0, len(img_paths)-batch_size, batch_size):
                print(f"Taking images {i}-{i+batch_size}")
                #batch_img: Tensor = torch.empty(1, 1, 1)
                for img_i in range(i, i+batch_size, 1):
                    detected_id = None
                    #img = cv.imread(img_paths[img_i])
                    p_img = Image.open(img_paths[img_i])
                    #tensor_img = transforms.ToTensor()(img)
                    #tensor_img: Tensor = transforms.ToTensor()(p_img)
                    #if batch_img is None:
                    #    batch_img = tensor_img
                    #else:
                    #    torch.cat((batch_img, tensor_img), dim=0)

                    np_img = np.array(p_img)
                    person_detections = self.person_detector.find_persons(np_img)
                    if len(person_detections) == 0:
                        continue
                    first_det = person_detections[0]
                    cropped_img = self.person_detector.crop_bounding_box(np_img, first_det)
                    features = self.feature_extractor.get_features(cropped_img)
                    if len(person_feature_mapping) == 0:
                        person_feature_mapping.append((person_id, features))
                        detected_id = person_id
                        person_id += 1
                    else:
                        for pid, other_features in person_feature_mapping:
                            same_person = is_same_person(features, other_features, threshold=0.8)
                            dist = embedding_distance(features, other_features)
                            print(f"Dist to {pid}: {dist:.4f}")
                            if same_person:
                                detected_id = pid
                                break

                    if detected_id is None:
                        print("Found new person")
                        person_feature_mapping.append((person_id, features))
                        detected_id = person_id
                        person_id += 1

                    print(f"CID: {cid}, img: {img_i} got id: {detected_id}")
                    img_with_bbox = plot_person_detections([(first_det, detected_id)], np_img, input_is_rgb=True)
                    save_name = os.path.join(self.save_folder, f"{cid}-{img_i}.png")
                    print(f"Saving as: {save_name}")
                    cv.imwrite(save_name, img_with_bbox)


if __name__ == "__main__":
    b = Benchmark("benchmark_set", "triplet-epoch-25-loss-0.00096.pth", "yolov3.weights")
    b.run()

