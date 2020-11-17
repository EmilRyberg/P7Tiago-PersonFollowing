import torch
import torch.nn.functional as F
import numpy as np
import torchvision.transforms as transforms
import cv2 as cv
from person_detector.person_finder.yolo_model import YOLOv3
from person_detector.person_finder.yolo_utils import non_max_suppression, rescale_boxes, load_classes
import argparse
from glob import glob
import os


def pad_to_square(img, pad_value):
    c, h, w = img.shape
    dim_diff = np.abs(h - w)
    # (upper / left) padding and (lower / right) padding
    pad1, pad2 = dim_diff // 2, dim_diff - dim_diff // 2
    # Determine padding
    pad = (0, 0, pad1, pad2) if h <= w else (pad1, pad2, 0, 0)
    # Add padding
    img = F.pad(img, pad, "constant", value=pad_value)

    return img, pad


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset_dir", metavar="dataset_dir", type=str, default="dataset", help="the dataset directory")
    parser.add_argument("-o", "--output_dir", type=str, default="dataset_processed", help="the output directory of the cropped dataset")
    parser.add_argument("-w", "--weights", type=str, default="yolov3.weights", help="path to weight file")
    parser.add_argument("-n", "--names", type=str, default="data/coco.names", help="path to names file")
    args = parser.parse_args()
    print(args)
    dataset_dir = args.dataset_dir
    output_dir = args.output_dir
    weight_path = args.weights
    dev = torch.device('cuda')
    cpu = torch.device('cpu')
    yolo = YOLOv3(80)
    yolo.load_weights(weight_path)
    yolo.eval()
    yolo = yolo.to(dev)
    classes = load_classes(args.names)

    for class_folder in glob(f"{dataset_dir}/*/"):
        class_name = os.path.basename(os.path.normpath(class_folder))
        print(f"class: {class_name}")
        class_output_dir = os.path.join(output_dir, class_name)
        if not os.path.isdir(class_output_dir):
            os.makedirs(class_output_dir)
        png_paths = glob(f"{class_folder}*.png")
        jpg_paths = glob(f"{class_folder}*.jpg")
        all_images = []
        all_images.extend(png_paths)
        all_images.extend(jpg_paths)
        for image_path in all_images:
            img_base_name = os.path.basename(image_path)
            img_name_split = os.path.splitext(img_base_name)
            cv_img = cv.imread(image_path)
            np_image = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
            p_img = transforms.ToTensor()(np_image)
            square_img, _ = pad_to_square(p_img, 0)
            p_img = F.interpolate(square_img.unsqueeze(0), size=416, mode="nearest").squeeze(0)
            p_img = p_img.unsqueeze(0)
            p_img = p_img.to(dev)
            results = yolo(p_img)
            detections = non_max_suppression(results, 0.4)
            detections = detections[0]
            if detections is not None:
                detections = rescale_boxes(detections, 416, np_image.shape[:2])
                person_dets = [det for det in detections if classes[int(det[6])].lower() == "person"]
                extra_path = os.path.join(class_output_dir, "extra")
                if len(person_dets) == 0:
                    print("no person in image")
                    continue
                if len(person_dets) > 1:
                    print("found more than 1 person in image")
                    if not os.path.isdir(extra_path):
                        os.mkdir(extra_path)
                for i, person_det in enumerate(person_dets):
                    x1, y1, x2, y2, conf, cls_conf, cls_pred = person_det
                    box_w = x2 - x1
                    box_h = y2 - y1
                    aspect = box_h / box_w
                    # print(f"aspect: {aspect}")
                    # if aspect < 1.5:
                    #     print("aspect ratio below threshold - skipping")
                    #     continue
                    #print(person_det[0])
                    ix1 = max(0, int(x1))
                    ix2 = max(0, int(x2))
                    iy1 = max(0, int(y1))
                    iy2 = max(0, int(y2))
                    cropped_img = np.array(np_image[iy1:iy2, ix1:ix2, :])
                    #print(square_img.shape)
                    #print(cropped_img.shape)
                    cropped_img = cv.cvtColor(cropped_img, cv.COLOR_RGB2BGR)
                    img_file_name = f"{img_name_split[0]}-cropped{img_name_split[1]}" if i == 0 else f"{img_name_split[0]}-{i+1}-cropped{img_name_split[1]}"
                    img_save_name = os.path.join(class_output_dir, img_file_name) if i == 0 else os.path.join(extra_path, img_file_name)
                    cv.imwrite(img_save_name, cropped_img)
                    print(f"Saving image as {img_save_name}")

    print("Done")
