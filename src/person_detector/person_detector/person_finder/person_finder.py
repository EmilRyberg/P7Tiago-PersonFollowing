import torch.nn.functional as F
import numpy as np
import torchvision.transforms as transforms
from person_detector.person_finder.yolo_model import YOLOv3
from person_detector.person_finder.yolo_utils import non_max_suppression, rescale_boxes, load_classes


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


class PersonFinder:
    def __init__(self, yolo_weights_dir, on_gpu=True):
        self.model = YOLOv3(80)
        self.model.load_weights(yolo_weights_dir)
        self.model.eval()
        self.on_gpu = on_gpu
        if on_gpu:
            self.model.cuda()

    def crop_bounding_box(self, np_image, detection):
        x1, y1, x2, y2, conf, cls_conf, cls_pred = detection
        box_w = x2 - x1
        box_h = y2 - y1
        ix1 = max(0, int(x1))
        ix2 = max(0, int(x2))
        iy1 = max(0, int(y1))
        iy2 = max(0, int(y2))
        cropped_img = np.array(np_image[iy1:iy2, ix1:ix2, :])
        return cropped_img

    def find_persons(self, np_image):
        p_img = transforms.ToTensor()(np_image)
        square_img, _ = pad_to_square(p_img, 0)
        p_img = F.interpolate(square_img.unsqueeze(0), size=416, mode="nearest").squeeze(0)
        p_img = p_img.unsqueeze(0)
        if self.on_gpu:
            p_img = p_img.cuda()
        results = self.model(p_img)
        detections = non_max_suppression(results, 0.4)
        detections = detections[0]
        if detections is not None:
            detections = rescale_boxes(detections, 416, np_image.shape[:2])
            person_dets = [det for det in detections if int(det[6]) == 0]  # Detection of ID 0 is person
            return person_dets
        else:
            return []
