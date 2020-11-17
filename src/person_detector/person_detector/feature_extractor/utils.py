import cv2 as cv


def plot_person_detections(det_with_person_ids, img):
    # Rescale boxes to original image
    img_cp = img.copy()
    #classes = load_classes("data/coco.names")
    if det_with_person_ids is None:
        return None
    #detections = rescale_boxes(det, 416, img.shape[:2])
    #unique_labels = detections[:, -1].cpu().unique()
    #n_cls_preds = len(unique_labels)

    if det_with_person_ids is not None:
        for (x1, y1, x2, y2, conf, cls_conf, cls_pred), pid in det_with_person_ids:
            #print("\t+ Label: %s, Conf: %.5f" % (classes[int(cls_pred)], cls_conf.item()))
            # Create a Rectangle patch
            cv.rectangle(img_cp, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv.putText(img_cp, f"Person: {pid}", (x1 + 20, y1 + 20), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    return img_cp