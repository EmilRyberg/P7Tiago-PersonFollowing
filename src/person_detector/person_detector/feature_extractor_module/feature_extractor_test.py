from person_detector.feature_extractor_module.feature_extractor import FeatureExtractor
from person_detector.person_finder import PersonFinder
import cv2 as cv


def plot_detections(det_with_person_ids, img):
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


if __name__ == "__main__":
    fe = FeatureExtractor("triplet_weights.pth")
    pf = PersonFinder("yolov3.weights", "data/coco.names")

    cap = cv.VideoCapture(0)
    embeddings = []
    person_id = 0

    while True:
        ret, frame = cap.read()
        img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        detections = pf.find_persons(img)
        det_to_person_id = []
        print(f"Found {len(detections)} persons")
        for det in detections:
            cropped_person = pf.crop_bounding_box(img, det)
            features = fe.get_features(cropped_person)
            if len(embeddings) == 0:
                embeddings.append((person_id, features))
                det_to_person_id.append((det, person_id))
                person_id += 1
            else:
                found_same_person = False
                for pid, emb in embeddings:
                    distance = fe.embedding_distance(features, emb)
                    #print(f"Distance: {distance}")
                    is_same_person = fe.is_same_person(features, emb)
                    if is_same_person:
                        found_same_person = True
                        det_to_person_id.append((det, pid))
                if not found_same_person:
                    print(f"Found new person")
                    embeddings.append((person_id, features))
                    det_to_person_id.append((det, person_id))
                    person_id += 1
        img_s = plot_detections(det_to_person_id, frame)
        if img_s is None:
            img_s = frame
        cv.imshow("det", img_s)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()