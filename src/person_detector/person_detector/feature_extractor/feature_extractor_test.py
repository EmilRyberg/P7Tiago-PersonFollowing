from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.feature_extractor.utils import plot_person_detections
from person_detector.person_finder.person_finder import PersonFinder
import cv2 as cv

if __name__ == "__main__":
    fe = FeatureExtractor("triplet_weights.pth")
    pf = PersonFinder("yolov3.weights")

    cap = cv.VideoCapture(0)
    embeddings = []
    person_id = 0

    while True:
        ret, frame = cap.read()
        img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        detections = pf.find_persons(img)
        det_to_person_id = []
        #print(f"Found {len(detections)} persons")
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
                    distance = embedding_distance(features, emb)
                    print(f"Distance: {distance}")
                    same_person = is_same_person(features, emb, threshold=0.5)
                    if same_person:
                        found_same_person = True
                        det_to_person_id.append((det, pid))
                        break
                if not found_same_person:
                    print(f"Found new person")
                    embeddings.append((person_id, features))
                    det_to_person_id.append((det, person_id))
                    person_id += 1
        img_s = plot_person_detections(det_to_person_id, frame)
        if img_s is None:
            img_s = frame
        cv.imshow("det", img_s)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
