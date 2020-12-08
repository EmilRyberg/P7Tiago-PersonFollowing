import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
import os
from cv_bridge import CvBridge
import cv2 as cv
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.person_finder.person_finder import PersonFinder
from person_detector.feature_extractor.utils import plot_person_detections
import numpy as np

UNCONFIRMED_COUNT_THRESHOLD = 15
UNCONFIRMED_TIME_THRESHOLD_SECONDS = 30  # secs


class PersonDetectorTest(Node):
    def __init__(self):
        super().__init__("person_detector_test")
        self.declare_parameter("feature_weights_path")
        self.declare_parameter("yolo_weights_path")
        self.declare_parameter("on_gpu", value=True)
        qos_profile=QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        feature_weights_path = self.get_parameter("feature_weights_path").get_parameter_value().string_value
        feature_weights_path = os.path.expanduser(feature_weights_path)
        yolo_weights_path = self.get_parameter("yolo_weights_path").get_parameter_value().string_value
        yolo_weights_path = os.path.expanduser(yolo_weights_path)
        on_gpu = self.get_parameter("on_gpu").get_parameter_value().bool_value
        self.image_subscriber = self.create_subscription(CompressedImage,
                                                         "/compressed_images",
                                                         self.image_callback,
                                                         qos_profile)
        self.feature_extractor = FeatureExtractor(feature_weights_path, on_gpu=on_gpu)
        self.person_finder = PersonFinder(yolo_weights_path, on_gpu=on_gpu)
        self.cv_bridge = CvBridge()
        self.image = None
        self.depth_image = None
        self.image_stamp = None
        self.old_depth_stamp = None
        self.depth_stamp = None
        self.image_is_updated = False
        self.depth_is_updated = False

        self.person_features_mapping = []
        self.unconfirmed_persons = []
        self.person_id = 0
        self.get_logger().info("Node started")

    def image_callback(self, msg: CompressedImage):
        self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_stamp = msg.header.stamp
        self.image_is_updated = True
        self.got_image_callback()

    def got_image_callback(self):
        person_detections = self.person_finder.find_persons(self.image)
        person_detection_mapping = []
        for person_detection in person_detections:
            cropped_person_img = self.person_finder.crop_bounding_box(self.image, person_detection)
            features = self.feature_extractor.get_features(cropped_person_img)
            found_same_person = False
            features_below_threshold = []
            for i, (pid, emb) in enumerate(self.person_features_mapping):
                distance = embedding_distance(features, emb)
                self.get_logger().info(f"Distance to person {pid}: {distance:.5f}")
                # print(f"Distance: {distance}")
                is_below_threshold = is_same_person(features, emb, threshold=0.8)
                if is_below_threshold:
                    found_same_person = True
                    features_below_threshold.append((pid, distance, person_detection, i))
                    #person_id = pid
                    #person_detection_mapping.append((person_detection, pid))
            if found_same_person:
                min_distance = 3  # distance is between 0 and 2
                best_match = None
                best_index = None
                for pid, distance, person_detection, index in features_below_threshold:
                    if distance < min_distance:
                        min_distance = distance
                        best_match = (person_detection, pid)
                        best_index = index
                current_person_id, current_features = self.person_features_mapping[best_index]
                new_emb = current_features * 0.9 + features * 0.10
                diff_dist = embedding_distance(current_features, new_emb)
                #self.get_logger().info(f"Moving embedding, diff distance: {diff_dist}")
                self.person_features_mapping[best_index] = (best_match[1], new_emb)
                person_detection_mapping.append(best_match)
            else:
                indices_to_remove = []
                found_same_unconfirmed_person = False
                time_now = self.get_clock().now()
                unconfirmed_below_threshold = []
                for i, (emb, times_found, time_last_seen) in enumerate(self.unconfirmed_persons):
                    if time_now-time_last_seen > Duration(seconds=UNCONFIRMED_TIME_THRESHOLD_SECONDS):
                        self.get_logger().info(f"Removing index: {i}")
                        indices_to_remove.append(i)
                        continue
                    distance = embedding_distance(features, emb)
                    is_below_threshold = is_same_person(features, emb, threshold=0.8)
                    #self.get_logger().info(f"avg emb: {avg_emb}, original: {features}, org emb: {emb}")
                    if is_below_threshold:
                        unconfirmed_below_threshold.append((i, distance, (emb, times_found, time_last_seen)))
                min_distance = 3  # distance is between 0 and 2
                best_match = None
                for i, distance, info_tuple in unconfirmed_below_threshold:
                    if distance < min_distance:
                        min_distance = distance
                        best_match = (i, info_tuple)
                if best_match is not None:
                    best_index, (all_features, times_found, time_last_seen) = best_match
                    #avg_emb = (features + emb) / 2
                    all_features.append(features)
                    new_times_found = times_found + 1
                    if new_times_found >= UNCONFIRMED_COUNT_THRESHOLD:
                        indices_to_remove.append(best_index)
                        all_features_np = np.array(all_features).reshape((-1, 8))
                        median_embeddings = np.median(all_features_np, axis=0)
                        self.person_features_mapping.append((self.person_id, median_embeddings))
                        person_detection_mapping.append((person_detection, self.person_id))
                        self.person_id += 1
                    found_same_unconfirmed_person = True
                    self.unconfirmed_persons[best_index] = (all_features, new_times_found, time_now)
                if len(indices_to_remove) > 0:
                    self.unconfirmed_persons = [p for i, p in enumerate(self.unconfirmed_persons) if
                                                i not in indices_to_remove]
                    #self.get_logger().info(f"New list length: {len(self.unconfirmed_persons)}")
                if not found_same_unconfirmed_person:
                    self.unconfirmed_persons.append(([features], 0, time_now))
        img_drawn = plot_person_detections(person_detection_mapping, self.image)
        cv.imshow("Image", img_drawn)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    person_detector_test = PersonDetectorTest()

    rclpy.spin(person_detector_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    person_detector_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
