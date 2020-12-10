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

UNCONFIRMED_COUNT_THRESHOLD = 20
UNCONFIRMED_TIME_THRESHOLD_SECONDS = 6  # secs


class PersonDetectorTest(Node):
    def __init__(self):
        super().__init__("person_detector_test")
        self.declare_parameter("feature_weights_path")
        self.declare_parameter("yolo_weights_path")
        self.declare_parameter("on_gpu", value=True)
        self.declare_parameter("log_file_path", "feature_log.txt")
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        feature_weights_path = self.get_parameter("feature_weights_path").get_parameter_value().string_value
        feature_weights_path = os.path.expanduser(feature_weights_path)
        yolo_weights_path = self.get_parameter("yolo_weights_path").get_parameter_value().string_value
        yolo_weights_path = os.path.expanduser(yolo_weights_path)
        on_gpu = self.get_parameter("on_gpu").get_parameter_value().bool_value
        self.log_file_path = self.get_parameter("log_file_path").get_parameter_value().string_value
        self.image_subscriber = self.create_subscription(CompressedImage,
                                                         "/compressed_images",
                                                         self.image_callback,
                                                         qos_profile)
        self.feature_extractor = FeatureExtractor(feature_weights_path, on_gpu=on_gpu)
        self.person_finder = PersonFinder(yolo_weights_path, on_gpu=on_gpu)
        self.cv_bridge = CvBridge()
        self.image = None
        self.start_node = True

        self.batch_size = 20
        self.feature_count = 0
        self.feature_buffer = []
        self.all_features = []

        if os.path.isfile(self.log_file_path):
            self.get_logger().error(f"Log file already exists, aborting.")
            self.start_node = False

        self.get_logger().info("Node started")

    def image_callback(self, msg: CompressedImage):
        self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_stamp = msg.header.stamp
        self.got_image_callback()

    def got_image_callback(self):
        person_detections = self.person_finder.find_persons(self.image)
        person_detection_mapping = []
        first_detection = next((person for person in person_detections), None)
        if first_detection is not None:
            cropped_person_img = self.person_finder.crop_bounding_box(self.image, first_detection)
            features = self.feature_extractor.get_features(cropped_person_img)
            self.feature_buffer.append(features)
            self.all_features.append(features)
            person_detection_mapping.append((first_detection, 0))
            self.feature_count += 1
            if self.feature_count % self.batch_size == 0:
                self.get_logger().info(f"Writing to log")
                seperator = "\t"
                with open(self.log_file_path, "a+") as file:
                    for feature in self.feature_buffer:
                        feature_string = ""
                        for i in range(feature.shape[1]):
                            feature_string += f"{feature[0, i]:.4f}{seperator}"
                        feature_string += "\n"
                        file.write(feature_string)
                    self.feature_buffer = []
                self.get_logger().info(f"Calculating distances")
                max_distance = 0
                min_distance = 3
                distances = []
                for i, feature in enumerate(self.all_features):
                    #self.get_logger().info(f"Outer: {i+1}/{len(self.all_features)}")
                    other_features = [f for si, f in enumerate(self.all_features) if si != i]
                    for ci, other_feature in enumerate(other_features):
                        #self.get_logger().info(f"\tInner: {ci + 1}/{len(other_features)}")
                        distance = embedding_distance(feature, other_feature)
                        if distance > max_distance:
                            max_distance = distance
                        elif distance < min_distance:
                            min_distance = distance
                        distances.append(distance)
                distances = np.array(distances)
                average_distance = distances.mean()
                median_distance = np.median(distances)
                self.get_logger().info(f"Min dist: {min_distance:.4f}, max: {max_distance:.4f}, average: {average_distance}, median: {median_distance}")


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
