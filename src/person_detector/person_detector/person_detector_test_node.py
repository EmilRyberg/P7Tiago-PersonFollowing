import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
import os
from cv_bridge import CvBridge
import cv2 as cv
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.person_finder.person_finder import PersonFinder
from person_detector.feature_extractor.utils import plot_person_detections


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
        self.depth_subscriber = self.create_subscription(Image,
                                                         "/depth",
                                                         self.depth_callback,
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
        self.person_id = 0
        self.get_logger().info("Node started")

    def image_callback(self, msg: CompressedImage):
        self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_stamp = msg.header.stamp
        self.image_is_updated = True
        self.got_image_callback()

    def depth_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #self.get_logger().info(f"got depth image {self.depth_image.shape}")
        self.depth_stamp = msg.header.stamp
        self.depth_is_updated = True
        self.got_image_callback()

    def got_image_callback(self):
        if not self.image_is_updated or not self.depth_is_updated:
            return
        self.image_is_updated = False
        self.depth_is_updated = False

        person_detections = self.person_finder.find_persons(self.image)
        person_detection_mapping = []
        for person_detection in person_detections:
            cropped_person_img = self.person_finder.crop_bounding_box(self.image, person_detection)
            features = self.feature_extractor.get_features(cropped_person_img)
            if len(self.person_features_mapping) == 0:
                self.person_features_mapping.append((self.person_id, features))
                person_detection_mapping.append((self.person_id, person_detection))
                self.person_id += 1
            else:
                found_same_person = False
                for pid, emb in self.person_features_mapping:
                    distance = embedding_distance(features, emb)
                    # print(f"Distance: {distance}")
                    same_person = is_same_person(features, emb, threshold=1.2)
                    if same_person:
                        found_same_person = True
                        person_detection_mapping.append((pid, person_detection))
                        break
                if not found_same_person:
                    self.person_features_mapping.append((self.person_id, features))
                    person_detection_mapping.append((person_detection, self.person_id))
                    self.person_id += 1
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
