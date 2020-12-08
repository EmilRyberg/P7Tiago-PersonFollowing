import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose
from std_msgs.msg import Header
import os
from cv_bridge import CvBridge
import cv2
import tf2_ros
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.person_finder.person_finder import PersonFinder
from person_follower_interfaces.msg import PersonInfoList, PersonInfo
import numpy as np
import math
import struct



class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")
        self.declare_parameter("feature_weights_path")
        self.declare_parameter("yolo_weights_path")
        self.declare_parameter("on_gpu", value=True)
        qos_profile=QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.publisher_ = self.create_publisher(PersonInfoList, "/persons", 1)
        self.get_logger().info("Loading weights")
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

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, spin_thread=True, node=self)
        self.first_run = True
        self.found_transform = False
        self.last_error = None

        self.get_logger().info("Subscribing to topics")
        self.image_subscriber = self.create_subscription(CompressedImage,
                                                         "/compressed_images",
                                                         self.image_callback,
                                                         qos_profile)
        self.depth_subscriber = self.create_subscription(CompressedImage,
                                                         "/compressed_depth_images",
                                                         self.depth_callback,
                                                         qos_profile)
        self.get_logger().info("Node started")


    def image_callback(self, msg: CompressedImage):
        self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_stamp = msg.header.stamp
        self.image_is_updated = True
        #self.got_image_callback()

    def depth_callback(self, msg: CompressedImage):
        #self.depth_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info(f"got depth image")
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]
        raw_data=np.array(raw_data, dtype=np.uint8)
        depth_img_raw = cv2.imdecode(raw_data, cv2.IMREAD_UNCHANGED)
        raw_header = msg.data[:depth_header_size]
        # header: int, float, float
        [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
        depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32) - depthQuantB)
        # filter max values
        depth_img_scaled[depth_img_raw == 0] = 0
        self.depth_stamp = msg.header.stamp
        self.depth_is_updated = True
        #self.got_image_callback()

def main(args=None):
    rclpy.init(args=args)
    person_detector = PersonDetector()

    rclpy.spin(person_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    person_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()