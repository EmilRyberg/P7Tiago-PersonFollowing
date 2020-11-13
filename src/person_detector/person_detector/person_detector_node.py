import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import CompressedImage
from person_follower_interfaces.msg import PersonInfo, PersonInfoList
import numpy
import torch
import os
from cv_bridge import CvBridge
from person_detector.feature_extractor_module.feature_extractor import FeatureExtractor

from std_msgs.msg import String


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")
        self.declare_parameter("feature_weights_path")
        feature_weights_path = self.get_parameter("feature_weights_path").get_parameter_value().string_value
        feature_weights_path = os.path.expanduser(feature_weights_path)
        self.image_subscriber = self.create_subscription(CompressedImage,
                                                         "/xtion/rgb/image_raw/compressed",
                                                         self.image_callback,
                                                         1)
        self.fe = FeatureExtractor(feature_weights_path, on_gpu=False)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = PersonDetector()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
