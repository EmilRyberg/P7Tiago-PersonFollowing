import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
import numpy
import torch
import os
from person_detector.feature_extractor_module.feature_extractor import FeatureExtractor

from std_msgs.msg import String


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.declare_parameter("feature_weights_path")
        feature_weights_path = self.get_parameter("feature_weights_path").get_parameter_value().string_value
        feature_weights_path = os.path.expanduser(feature_weights_path)
        print(f"Weights path: {feature_weights_path}")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.fe = FeatureExtractor(feature_weights_path, on_gpu=False)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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
