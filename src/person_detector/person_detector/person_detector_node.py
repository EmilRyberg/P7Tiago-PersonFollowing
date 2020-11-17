import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import os
from cv_bridge import CvBridge
import tf2_ros
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.person_finder.person_finder import PersonFinder
from person_follower_interfaces.msg import PersonInfoList, PersonInfo
import numpy as np
import math
from typing import Optional

HFOV = 1.01229096615671
W = 640
VFOV = 0.785398163397448
H = 480


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")
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
        self.publisher_ = self.create_publisher(PersonInfoList, "/persons", 1)
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

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, spin_thread=True, node=self)
        self.first_run = True
        self.found_transform = False
        self.last_error = None
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
        persons = []
        for person_detection in person_detections:
            cropped_person_img = self.person_finder.crop_bounding_box(self.image, person_detection)
            features = self.feature_extractor.get_features(cropped_person_img)
            person_id = None
            if len(self.person_features_mapping) == 0:
                self.person_features_mapping.append((self.person_id, features))
                person_detection_mapping.append((self.person_id, person_detection))
                person_id = self.person_id
                self.person_id += 1
            else:
                found_same_person = False
                for pid, emb in self.person_features_mapping:
                    distance = embedding_distance(features, emb)
                    # print(f"Distance: {distance}")
                    same_person = is_same_person(features, emb, threshold=1.2)
                    if same_person:
                        found_same_person = True
                        person_id = pid
                        person_detection_mapping.append((pid, person_detection))
                        break
                if not found_same_person:
                    self.person_features_mapping.append((self.person_id, features))
                    person_id = self.person_id
                    person_detection_mapping.append((self.person_id, person_detection))
                    self.person_id += 1

            map_point = self.transform_image_to_map(bounding_box=person_detection)
            if map_point is None:
                self.get_logger().warn(f"Returned map_point is None, skipping person with ID: {person_id}")
            else:
                person = PersonInfo()
                person.person_id = person_id
                person.point = map_point
                persons.append(person)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 0 # TODO change this to the correct frame id of depth sensor
        person_info = PersonInfoList(header=header, persons=persons)
        self.publisher_.publish(person_info)

    def transform_image_to_map(self, bounding_box) -> Optional[Point]:
        stamp = self.depth_stamp

        if self.old_depth_stamp is None:
            self.old_depth_stamp = self.depth_stamp

        time_difference = self.depth_stamp.sec + self.depth_stamp.nanosec*1e-9 - (self.old_depth_stamp.sec+self.old_depth_stamp.nanosec*1e-9)
        if time_difference > 1:
            self.get_logger.error(f"Images and tf frames are delayed by more than 1s. Last error: {self.last_error}")
        elif time_difference > 0.5:
            self.get_logger.warn("Images and tf frames are delayed by more than 0.5s")

        try:
            transform = self.tf_buffer.lookup_transform('map', 'xtion_depth_frame', stamp)
        except tf2_ros.ExtrapolationException as e:
            self.last_error = e
            return None

        # Rotation and translation
        R = self.quaternion_to_rotation_matrix(transform)
        T = np.array([transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z])

        # Here I made some coefficients for a linear function for calculating the angle for each pixel
        ah = -HFOV / (W - 1)
        bh = HFOV / 2
        av = -VFOV / (H - 1)
        bv = VFOV / 2

        # Here, if we have a bounding box, we find the position of the object and publish it
        point = Point()
        # Getting the middle pixel of the bounding box
        # TODO replace with better method
        xp = int(bounding_box[0]) + int(bounding_box[2] / 2)
        yp = int(bounding_box[1]) + int(bounding_box[3] / 2)

        # The middle pixel can be outside the frame, so we stop that
        xp = W - 1 if xp > W else xp
        xp = 0 if xp < 0 else xp
        yp = H - 1 if yp > H else yp
        yp = 0 if yp < 0 else yp

        # Getting the distance to the target
        dist = self.depth_image.item(yp, xp)

        # We calculate the two angles for the pixel
        horizontal_angle = ah * xp + bh
        vertical_angle = av * yp + bv

        c_horizontal = math.cos(horizontal_angle)
        c_vertical = math.cos(vertical_angle)
        s_horizontal = math.sin(horizontal_angle)
        s_vertical = math.sin(vertical_angle)

        # We get the x, y, z in the camera frame
        v = np.array([c_horizontal * (dist * c_vertical),
                      s_horizontal * (dist * c_vertical),
                      s_vertical * (dist * c_horizontal)])

        # We transform it to map frame
        map_point = np.dot(R, v) + T
        point.x = map_point[0]
        point.y = map_point[1]
        point.z = map_point[2]

        self.old_depth_stamp = self.depth_stamp

        return point

    def quaternion_to_rotation_matrix(self, transform):
        w = transform.transform.rotation.w
        x = transform.transform.rotation.x
        y = transform.transform.rotation.y
        z = transform.transform.rotation.z
        Nq = w*w + x*x + y*y + z*z
        s = 2.0/Nq
        X = x*s
        Y = y*s
        Z = z*s
        wX = w*X; wY = w*Y; wZ = w*Z
        xX = x*X; xY = x*Y; xZ = x*Z
        yY = y*Y; yZ = y*Z; zZ = z*Z

        return np.array(
               [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
                [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
                [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])


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
