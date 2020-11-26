import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose
from std_msgs.msg import Header
import os
from cv_bridge import CvBridge
import tf2_ros
from person_detector.feature_extractor.feature_extractor import FeatureExtractor, embedding_distance, is_same_person
from person_detector.person_finder.person_finder import PersonFinder
from person_follower_interfaces.msg import PersonInfoList, PersonInfo
import numpy as np
import cv2 as cv
import math
from typing import Optional
from enum import Enum
from rclpy.action import ActionClient
import time

HFOV = 1.01229096615671
W = 640
VFOV = 0.785398163397448
H = 480
UNCONFIRMED_COUNT_THRESHOLD = 3
UNCONFIRMED_TIME_THRESHOLD_SECONDS = 10  # secs


class ImageToFrameEnum(Enum):
    CAMERA_ANGLE = 0
    CAMERA_FRAME = 1
    ROBOT_FRAME = 2
    MAP_FRAME = 3


class CustomDuration: # Hack to make duration for tf2_ros work since it expects 2 fields, sec and nanosec
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


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

        self.publisher_ = self.create_publisher(PersonInfoList, "/persons", 1)
        self.get_logger().info("Loading weights")
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

        self.tf_buffer = tf2_ros.Buffer(cache_time=CustomDuration(sec=20))
        self.listener = tf2_ros.TransformListener(self.tf_buffer, spin_thread=True, node=self)
        self.first_run = True
        self.found_transform = False
        self.last_error = None

        self.get_logger().info("Subscribing to topics")
        self.image_subscriber = self.create_subscription(CompressedImage,
                                                         "/compressed_images",
                                                         self.image_callback,
                                                         qos_profile)
        #self.depth_subscriber = self.create_subscription(CompressedImage,
        #                                                 "/compressed_depth_images",
        #                                                 self.depth_callback,
        #                                                 qos_profile)
        self.depth_subscriber = self.create_subscription(Image,
                                                         "/depth",
                                                         self.depth_callback,
                                                         qos_profile)
        self.get_logger().info("Node started")

    def image_callback(self, msg: CompressedImage):
        self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_stamp = msg.header.stamp
        self.image_is_updated = True
        self.got_image_callback()

    def depth_callback(self, msg):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #str_msg = msg.data
        #buf = np.ndarray(shape=(1, len(str_msg)),
        #                 dtype=np.float32, buffer=msg.data)
        #self.depth_image = cv.imdecode(buf, cv.IMREAD_ANYDEPTH)
        #self.get_logger().info(f"got depth image {self.depth_image}")
        self.depth_stamp = msg.header.stamp
        self.depth_is_updated = True
        self.got_image_callback()

    def got_image_callback(self):
        if not self.image_is_updated or not self.depth_is_updated:
            return
        self.image_is_updated = False
        self.depth_is_updated = False

        self.get_logger().info("Running")
        person_detections = self.person_finder.find_persons(self.image)
        persons = []
        id_to_track = -1
        for person_detection in person_detections:
            if person_detection is None:
                self.get_logger().warn("person det is None")
            cropped_person_img = self.person_finder.crop_bounding_box(self.image, person_detection)
            features = self.feature_extractor.get_features(cropped_person_img)
            found_same_person = False
            features_below_threshold = []
            person_id = None
            for i, (pid, emb) in enumerate(self.person_features_mapping):
                distance = embedding_distance(features, emb)
                #self.get_logger().info(f"Distance to person {pid}: {distance:.5f}")
                # print(f"Distance: {distance}")
                is_below_threshold = is_same_person(features, emb, threshold=0.9)
                if is_below_threshold:
                    found_same_person = True
                    features_below_threshold.append((pid, distance, person_detection, i))
            if found_same_person:
                min_distance = 3  # distance is between 0 and 2
                best_match = None
                best_index = None
                best_person_id = None
                for pid, distance, person_detection, index in features_below_threshold:
                    if distance < min_distance:
                        min_distance = distance
                        best_match = (person_detection, pid)
                        best_person_id = pid
                        best_index = index
                current_person_id, current_features = self.person_features_mapping[best_index]
                new_emb = current_features * 0.8 + features * 0.2
                #diff_dist = embedding_distance(current_features, new_emb)
                #self.get_logger().info(f"Moving embedding, diff distance: {diff_dist}")

                self.person_features_mapping[best_index] = (best_match[1], new_emb)
                person_id = best_person_id
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
                    is_below_threshold = is_same_person(features, emb, threshold=0.9)
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
                    best_index, (emb, times_found, time_last_seen) = best_match
                    avg_emb = (features + emb) / 2
                    new_times_found = times_found + 1
                    if new_times_found >= UNCONFIRMED_COUNT_THRESHOLD:
                        indices_to_remove.append(best_index)
                        self.person_features_mapping.append((self.person_id, avg_emb))
                        person_id = self.person_id
                        self.person_id += 1
                    found_same_unconfirmed_person = True
                    self.unconfirmed_persons[best_index] = (avg_emb, new_times_found, time_now)
                if len(indices_to_remove) > 0:
                    self.unconfirmed_persons = [p for i, p in enumerate(self.unconfirmed_persons) if
                                                i not in indices_to_remove]
                    #self.get_logger().info(f"New list length: {len(self.unconfirmed_persons)}")
                if not found_same_unconfirmed_person:
                    self.unconfirmed_persons.append((features, 0, time_now))
            if person_id is not None:
                map_pose = self.transform_image_to_frame(bounding_box=person_detection, frame=ImageToFrameEnum.MAP_FRAME)
                robot_pose = self.transform_image_to_frame(bounding_box=person_detection, frame=ImageToFrameEnum.ROBOT_FRAME)
                horizontal_angle, vertical_angle = self.transform_image_to_frame(bounding_box=person_detection,
                                                                                 frame=ImageToFrameEnum.CAMERA_ANGLE)
                if map_pose is None:
                    self.get_logger().warn(f"Returned map_pose is None, skipping person with ID: {person_id}")
                else:
                    person = PersonInfo()
                    person.person_id = person_id
                    person_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
                    person.pose = PoseStamped(header=person_header, pose=map_pose)
                    persons.append(person)

                    self.get_logger().info(f"Robot pose is None?: {robot_pose}")

                    if robot_pose is not None:
                        if self.is_person_to_track(robot_pose.position.x, robot_pose.position.y):
                            self.get_logger().info(f"Found person to track")
                            id_to_track = person_id

                    person.horizontal_angle = horizontal_angle
                    person.vertical_angle = vertical_angle
                    

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # TODO change this to the correct frame id of depth sensor
        person_info = PersonInfoList(header=header, persons=persons, tracked_id=id_to_track)
        self.get_logger().info(f"Publishing {person_info}")
        self.publisher_.publish(person_info)
        self.get_logger().info("Published")

    def transform_image_to_frame(self, bounding_box, frame):
        stamp = self.depth_stamp
        self.old_depth_stamp = stamp
        #if self.found_transform or self.first_run:
        #    stamp = self.depth_stamp
        #    self.old_depth_stamp = self.depth_stamp
        #    self.first_run = False
        #else:
        #    stamp = self.old_depth_stamp


        # Here I made some coefficients for a linear function for calculating the angle for each pixel
        ah = -HFOV / (W - 1)
        bh = HFOV / 2
        av = -VFOV / (H - 1)
        bv = VFOV / 2

        # Here, if we have a bounding box, we find the position of the object and publish it
        # Getting the middle pixel of the bounding box
        # TODO replace with better method

        # We calculate the two angles for the pixel
        centerx=int((bounding_box[0]+bounding_box[2])/2)
        centery=int((bounding_box[1]+bounding_box[3])/2)
        horizontal_angle = ah * centerx + bh
        vertical_angle = av * centery + bv

        if frame == ImageToFrameEnum.CAMERA_ANGLE:
            return horizontal_angle, vertical_angle

        c_horizontal = math.cos(horizontal_angle)
        c_vertical = math.cos(vertical_angle)
        s_horizontal = math.sin(horizontal_angle)
        s_vertical = math.sin(vertical_angle)

        #self.get_logger().info(f"ha : {horizontal_angle}, va: {vertical_angle}, dist: {dist}, c_h: {c_horizontal}\n"
        #                       f"c_v: {c_vertical} s_h: {s_horizontal} s_v: {s_vertical}")
        dist = self.read_depth(self.depth_image, bounding_box)
        if dist is None:
            return None
        # We get the x, y, z in the camera frame
        v = np.array([c_horizontal * (dist * c_vertical),
                      s_horizontal * (dist * c_vertical),
                      s_vertical * (dist * c_horizontal)])

        if frame == ImageToFrameEnum.CAMERA_FRAME:
            point = Point()

            point.x = v[0]
            point.y = v[1]
            point.z = v[2]

            return point

        time_difference = self.depth_stamp.sec + self.depth_stamp.nanosec*1e-9 - (self.old_depth_stamp.sec+self.old_depth_stamp.nanosec*1e-9)
        if time_difference > 5:
            self.get_logger().error(f"Images and tf frames are delayed by more than 5s. Last error: {self.last_error}")
        elif time_difference > 0.5:
            self.get_logger().warn("Images and tf frames are delayed by more than 0.5s")

        transform = None
        if frame == ImageToFrameEnum.ROBOT_FRAME:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', 'xtion_depth_frame', stamp)
            except tf2_ros.ExtrapolationException as e:
                self.last_error = e
                self.found_transform = False
                return None
        elif frame == ImageToFrameEnum.MAP_FRAME:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'xtion_depth_frame', stamp)
            except tf2_ros.ExtrapolationException as e:
                self.last_error = e
                self.found_transform = False
                return None
                
        self.found_transform = True

        self.get_logger().info(f"Depth shape: {self.depth_image.shape}, point checked: {self.depth_image[centery, centerx]}")

        # Rotation and translation
        R = self.quaternion_to_rotation_matrix(transform)
        T = np.array([transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z])

        # We transform it to robot/map frame
        map_point = np.dot(R, v) + T
        self.get_logger().info(f"Map point: {map_point}")
        pose = Pose()
        pose.position.x = map_point[0]
        pose.position.y = map_point[1]
        pose.position.z = map_point[2]
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w

        #self.old_depth_stamp = self.depth_stamp

        return pose

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

    def is_person_to_track(self, x, y): # x, y in robot_frame
        angle = np.arctan2(y, x)
        self.get_logger().info(f"Angle: {angle}, x: {x}")
        return x < 1.25 and np.abs(angle) < 0.3491
    def read_depth(self, dImg, bbox):
        centerx=int((bbox[0]+bbox[2])/2)
        centery=int((bbox[1]+bbox[3])/2)
        x1 = centerx - 10
        y1 = centery - 10
        x2 = centerx + 10
        y2 = centery + 10
        count = 0
        dist = 0
        while x1 <= x2:
            while y1 <= y2:
                if not np.isnan(dImg.item(y1, x1)):
                    dist = dist + dImg.item(y1, x1)
                    count+=1
                y1 = y1 + 1
            x1 = x1 + 1
        if count == 0:
            return None
        average_depth = dist / count
        return average_depth


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
