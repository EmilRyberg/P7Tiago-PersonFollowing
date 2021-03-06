import rclpy
from person_follower_interfaces.msg import PersonInfoList, BridgeAction
from person_follower_interfaces.action import *
from kalman_action_server.ownKalman import KfTracker
from rclpy.action import ActionServer
from rclpy.node import Node
import numpy as np
import math
from scipy.spatial.transform import Rotation
import time
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Vector3
from std_msgs.msg import Header
from typing import List, Tuple, Dict
import csv

VELOCITY_NORM_THRESHOLD = 2  # m/s. 2 m/s -> 7.2 km/h

test3_csv = open('test2.csv', mode='w')
fieldnames = ['time', 'angle', 'tracked']
writer = csv.DictWriter(test3_csv, fieldnames=fieldnames)
writer.writeheader()

def log_for_test3(time, angle, tracked):
    global writer
    writer.writerow({'time': time, 'angle': angle, 'tracked': tracked})

class KalmanTracking(Node):
    def __init__(self):
        super().__init__('kalman_action_server')
        self.filters: Dict[int, KfTracker] = {}
        self.should_track_id = -1
        self.tracked_id = -1
        self.last_sent_head_movement = time.time()
        self.last_time = time.time()
        self._action_server = ActionServer(self, Kalman, 'find_human', self.action_cb)
        self.head_pub = self.create_publisher(BridgeAction, "/head_move_action", 1)
        self.visualization_publisher = self.create_publisher(PoseArray, "/detections_filtered", 1)
        self.head_move_publisher = self.create_publisher(Vector3, "/head_move", 1)
        self.subscription = self.create_subscription(PersonInfoList, '/persons', self.detection_cb, 1)
        self.person_last_positions: Dict[int, Tuple[np.ndarray, float]] = {}
        self.head_angle = 0

    def detection_cb(self, msg: PersonInfoList):
        ttime = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Conversion to seconds
        persons = msg.persons
        self.should_track_id = msg.tracked_id

        for person_id, kf_filter in self.filters.items():
            measurement = [person for person in persons if person.person_id == person_id]
            if len(measurement) == 0:
                # person not in measurement, predict
                kf_filter.predict(ttime)
                kf_filter.is_tracked = False
            else:
                measurement_item = measurement[0]
                kf_filter.predict(ttime)

                # Getting the measured position into the right format
                map_position = np.array([[measurement_item.pose.pose.position.x],
                                         [measurement_item.pose.pose.position.y]])

                last_position, last_time = self.person_last_positions[person_id]
                delta_time = ttime - last_time
                x_diff = map_position[0, 0] - last_position[0, 0]
                y_diff = map_position[1, 0] - last_position[1, 0]
                x_vel = x_diff / delta_time
                y_vel = y_diff / delta_time
                vel_norm = math.sqrt(x_vel ** 2 + y_vel ** 2)

                if vel_norm >= VELOCITY_NORM_THRESHOLD:
                    self.get_logger().warn(
                        f"Discarding measurement of person: {person_id}, since the velocity is {vel_norm} m/s")
                    kf_filter.is_tracked = False
                else:
                    self.person_last_positions[person_id] = (map_position, ttime)
                    kf_filter.update(map_position)
                    kf_filter.is_tracked = True

        new_persons = [person for person in persons if person.person_id not in self.filters.keys()]
        for new_person in new_persons:
            # Getting the measured position into the right format
            map_position = np.array([[new_person.pose.pose.position.x],
                                     [new_person.pose.pose.position.y]])
            self.person_last_positions[new_person.person_id] = (map_position, ttime)
            self.filters[new_person.person_id] = KfTracker(map_position, ttime)

        #### move camera
        current_time = time.time()
        if current_time - self.last_sent_head_movement > 0.3:
            self.last_sent_head_movement = current_time
            if self.tracked_id != -1:
                tracked_person = next((x for x in persons if x.person_id == self.tracked_id), None)
                self.get_logger().info(f"tracked person: {tracked_person}")
                if tracked_person is not None:
                    self.move_head(tracked_person.image_x, tracked_person.image_y)

        tracked_person_temp = next((x for x in persons if x.person_id == self.tracked_id), None)
        if self.tracked_id != -1 and tracked_person_temp is not None:
            log_for_test3(ttime, tracked_person_temp.horizontal_angle, 1)
        else:
            log_for_test3(ttime, -9000, 0)

        poses = []
        for kf_filter in self.filters.values():
            pose = Pose()
            pose.position.x = kf_filter.x[0, 0]
            pose.position.y = kf_filter.x[1, 0]
            pose.position.z = 0.0
            # self.get_logger().info(f"velocities x={filter.x[2, 0]} y={filter.x[3, 0]}")
            orientation = self.get_orientation(kf_filter.x[2, 0], kf_filter.x[3, 0])
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
            poses.append(pose)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        pose_array = PoseArray(header=header, poses=poses)
        self.visualization_publisher.publish(pose_array)

    def action_cb(self, cb_handle):
        id = cb_handle.request.id
        self.get_logger().info(f"Got ID: {id}")

        result = Kalman.Result()  # Creating result message

        if id == -1:
            if self.should_track_id == -1:
                return result
            id = self.should_track_id
        self.tracked_id = id

        if id not in self.filters.keys() and id != -1:
            self.get_logger().warn(f"Got id {id}, which does not exist")
            return result

        if cb_handle.request.remove_filter:
            del self.filters[id]
            cb_handle.succeed()
            return result

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        map_pose = Pose()
        self.get_logger().info(f"Sending goal: {self.filters[id].x[0, 0]} - {self.filters[id].x[1, 0]} - "
                               f"is_tracked: {self.filters[id].is_tracked}")
        map_pose.position.x = self.filters[id].x[0, 0]
        map_pose.position.y = self.filters[id].x[1, 0]
        map_pose.position.z = 0.0

        orientation = self.get_orientation(self.filters[id].x[2, 0], self.filters[id].x[3, 0])

        map_pose.orientation.x = orientation[0]
        map_pose.orientation.y = orientation[1]
        map_pose.orientation.z = orientation[2]
        map_pose.orientation.w = orientation[3]
        result.pose = PoseStamped(header=header, pose=map_pose)
        result.is_tracked = self.filters[id].is_tracked

        # map_pose = Pose()
        # map_pose.position.x = -0.84
        # map_pose.position.y = -6.5
        # map_pose.position.z = 0.0
        # map_pose.orientation.x = 0.0
        # map_pose.orientation.y = 0.0
        # map_pose.orientation.z = -0.655247
        # map_pose.orientation.w = 0.75541
        # result.pose = PoseStamped(header=header, pose=map_pose)
        # result.is_tracked = False

        self.get_logger().info("Returning point")
        cb_handle.succeed()  # Saying the goal was accomplished

        result.tracked_id = id  # Replying which ID is to be tracked, essentially forwarding from earlier
        return result

    def get_orientation(self, x_velocity, y_velocity):
        # Using SohCahToa -- We know opposite and adjacent, hence we use atan
        norm = math.sqrt(x_velocity ** 2 + y_velocity ** 2)
        x_velocity = x_velocity / norm
        y_velocity = y_velocity / norm
        angle_around_z = math.atan2(y_velocity, x_velocity)
        rotation = Rotation.from_rotvec(angle_around_z * np.array([0, 0, 1]))
        return rotation.as_quat()

    def move_head(self, x, y, min_duration=0.5):
        center_x = x
        self.get_logger().warn(f"center_x {center_x}")
        center_x -= (640 / 2)  # offset center to be zero
        center_x *= -1  # flip direction
        self.head_angle += center_x * (1 / 320) * 0.45
        if self.head_angle > 1.23:
            self.head_angle = 1.23
        elif self.head_angle < -1.23:
            self.head_angle = -1.23
        self.get_logger().warn(f"head_angle {self.head_angle}")
        msg = Vector3(x=self.head_angle, y=0.0)
        self.head_move_publisher.publish(msg)


def main():
    rclpy.init()

    KT = KalmanTracking()

    rclpy.spin(KT)


if __name__ == '__main__':
    main()
