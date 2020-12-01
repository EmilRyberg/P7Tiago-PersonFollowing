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
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header


# Used for sorting the list from the vision module
def sort_obj_based_on_id(obj):
    return obj.person_id


class KalmanTracking(Node):
    def __init__(self):
        super().__init__('kalman_action_server')
        self.kf = []
        self.kf_number = 0
        self.should_track_id = -1
        self.tracked_id = -1
        self.last_sent_head_movement = time.time()
        self._action_server = ActionServer(self, Kalman, 'find_human', self.action_cb)
        self.head_pub = self.create_publisher(BridgeAction, "/head_move_action", 1)
        self.subscription = self.create_subscription(PersonInfoList, '/persons', self.detection_cb, 1)

    def detection_cb(self, msg: PersonInfoList):
        ttime = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Conversion to seconds
        persons = msg.persons
        persons.sort(key=sort_obj_based_on_id)  # Sorting based on ID
        self.should_track_id = msg.tracked_id

        if len(persons) == 0:  # If no new IDs have come, these are simply updated
            for i in range(self.kf_number):
                self.kf[i].predict(ttime)
                self.kf[i].isTracked = False

            return

        newPersonIdx = 0
        for i in range(self.kf_number if self.kf_number > persons[-1].person_id + 1 else persons[-1].person_id + 1):
            # This runs when there is no update to the specific ID
            if newPersonIdx >= len(persons):
                newPersonIdx = len(persons) - 1

            if i != persons[newPersonIdx].person_id and i < self.kf_number:
                self.kf[i].predict(ttime)
                self.kf[i].isTracked = False

            # Runs when there is an update to a specific ID
            elif (i == persons[newPersonIdx].person_id and i < self.kf_number):
                self.kf[i].predict(ttime)

                # Getting the measured position into the right format
                mPos = np.array([[persons[newPersonIdx].pose.pose.position.x],
                                 [persons[newPersonIdx].pose.pose.position.y]])

                self.kf[i].update(mPos)

                self.kf[i].isTracked = True
                newPersonIdx += 1

            # When a new ID has come
            else:
                # Getting the measured position into the right format
                mPos = np.array([persons[newPersonIdx].pose.pose.position.x,
                                 persons[newPersonIdx].pose.pose.position.y])

                self.kf.append(KfTracker(mPos, ttime))
                self.kf_number += 1

                newPersonIdx += 1

        #### move camera
        current_time = time.time()
        if current_time - self.last_sent_head_movement > 2:
            self.last_sent_head_movement = current_time
            if self.tracked_id != -1:
                tracked_person = next((x for x in persons if x.person_id == self.tracked_id), None)
                self.get_logger().info(f"tracked person: {tracked_person}")
                if tracked_person is not None:
                    self.move_head(tracked_person.image_x, tracked_person.image_y)

    def action_cb(self, cb_handle):
        id = cb_handle.request.id
        self.get_logger().info(f"Got ID: {id}")

        result = Kalman.Result()  # Creating result message

        if id == -1:
            id = self.should_track_id
        self.tracked_id = id

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        map_pose = Pose()
        if self.kf_number != 0:  # Making sure atleast one kalman filter is running before indexing
            self.get_logger().info(f"Sending goal: {self.kf[id].x[0, 0]} - {self.kf[id].x[1, 0]} - "
                                   f"isTracked: {self.kf[id].isTracked}")
            map_pose.position.x = self.kf[id].x[0, 0]
            map_pose.position.y = self.kf[id].x[1, 0]
            map_pose.position.z = 0.0

            orientation = self.get_orientation(self.kf[id].x[2, 0], self.kf[id].x[3, 0])

            map_pose.orientation.x = orientation[0]
            map_pose.orientation.y = orientation[1]
            map_pose.orientation.z = orientation[2]
            map_pose.orientation.w = orientation[3]
            result.pose = PoseStamped(header=header, pose=map_pose)
            result.is_tracked = self.kf[id].isTracked

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
        msg = BridgeAction()

        # self.get_logger().info(f"move head horizontal: {horizontal}")
        msg.x = x
        msg.y = y

        msg.min_duration = min_duration
        msg.max_velocity = 25.

        self.head_pub.publish(msg)


def main():
    rclpy.init()

    KT = KalmanTracking()

    rclpy.spin(KT)


if __name__ == '__main__':
    main()
