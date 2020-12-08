import rclpy
from person_follower_interfaces.msg import PersonInfoList, BridgeAction
from person_follower_interfaces.action import *
from kalman_action_server.ownKalman import KfTracker
from rclpy.action import ActionServer
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header

# Used for sorting the list from the vision module
def sort_obj_based_on_id(obj):
    return obj.person_id

test3_csv = open('test3.csv', mode='w')
fieldnames = ['time', 'angle', 'tracked']
writer = csv.DictWriter(test3_csv, fieldnames=fieldnames)
writer.writeheader()

def log_for_test3(time, angle, tracked):
    global writer
    writer.writerow({'time': time, 'angle': angle, 'tracked': tracked})

class KalmanTracking(Node):

    def __init__(self):
        super().__init__('kalman_action_server')
        self.subscription = self.create_subscription(PersonInfoList, '/persons', self.detection_cb, 1)
        self.kf = []
        self.kf_number = 0
        self.should_track_id = -1
        self.tracked_id = -1
        self._action_server = ActionServer(self, Kalman, 'find_human', self.action_cb)
        self.head_pub = self.create_publisher(BridgeAction, "/head_move_action", 1)

    def detection_cb(self, msg: PersonInfoList):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 # Conversion to seconds
        persons = msg.persons
        persons.sort(key=sort_obj_based_on_id) # Sorting based on ID
        self.should_track_id = msg.tracked_id

        if len(persons) == 0: # If no new IDs have come, these are simply updated
            for i in range(self.kf_number):
                self.kf[i].predict(time)
                self.kf[i].isTracked = False

            return

        newPersonIdx = 0
        for i in range(self.kf_number if self.kf_number > persons[-1].person_id+1 else persons[-1].person_id+1):
            # This runs when there is no update to the specific ID
            if newPersonIdx >= len(persons):
                newPersonIdx = len(persons) - 1

            if i != persons[newPersonIdx].person_id and i < self.kf_number:
                self.kf[i].predict(time)
                self.kf[i].isTracked = False
                
            # Runs when there is an update to a specific ID
            elif(i == persons[newPersonIdx].person_id and i < self.kf_number):
                self.kf[i].predict(time)

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

                self.kf.append(KfTracker(mPos, time))
                self.kf_number += 1
                
                newPersonIdx += 1

        #### move camera
        if self.tracked_id != -1:
            tracked_person = next((x for x in persons if x.person_id == self.tracked_id), None)
            if tracked_person is not None:
                self.move_head(tracked_person.horizontal_angle)
                log_for_test3(time, tracked_person.horizontal_angle, 1)
            else:
                log_for_test3(time, np.nan, 0)


    def action_cb(self, cb_handle):
        id = cb_handle.request.id
        self.get_logger().info(f"Got ID: {id}")

        result = Kalman.Result() # Creating result message

        if id == -1:
            id = self.should_track_id
        self.tracked_id = id

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        map_pose = Pose()
        if self.kf_number != 0: # Making sure atleast one kalman filter is running before indexing
            self.get_logger().info(f"Sending goal: {self.kf[id].x[0, 0]} - {self.kf[id].x[1, 0]}")
            map_pose.position.x = self.kf[id].x[0, 0]
            map_pose.position.y = self.kf[id].x[1, 0]
            map_pose.position.z = 0.0
            map_pose.orientation.x = 0.0
            map_pose.orientation.y = 0.0
            map_pose.orientation.z = -0.655247
            map_pose.orientation.w = 0.75541
            result.pose = PoseStamped(header=header, pose=map_pose)
            result.is_tracked = self.kf[id].isTracked


        #map_pose = Pose()
        #map_pose.position.x = -0.84
        #map_pose.position.y = -6.5
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

    def move_head(self, horizontal, vertical = 0):
        msg = BridgeAction()

        msg.point.x = -horizontal # Makes it go left on positive
        msg.point.y = 0 #-vertical   # Makes it go up on positive
        msg.point.z = 1.

        msg.min_duration = 0.5
        msg.max_velocity = 25.

        self.head_pub.publish(msg)

def main():
    rclpy.init()

    KT = KalmanTracking()

    rclpy.spin(KT)


if __name__ == '__main__':
    main()
