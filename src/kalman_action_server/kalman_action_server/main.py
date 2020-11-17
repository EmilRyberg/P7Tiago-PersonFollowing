import rclpy
from person_follower_interfaces.msg import *
from person_follower_interfaces.action import *
from kalman_action_server.ownKalman import KfTracker
from rclpy.action import ActionServer
from rclpy.node import Node
import numpy as np

def sort_obj_based_on_id(obj):
    return obj.person_id

class KalmanTracking(Node):

    kf = list()
    kf_number = 0

    def __init__(self):

        super().__init__('kalman_action_server')
        self.subscription = self.create_subscription(PersonInfoList, 'topic', self.detection_cb, 1)

        self._action_server = ActionServer(self, Kalman, 'kalmanTracker', self.action_cb)

    def detection_cb(self, msg):
        time = msg.header.timestamp.sec + msg.header.timestamp.nanosec / 1e9 # Conversion to seconds
        persons = msg.persons.persons
        persons.sort(key=sort_obj_based_on_id)

        if len(persons) == 0:
            for i in range(self.kf_number):
                self.kf[i].predict(time)
                self.kf[i].isTracked = False

            return

        newPersonIdx = 0
        for i in range(self.kf_number if self.kf_number > persons[-1].person_id+1 else persons[-1].person_id+1):
            # This runs when there is no update to the specific ID
            if i != persons[newPersonIdx].person_id and i < self.kf_number:
                self.kf[i].predict(time)

                self.kf[i].isTracked = False
                
            # Runs when there is an update to a specific ID
            elif(i == persons[newPersonIdx].person_id and i < self.kf_number):
                self.kf[i].predict(time)

                # Getting the measured position into the right format
                mPos = np.array([[persons[newPersonIdx].point.x], 
                                 [persons[newPersonIdx].point.y]])

                self.kf[i].update(mPos)
                
                self.kf[i].isTracked = True
                newPersonIdx += 1

            # When a new ID has come
            else:
                # Getting the measured position into the right format
                mPos = np.array([[persons[newPersonIdx].point.x], 
                                 [persons[newPersonIdx].point.y]])

                self.kf.append(KfTracker(mPos, time))
                self.kf_number += 1
                
                newPersonIdx += 1

    def action_cb(self, cb_handle):
        print("Executing goal...")

        id = cb_handle.request.id

        feedback_msg = Kalman.Feedback()

        while True:
            feedback_msg.point.x = self.kf[id].x[0, 0]
            feedback_msg.point.y = self.kf[id].x[1, 0]
            feedback_msg.is_found = self.kf[id].isTracked

            cb_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        cb_handle.succeed()

        result = Kalman.Result()
        result.is_tracked = False#self.kf[]
        return result



import time

def main():
    rclpy.init()

    KT = KalmanTracking()

    rclpy.spin(KT)


if __name__ == '__main__':
    main()
