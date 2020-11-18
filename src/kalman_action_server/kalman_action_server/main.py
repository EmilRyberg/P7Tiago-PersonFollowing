import rclpy
from person_follower_interfaces.msg import PersonInfoList
from person_follower_interfaces.action import *
from kalman_action_server.ownKalman import KfTracker
from rclpy.action import ActionServer
from rclpy.node import Node
import numpy as np

# Used for sorting the list from the vision module
def sort_obj_based_on_id(obj):
    return obj.person_id

class KalmanTracking(Node):

    kf = list()
    kf_number = 0

    # Which ID to follow
    tracked_id = -1

    def __init__(self):

        super().__init__('kalman_action_server')
        self.subscription = self.create_subscription(PersonInfoList, '/persons', self.detection_cb, 1)

        self._action_server = ActionServer(self, Kalman, 'kalmanTracker', self.action_cb)

    def detection_cb(self, msg: PersonInfoList):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 # Conversion to seconds
        persons = msg.persons
        persons.sort(key=sort_obj_based_on_id) # Sorting based on ID
        self.tracked_id = msg.tracked_id

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
                mPos = np.array([[persons[newPersonIdx].point.x],
                                 [persons[newPersonIdx].point.y]])

                self.kf[i].update(mPos)
                
                self.kf[i].isTracked = True
                newPersonIdx += 1

            # When a new ID has come
            else:
                # Getting the measured position into the right format
                mPos = np.array([persons[newPersonIdx].point.x, 
                                 persons[newPersonIdx].point.y])

                self.kf.append(KfTracker(mPos, time))
                self.kf_number += 1
                
                newPersonIdx += 1

    def action_cb(self, cb_handle):
        id = cb_handle.request.id

        cb_handle.succeed() # Saying the goal was accomplished

        result = Kalman.Result() # Creating result message
        
        if self.kf_number != 0: # Making sure atleast one kalman filter is running before indexing
            result.point.x = self.kf[id].x[0, 0]
            result.point.y = self.kf[id].x[1, 0]
            result.is_tracked = self.kf[id].isTracked

        result.tracked_id = self.tracked_id # Replying which ID is to be tracked, essentially forwarding from earlier
        return result


def main():
    rclpy.init()

    KT = KalmanTracking()

    rclpy.spin(KT)


if __name__ == '__main__':
    main()
