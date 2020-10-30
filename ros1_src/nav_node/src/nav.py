#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
import math
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import tf2_ros

np.set_printoptions(suppress=True)

timeLastSent = rospy.Time(0)
meanPos = [0, 0, 0]
filtSize = 3
filtInit = 0
CD = rospy.Duration(1)

def pose2mat(trans):
    w = trans.transform.rotation.w
    x = trans.transform.rotation.x
    y = trans.transform.rotation.y
    z = trans.transform.rotation.z
    Nq = w*w + x*x + y*y + z*z
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z


    return np.array(
           [[1.0-(yY+zZ), xY-wZ, xZ+wY, trans.transform.translation.x],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX, trans.transform.translation.y],
            [xZ-wY, yZ+wX, 1.0-(xX+yY), trans.transform.translation.z],
            [0, 0, 0, 1]])


def navCB(p):
    T = pose2mat(tfBuffer.lookup_transform('map', 'base_link', p.header.stamp))

    mapP = np.dot(T, np.array([[p.point.x],
                        [p.point.y],
                        [p.point.z],
                        [1.0]]))
    
    if np.isnan(p.point.x):
        print("Is NaN")
        return

    global meanPos
    global filtInit
    if not filtInit:
        filtInit = 1
        for i in range(3):
            meanPos[i] = mapP[i]
    else:  
        for i in range(3):
            meanPos[i] = meanPos[i] * (filtSize - 1) / filtSize + mapP[i] / filtSize

    global timeLastSent
    global CD
    print(rospy.Time.now() - timeLastSent > CD)
    if rospy.Time.now() - timeLastSent > CD:
        print(1)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = meanPos[0]
        goal.target_pose.pose.position.y = meanPos[1]
        goal.target_pose.pose.position.z = meanPos[2]
        goal.target_pose.pose.position.x -= 1
        goal.target_pose.pose.orientation.w = 1

        client.send_goal(goal)
        timeLastSent = rospy.Time.now()

rospy.init_node('nav_node', anonymous=True)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rospy.sleep(1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

rospy.Subscriber("target_pos", PointStamped, navCB)

rospy.spin()