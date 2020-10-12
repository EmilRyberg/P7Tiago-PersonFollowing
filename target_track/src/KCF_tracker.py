#!/usr/bin/env python
# coding: utf-8

import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import *

bridge = CvBridge()
global init
init = 0
global dep_rec
dep_rec = 0
dImg = 0

HFOV = 1.01229096615671
w = 640
VFOV = 0.785398163397448
h = 480

ah = -HFOV/(w-1)
bh = HFOV / 2

av = -VFOV/(h-1)
bv = VFOV / 2


def cb(data):
    # Read a new frame
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    global init
    if not init:
        init = 1
        # Define an initial bounding box
        bbox = (287, 23, 86, 320)
    
        # Uncomment the line below to select a different bounding box
        bbox = cv2.selectROI(frame, False)
        cv2.destroyAllWindows()

        ok = tracker.init(frame, bbox)

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Draw bounding box
    global dep_rec
    if ok and dep_rec:
        # Tracking success
        font = cv2.FONT_HERSHEY_SIMPLEX
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        p3 = (int((bbox[0]+bbox[0] + bbox[2])/2),int((bbox[1]+bbox[1] + bbox[3])/2))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        cv2.putText(frame, str(p3) , p2, font, 2, (255,0,0), 2)

        xp = int(bbox[0]) + int(bbox[2] / 2)
        yp = int(bbox[1]) + int(bbox[3] / 2)

        dist = dImg.item(yp, xp)

        Hangle = ah * xp + bh
        Vangle = av * yp + bv

        ch = cos(Hangle)
        cv = cos(Vangle)
        sh = sin(Hangle)
        sv = sin(Vangle)

        x = ch * (dist * cv)
        y = sh * (dist * cv)
        z = sv * (dist * ch)

        print(x, y, z)

        
    else:
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)


    # Display result
    cv2.imshow("Tracking", frame)

    cv2.waitKey(1)

def cb_d(data):
    global dImg
    dImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global dep_rec 
    dep_rec = 1

 
if __name__ == '__main__':
 
    # Set up tracker.
    # Instead of MIL, you can also use
 
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    tracker_type = tracker_types[4]
 
 
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()
    if tracker_type == "MOSSE":
        tracker = cv2.TrackerMOSSE_create()     
 
    rospy.init_node('kcf_track', anonymous=True)

    rospy.Subscriber("/xtion/rgb/image_color", Image, cb)
    rospy.Subscriber("/xtion/depth_registered/image_raw", Image, cb_d)

    rospy.spin()
