#!/usr/bin/env python
# coding: utf-8

import cv2
import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from math import *
import tf2_ros
import numpy as np

np.set_printoptions(suppress=True)

bridge = CvBridge()
# This is for some KCF setup
init = 0
# This makes sure the depth image is received before attempting to use it
dep_rec = 0
dImg = 0
p = PointStamped()
# This gets the transformation between the base_link and camera
trans = 0

# Horizontal and vertical FOV + resolution
HFOV = 1.01229096615671
w = 640
VFOV = 0.785398163397448
h = 480

# Here I made some coefficients for a linear function for calculating the angle for each pixel
ah = -HFOV/(w-1)
bh = HFOV / 2

av = -VFOV/(h-1)
bv = VFOV / 2

# Gets the rotation matrix and translation vector between frames
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
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]]), np.array([trans.transform.translation.x, 
            trans.transform.translation.y, 
            trans.transform.translation.z])


# CB for RGB image
def cb(data):
    # Read a new frame
    np_arr = np.fromstring(data.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    p.header.stamp = data.header.stamp

    # Getting the transformation between base and camera
    trans = tfBuffer.lookup_transform('base_link', 'xtion_depth_frame', p.header.stamp)

    # Rotation and translation
    R, T = pose2mat(trans)

    # KCF setup stuff
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

    # Here, if we have a bounding box, we find the position of the object and publish it
    global dep_rec
    global pub
    if ok and dep_rec:
        # Shows some visual information about the tracking
        font = cv2.FONT_HERSHEY_SIMPLEX
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        p3 = (int((bbox[0]+bbox[0] + bbox[2])/2),int((bbox[1]+bbox[1] + bbox[3])/2))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        cv2.putText(frame, str(p3) , p2, font, 2, (255,0,0), 2)

        # Getting the middle pixel of the bounding box
        xp = int(bbox[0]) + int(bbox[2] / 2)
        yp = int(bbox[1]) + int(bbox[3] / 2)
        #a=int(bbox[0])
        #b=int(bbox[1])
        #c=int(bbox[0]+bbox[2])
        #d=int(bbox[1]+bbox[3])
        #xleft=(a+xp)/2
        #yleft=yp
        #xright=(c+yp)/2
        #yright=yp
        #xup=xp
        #yup=(b+yp)/2
        #xdown=xp
        #ydown=(d+yp)/2


        # The middle pixel can be outsude the frame, so we stop that
        xp = w-1 if xp > w else xp
        xp = 0 if xp < 0 else xp
        yp = h-1 if yp > h else yp
        yp = 0 if yp < 0 else yp

        # Getting the distance to the target
        dist = dImg.item(yp, xp)
        #distmiddle=dImg.item(yp, xp)
        #distleft=dImg.item(yleft, xleft)
        #distright=dImg.item(yright, xright)
        #distup=dImg.item(yup, xdown)
        #distdown=dImg.item(ydown, xdown)
        #distmean=(distleft+distmiddle+distup+distdown+distright)/5

        # We calculate the two angles for the pixel
        Hangle = ah * xp + bh
        Vangle = av * yp + bv

        cHor = cos(Hangle)
        cVer = cos(Vangle)
        sHor = sin(Hangle)
        sVer = sin(Vangle)

        # We get the x, y, z in the camera frame
        v = np.array([cHor * (dist * cVer), 
        sHor * (dist * cVer),
        sVer * (dist * cHor)])

        # We transform it to the robot frame
        newP = np.dot(R, v) + T
        p.point.x = newP[0]
        p.point.y = newP[1]
        p.point.z = newP[2]

        pub.publish(p)
        
    else:
        # If nothing was found, we send nan values
        p.point.x = float('nan')
        p.point.y = float('nan')
        p.point.z = float('nan')

        pub.publish(p)

        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display result
    cv2.imshow("Tracking", frame)

    cv2.waitKey(1)

# Depth image cb
def cb_d(data):
    global dImg
    dImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global dep_rec 
    dep_rec = 1


rospy.init_node('kcf_track', anonymous=True)

# KCF stuff, guessing it's just a part of openCV
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

# We're setting it up so that we can get transforms between the frames in the robot
# My understanding is that all the transformations are saved and we then query for the ones we want
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rospy.sleep(1) # Gotta sleep a bit so that we get some transformation, otherwise stuff crashes

rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, cb)
rospy.Subscriber("/xtion/depth_registered/image_raw", Image, cb_d)

pub = rospy.Publisher('target_pos', PointStamped, queue_size=1)

rospy.spin()
