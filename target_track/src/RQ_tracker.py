#! /usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import *
# initialize the camera
qrDet = cv2.QRCodeDetector()
qrDet.setEpsX(0.05)
qrDet.setEpsY(0.05)
bridge = CvBridge()
dImg = 0
global dep_rec
dep_rec = 0

HFOV = 1.01229096615671
w = 640
VFOV = 0.785398163397448
h = 480

ah = -HFOV/(w-1)
bh = HFOV / 2

av = -VFOV/(h-1)
bv = VFOV / 2

def cb(data):
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    decodedText, points, _ = qrDet.detectAndDecode(img)

    global dep_rec
    if points is not None and dep_rec:
        nrOfPoints = len(points)
        xp = 0
        yp = 0

        for i in range(nrOfPoints):
            nextPointIndex = (i+1) % nrOfPoints
            cv2.line(img, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)
            xp += points[i][0][0]
            yp += points[i][0][1]
            
        xp = int(x / 4) # cols
        yp = int(y / 4) # rows

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


    
    cv2.imshow("Image", img)
    cv2.waitKey(1)

def cb_d(data):
    global dImg
    dImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


rospy.init_node('rq_track', anonymous=True)

rospy.Subscriber("/xtion/rgb/image_color", Image, cb)
rospy.Subscriber("/xtion/depth_registered/image_raw", Image, cb_d)

rospy.spin()