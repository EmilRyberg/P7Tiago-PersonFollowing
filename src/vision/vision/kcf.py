#!/usr/bin/env python
# coding: utf-8

import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
#from cv_bridge import CvBridge
from cv_bridge_custom import CvBridgeCustom
from math import *
import tf2_ros
import numpy as np
import time

np.set_printoptions(suppress=True)

class KcfTracker(Node):
    def __init__(self):
        super().__init__("kcf_tracker")
        qos_profile=QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.bridge = CvBridgeCustom()

        # We're setting it up so that we can get transforms between the frames in the robot
        # My understanding is that all the transformations are saved and we then query for the ones we want
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, spin_thread=True, node=self)
        time.sleep(1)

        self.depth_image = None
        self.image = None
        self.transform = None
        self.first_run = True
        self.found_transform = False
        self.new_stamp = None
        self.old_stamp = None
        self.last_error = None
        self.bbox = [287, 23, 86, 320]


        # This gets the transformation between the base_link and camera
        trans = 0



        self.image_sub = self.create_subscription(CompressedImage, "compressed_images", self.image_callback,
                                                  qos_profile=qos_profile)
        self.depth_sub = self.create_subscription(Image, "/depth", self.depth_callback, qos_profile=qos_profile)
        self.pub = self.create_publisher(PointStamped, '/target_pos', 1)

            # CB for RGB image
    def image_callback(self, data):
        #print("received image")
        # Read a new frame
        frame = self.bridge.compressed_imgmsg_to_cv2(data)
        self.new_stamp = data.header.stamp
        #cv2.imshow("a",frame)
        #cv2.waitKey()

        stamp = None
        if self.found_transform or self.first_run:
            stamp = self.new_stamp
            self.old_stamp = self.new_stamp
            #self.first_run = False
        else:
            stamp = self.old_stamp

        time_difference = ((self.new_stamp.sec*1e9+self.new_stamp.nanosec)-(self.old_stamp.sec*1e9+self.old_stamp.nanosec))*1e-9
        if time_difference > 1:
            print(f"[ERROR] Images and tf frames are delayed by more than 1s. Last error: {self.last_error}")
        elif time_difference > 0.5:
            print("[WARNING] Images and tf frames are delayed by more than 0.5s")

        try:
            self.transform = self.tfBuffer.lookup_transform('map', 'xtion_depth_frame', stamp)
        except tf2_ros.ExtrapolationException as e:
            self.last_error = e
            #print(e)
            self.found_transform = False
            return
        else:
            self.found_transform = True

        # Rotation and translation
        R, T = self.pose2mat(self.transform)

        # Horizontal and vertical FOV + resolution
        HFOV = 1.01229096615671
        w = 640
        VFOV = 0.785398163397448
        h = 480
        # Here I made some coefficients for a linear function for calculating the angle for each pixel
        ah = -HFOV / (w - 1)
        bh = HFOV / 2
        av = -VFOV / (h - 1)
        bv = VFOV / 2

        # Here, if we have a bounding box, we find the position of the object and publish it
        p = PointStamped()
        if self.depth_image is not None:
            # Shows some visual information about the tracking
            font = cv2.FONT_HERSHEY_SIMPLEX
            p1 = (int(self.bbox[0]), int(self.bbox[1]))
            p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
            p3 = (int((self.bbox[0] + self.bbox[0] + self.bbox[2]) / 2), int((self.bbox[1] + self.bbox[1] + self.bbox[3]) / 2))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
            cv2.putText(frame, str(p3), p2, font, 2, (255, 0, 0), 2)

            # Getting the middle pixel of the bounding box
            xp = int(self.bbox[0]) + int(self.bbox[2] / 2)
            yp = int(self.bbox[1]) + int(self.bbox[3] / 2)

            # The middle pixel can be outside the frame, so we stop that
            xp = w - 1 if xp > w else xp
            xp = 0 if xp < 0 else xp
            yp = h - 1 if yp > h else yp
            yp = 0 if yp < 0 else yp

            # Getting the distance to the target
            dist = self.depth_image.item(yp, xp)

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

            # We transform it to map frame
            newP = np.dot(R, v) + T
            p.point.x = newP[0]
            p.point.y = newP[1]
            p.point.z = newP[2]
            self.pub.publish(p)

    def depth_callback(self, data):
        #print("received depth image")
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def pose2mat(self, trans):
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

def main():
    rclpy.init()
    kcf_tracker = KcfTracker()
    time.sleep(1)

    print("kcf tracker running")
    rclpy.spin(kcf_tracker)


if __name__ == '__main__':
    main()