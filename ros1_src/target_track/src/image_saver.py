import cv2
import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import time

last_write_time = 0
img_id = 0

def cb(data):
    global last_write_time
    global img_id
    # Read a new frame
    current_time = time.time()
    if current_time - last_write_time <= 0.15:
        return
    last_write_time = current_time
    np_arr = np.fromstring(data.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    print(f"took image {img_id}")
    cv2.imwrite(f"{current_time}-{img_id}.png", frame)
    img_id += 1
    
    
rospy.init_node('image_saver', anonymous=True)
rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, cb)
rospy.spin()