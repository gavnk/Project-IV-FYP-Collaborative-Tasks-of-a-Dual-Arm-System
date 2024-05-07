#!/usr/bin/env python
import os
import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import json

# Initialize the CvBridge class
bridge = CvBridge()

def image_callback(msg):
    # Convert the ROS image message to a NumPy array
    img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Put your existing code here...

    # For example:
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, img = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
    

    # Convert the image back to a ROS message
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")

    # Publish the image
    image_pub.publish(img_msg)

rospy.init_node('image_processing_node')
image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)



if __name__ == '__main__':
image_callback(img)
     
    