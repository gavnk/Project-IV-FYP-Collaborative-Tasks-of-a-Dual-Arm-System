#!/usr/bin/env python
import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv2.imshow("Depth Camera Feed", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node("realsense_camera_node", anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    main()