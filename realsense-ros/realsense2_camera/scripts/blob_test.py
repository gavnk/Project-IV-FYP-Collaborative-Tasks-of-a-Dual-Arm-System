#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import cv2
import numpy as np

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    # Perform blob detection on the color image
    # TODO: Add your blob detection code here
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    blurred = cv2.GaussianBlur(cv_image, (11,11), 0)   # Blur image to smooth everything out
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # Convert to HSV to be able to see only the ball
    lower = np.array([35, 40, 40])                # Lower HSV values of the ball
    upper = np.array([85, 255, 255])                # Upper HSV values of the ball
    mask = cv2.inRange(hsv, lower, upper)           # Show the pixels that are in the HSV value range in white only 
    mask = cv2.erode(mask, None, iterations = 2)    # Remove white noise
    mask = cv2.dilate(mask, None, iterations = 2)   # Enlarge the image after eroding it (Common place to erode then dilate)
    cnt = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]  #Find contours, added [0] at the end to only get the relevant values
    # Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    center = ""
    if len(cnt) > 0:                                # If found a contour
        c = max(cnt, key=cv2.contourArea)           # Getting the largest contour. Largest defined by largest area
        ((x, y), radius) = cv2.minEnclosingCircle(c)# Getting the position and radius of smallest enclosing circle of the found contour
        M = cv2.moments(c)                          # Finding image moment, detailed in documentation exactly what it is and how it's used.
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))   # Formula shown in documentation
        if radius > 15:                             #Make sure not locking in on noise
            cv2.circle(cv_image, center, 6, (0, 0, 255), -1)   # Draw small circle around center of ball
    cv2.imshow("Blob Detection", cv_image)                      # Show the image with detected blob
    cv2.imshow("mask1", cv_image)                      # Show the OG frame with a small circle indicating the ball center
   # cv2.imshow("mask2", mask)                       # Show only the ball in white
    cv2.waitKey(1)

    # Display the image with detected blob
   # cv2.imshow("Blob Detection", cv_image)
   # cv2.waitKey(1)

def main():
    rospy.init_node("blob_detection_node")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()