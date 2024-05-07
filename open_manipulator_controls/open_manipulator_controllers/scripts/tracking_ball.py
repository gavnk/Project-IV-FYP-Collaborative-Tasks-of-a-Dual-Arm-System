#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np





# Author: Michael Aboulhair 

def callback(data):
  # Convert ROS Image message to OpenCV image
  bridge = CvBridge()
  color = bridge.imgmsg_to_cv2(data, "bgr8")
  res = color.copy()
  hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

  l_b = np.array([24, 133, 48])
  u_b = np.array([39, 200, 181])

  mask = cv2.inRange(hsv, l_b, u_b)
  color = cv2.bitwise_and(color, color, mask=mask)

  ### motion detector
  d = cv2.absdiff(color_init, color)
  gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
  blur = cv2.GaussianBlur(gray, (5, 5), 0)
  _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
  dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
  (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  #cv2.drawContours(color, c, -1, (0, 255, 0), 2)
  color_init = color

  for contour in c:
    if cv2.contourArea(contour) < 1500:
      continue
    (x, y, w, h) = cv2.boundingRect(contour)
    bottomLeftCornerOfText = (x, y)
    center_x = x + w / 2
    center_y = y + h / 2

    # Crop depth data:
    depth = depth[x:x+w, y:y+h].astype(float)

    depth_crop = depth.copy()

    if depth_crop.size == 0:
      continue
    depth_res = depth_crop[depth_crop != 0]

    # Get data scale from the device and convert to meters
    depth_scale = 0.001  # Assuming depth values are in millimeters
    depth_res = depth_res * depth_scale

    if depth_res.size == 0:
      continue

    dist = min(depth_res)

    cv2.rectangle(res, (x, y), (x + w, y + h), (255, 255, 102), 3)
    text = "Depth: " + str("{0:.2f}").format(dist) + " m"
    cv2.putText(res,
          text,
          bottomLeftCornerOfText,
          cv2.FONT_HERSHEY_SIMPLEX,
          1,
          (255, 255, 255),
          2)

  cv2.imshow('RBG', res)
  cv2.imshow('mask', mask)
  cv2.waitKey(1)

def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber('/camera/color/image_raw', Image, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
