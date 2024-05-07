#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class RealSenseNode(object):
    def __init__(self):
        rospy.init_node('realsense_processing_node', anonymous=True)

        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback, queue_size=10)
              # Create publishers
        self.color_pub = rospy.Publisher('/processed_color_image', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/processed_depth_image', Image, queue_size=10)
        self.color_image = None
        self.depth_image = None
        self.color_init = None

    def color_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.color_init is None:
                self.color_init = self.color_image.copy()
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_images(self):
        if self.color_image is None or self.depth_image is None:
            return

        color = self.color_image.copy()
        depth = self.depth_image.copy()

        # Convert depth image to 8-bit for display
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

        # Color filtering
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        l_b = np.array([24, 133, 48])
        u_b = np.array([39, 200, 181])
        mask = cv2.inRange(hsv, l_b, u_b)
        color_masked = cv2.bitwise_and(color, color, mask=mask)

        # Motion detection
        d = cv2.absdiff(self.color_init, color)
        gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 1500:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(color, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(depth_colormap, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Updating the initial frame
        self.color_init = color

        # Display
       # cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
      #  cv2.imshow('Color Image', color)
       # cv2.imshow('Depth Image', depth_colormap)
       # cv2.imshow('Mask', mask)
       # cv2.waitKey(1)
        # Publish the images
        self.color_pub.publish(self.bridge.cv2_to_imgmsg(color, "bgr8"))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8"))

if __name__ == '__main__':
    node = RealSenseNode()
    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            node.process_images()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
