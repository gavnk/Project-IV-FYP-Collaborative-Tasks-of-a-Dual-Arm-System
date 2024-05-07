#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ObjectLevelChecker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_level_checker', anonymous=True)

        # Set up a subscriber to the RealSense camera image topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        # Set up a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        level, processed_image = self.is_object_level(cv_image)
        if level:
            print("The object is level.")
        else:
            print("The object is not level.")
        
        # Display the image with lines
        cv2.imshow("Level Detection", processed_image)
        cv2.waitKey(1)

    def is_object_level(self, image):
         # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blur, 40, 150, apertureSize=3)
        # Find lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=30, maxLineGap=10)

        
        level = False

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                if abs(angle) < 10 or abs(angle) > 170:
                    level = True
                    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw horizontal line in green

        return level, image

if __name__ == '__main__':
    # Instantiate the object level checker
    checker = ObjectLevelChecker()
    
    # Spin to keep the script from exiting until the node is stopped
    rospy.spin()
    cv2.destroyAllWindows()
