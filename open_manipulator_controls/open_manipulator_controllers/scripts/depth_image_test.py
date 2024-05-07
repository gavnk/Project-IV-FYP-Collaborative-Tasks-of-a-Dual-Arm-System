#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np





def depth_callback(depth_image):
    # Convert depth image to OpenCV format
    bridge = CvBridge()
    depth_cv = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    # Process depth image to detect ball
    # TODO: Add your ball detection code here
    img = bridge.imgmsg_to_cv2(depth_image, "bgr8")

    # Get ball position or distance
    # TODO: Add your code to get the ball position or distance here

    # Check if the ball moves
    # TODO: Add your code to detect ball movement here



     # Convert the image back to a ROS message
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")

   

    
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
    image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
    rospy.init_node('ball_detection')

    # Publish the image
    image_pub.publish(img_msg)
   


if __name__ == '__main__':
   


   rospy.spin()
