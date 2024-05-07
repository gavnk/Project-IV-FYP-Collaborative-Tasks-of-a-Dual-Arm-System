#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import copy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped


    


rospy.init_node('moveit_end_effector_pose', anonymous=True)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("right_arm")  # Change "manipulator" to the name of your MoveIt group

while not rospy.is_shutdown():
    pose = group.get_current_pose().pose
    print("End effector pose:", pose)
    rospy.sleep(1.0)

     
    