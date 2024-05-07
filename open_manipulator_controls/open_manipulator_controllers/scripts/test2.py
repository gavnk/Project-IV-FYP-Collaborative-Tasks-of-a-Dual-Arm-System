#!/usr/bin/env python3


import sys
import rospy
import moveit_commander
import copy
import numpy as np
import math
import geometry_msgs.msg


rospy.init_node('plan_simple_motion')
rospy.loginfo("Planning simple motions for multiple arms and grippers using MoveIt move group interface")

# ROS spinner is required for the MoveGroupInterface to get the robot state
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


# Set the arms and grippers planning groups names
right_arm_group = "right_arm"
right_gripper_group = "right_gripper"

left_arm_group = "left_arm"
left_gripper_group = "left_gripper"

# Create MoveGroupInterface for each arm and gripper
right_arm_move_group = moveit_commander.MoveGroupCommander(right_arm_group)
right_gripper_move_group = moveit_commander.MoveGroupCommander(right_gripper_group)

left_arm_move_group = moveit_commander.MoveGroupCommander(left_arm_group)
left_gripper_move_group = moveit_commander.MoveGroupCommander(left_gripper_group)

def go_to_home():

    joint_goal = right_arm_move_group.get_current_joint_values()
    gripper_goal = right_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819
    
    gripper_goal[0]= -0.005

    right_arm_move_group.go(joint_goal, wait=True)
    right_arm_move_group.stop()
    right_gripper_move_group.go(gripper_goal, wait=True)
    right_gripper_move_group.stop()

    return 0

def go_to_home2():

    joint_goal = left_arm_move_group.get_current_joint_values()
    gripper_goal = left_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819
    
    gripper_goal[0]= -0.005

    left_arm_move_group.go(joint_goal, wait=True)
    left_arm_move_group.stop()
    left_gripper_move_group.go(gripper_goal, wait=True)
    left_gripper_move_group.stop()

    return 0



if __name__ == '__main__':

    
    print ("Starting...")
    
    go_to_home2()
  

