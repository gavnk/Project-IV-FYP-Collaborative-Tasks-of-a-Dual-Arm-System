#!/usr/bin/env python3


import sys
import rospy
import moveit_commander
import copy
import numpy as np
import math
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('plan_simple_motion', anonymous=True)
#rospy.loginfo("Planning simple motions for multiple arms and grippers using MoveIt move group interface")

# ROS spinner is required for the MoveGroupInterface to get the robot state
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Create MoveGroupInterface for each arm and gripper
right_arm_move_group = moveit_commander.MoveGroupCommander("right_arm")
right_gripper_move_group = moveit_commander.MoveGroupCommander("right_gripper")
left_arm_move_group = moveit_commander.MoveGroupCommander("left_arm")
left_gripper_move_group = moveit_commander.MoveGroupCommander("left_gripper")
both_arms_group =moveit_commander.MoveGroupCommander("dual_arms")

def go_to_home():

    joint_goal = right_arm_move_group.get_current_joint_values()
   # gripper_goal = right_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819
    
   # gripper_goal[0]= -0.005

    right_arm_move_group.go(joint_goal, wait=True)
    right_arm_move_group.stop()
   #right_gripper_move_group.go(gripper_goal, wait=True)
   #right_gripper_move_group.stop()

    return 0

def both_go_to_home():

    joint_goal = both_arms_group.get_current_joint_values()
   # gripper_goal = right_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819

    joint_goal[4] =  0.061
    joint_goal[5] = -1.118
    joint_goal[6] = 0.377
    joint_goal[7] = 0.819
    
   # gripper_goal[0]= -0.005

    both_arms_group.go(joint_goal, wait=True)
    both_arms_group.stop()
   #right_gripper_move_group.go(gripper_goal, wait=True)
   #right_gripper_move_group.stop()

    return 0


def both_go_to_home():

    joint_goal = both_arms_group.get_current_joint_values()
   # gripper_goal = right_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819

    joint_goal[4] =  0.061
    joint_goal[5] = -1.118
    joint_goal[6] = 0.377
    joint_goal[7] = 0.819
    
   # gripper_goal[0]= -0.005

    both_arms_group.go(joint_goal, wait=True)
    both_arms_group.stop()
   #right_gripper_move_group.go(gripper_goal, wait=True)
   #right_gripper_move_group.stop()

    return 0


def go_to_home2():

    joint_goal = left_arm_move_group.get_current_joint_values()
   # gripper_goal = left_gripper_move_group.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819
    
   # gripper_goal[0]= -0.005

    left_arm_move_group.go(joint_goal, wait=True)
    left_arm_move_group.stop()
   # left_gripper_move_group.go(gripper_goal, wait=True)
    #left_gripper_move_group.stop()

    return 0

def right_open_gripper():
    joint_goal = right_gripper_move_group.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  -0.007
   # //joint_goal[1] =  -1.0
   
    right_gripper_move_group.go(joint_goal, wait=True)
    right_gripper_move_group.stop()
    return 0


def right_close_gripper():
    joint_goal = right_gripper_move_group.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  0.001
   # //joint_goal[1] =  0.3
   
    right_gripper_move_group.go(joint_goal, wait=True)
    right_gripper_move_group.stop()
    return 0

def left_open_gripper():
    joint_goal = left_gripper_move_group.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  -0.007
  # // joint_goal[1] =  -1.0
   
    left_gripper_move_group.go(joint_goal, wait=True)
    left_gripper_move_group.stop()
    return 0


def left_close_gripper():
    joint_goal = left_gripper_move_group.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  0.001
   # joint_goal[1] =  0.3
   
    left_gripper_move_group.go(joint_goal, wait=True)
    left_gripper_move_group.stop()
    return 0



if __name__ == '__main__':
    
    print("right gripper joint values:")
    print(right_gripper_move_group.get_current_joint_values())
    
    print ("Starting...")
    both_go_to_home()
    rospy.sleep(1)
    right_open_gripper()
    print("right gripper joint values:")
    print(right_gripper_move_group.get_current_joint_values())
    rospy.sleep(1)
    right_close_gripper()
    print("right gripper joint values:")
    print(right_gripper_move_group.get_current_joint_values())
    rospy.sleep(1)
    left_open_gripper()
    rospy.sleep(1)
    left_close_gripper()
   # go_to_home2  # left arm
  

