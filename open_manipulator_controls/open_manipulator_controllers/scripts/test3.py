#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import copy
import numpy as np
import math
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

# Initialize ROS node
rospy.init_node('moveit_demo', anonymous=True)

# Initialize MoveIt commander
moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object for the arm
group_name = "right_arm"  # This should be the name of the planning group for your robot's arm
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame for poses
reference_frame = "world"  # Modify this according to your robot's base frame
move_group.set_pose_reference_frame(reference_frame)

# Set the end effector link
end_effector_link = move_group.get_end_effector_link()
 # Set the target pose for the end effector
    

def move_to_pose():
 

    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.pose.position.x = 0.05  # Desired x position
    target_pose.pose.position.y = 0.0  # Desired y position
    target_pose.pose.position.z = 0.05  # Desired z position
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    move_group.set_pose_target(target_pose, "right_end_effector_link")

    # Plan and execute the motion
    plan, _, _, _ = move_group.plan()
   # move_group.execute(plan, wait=True)

    if plan[0]:
        trajectory = plan[1]
        execute_trajectory(trajectory.joint_trajectory)
    else:
        rospy.logerr("Failed to plan trajectory.")

    return 0
  
def execute_trajectory(trajectory):
    # Create a RobotTrajectory message to execute
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = trajectory

    # Execute the trajectory
    pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Allow some time for the publisher to be registered
    pub.publish(robot_trajectory)

    return 0 


if __name__ == '__main__':
   
        print ("Starting...")
        move_to_pose()

     
    