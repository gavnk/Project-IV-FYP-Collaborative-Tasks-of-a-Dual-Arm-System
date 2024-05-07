#!/usr/bin/env python3
import sys
import rospy
import copy
import numpy as np
import moveit_commander
import math
import geometry_msgs.msg
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64
from moveit_msgs import msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose ,PoseStamped, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")  
group_gripper = moveit_commander.MoveGroupCommander("gripper")  



# Task Space
   
def pick(ox,oy,oz,ow,px,py,pz):
    planning_frame = group.get_planning_frame()
    pose=geometry_msgs.msg.Pose()
    #pose = group.get_current_pose().pose
   # pose = #geometry_msgs.msg.PoseStamped()
   # pose.header.frame_id = 'world'
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    pose.position.x = px
    pose.position.y = py
    pose.position.z = pz

   # q = quaternion_from_euler(0,pi/2, 0)
    #pose.pose.orientation =Quaternion(q[0],q[1],q[2],q[3])
    group.set_pose_target(pose)
     # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()  # Ensure that there is no residual movement
    group.clear_pose_targets()

    return 0

# pose for end effector
def pickup():
    reference_frame = "world"
    group.set_pose_reference_frame(reference_frame)

    #current_pose = geometry_msgs.msg.PoseStamped()
    current_pose = group.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0,math.pi/2, 0)

   
    target_pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    target_pose.position.x = 0.5
    target_pose.position.y =  0.1
    target_pose.position.z = 1.4
      # Set the target pose
    group.set_pose_target(target_pose)
    group.plan()
    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()  # Ensure that there is no residual movement
    group.clear_pose_targets()

    return 0

def cartesian():

    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

   
    wpose = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(0,math.pi, 0)
    wpose.orientation =Quaternion(q[0],q[1],q[2],q[3])
    wpose.position.x = waypoints[0].position.x + 0.5
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z += 1.0
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))
    group.set_pose_target(wpose)
   
    (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    
    group.execute(plan, wait=True)

    return 0



def plan_cartesian_path(ox,oy,oz,ow,px,py,pz):
    waypoints = []
    group.allow_replanning(True)
    # start with the current pose
    waypoints.append(group.get_current_pose().pose)
    end_effector_link = group.get_end_effector_link()
    start_pose = group.get_current_pose(end_effector_link).pose
    q = quaternion_from_euler(0,math.pi, 0)


    wpose = deepcopy(start_pose)
    wpose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    wpose.orientation.x = ox
    wpose.orientation.y = oy
    wpose.orientation.z = oz
    wpose.orientation.w = ow
    wpose.position.x = px
    wpose.position.y = py
    wpose.position.z = pz
    waypoints.append(deepcopy(wpose))
    if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
    +(wpose.position.x-start_pose.position.x)**2)<0.1:
     rospy.loginfo("Warnig: target position overlaps with the initial position!")

    group.set_pose_target(wpose)

# Set the internal state to the current state
    group.set_start_state_to_current_state()


    plan, fraction = group.compute_cartesian_path(waypoints, 0.02, 0.0, True)
   

    # plan = self.arm.plan()

    # If we have a complete plan, execute the trajectory
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
        group.execute(plan, wait=True)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed")

        # Note: We are just planning, not asking move_group to actually move the robot yet:
       
    #group.stop()
   # group.clear_pose_targets()
    return 0
        



# Command the UR5 to follow the trajectory
def execute_trajectory(waypoints):
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,
                            0.02,  # eef_step
                            0.0)   # jump_threshold

    if fraction == 1.0:
        print("Full trajectory computed!")
        group.execute(plan, wait=True)
    else:
        print("Could not compute full trajectory. Only " + str(fraction*100) + "% achieved.")


# Joint Space

def go_to_home():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.061
    joint_goal[1] = -1.118
    joint_goal[2] = 0.377
    joint_goal[3] = 0.819
    
    gripper_goal[0]= -0.005

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose2():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  -0.485
    joint_goal[1] = 0.089
    joint_goal[2] = 1.157
    joint_goal[3] = -1.193
    
    gripper_goal[0]= -0.005

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose3():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  -0.485
    joint_goal[1] = 0.276
    joint_goal[2] = 0.735
    joint_goal[3] = -0.960
    
    gripper_goal[0]= -0.005

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose4():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  -0.474
    joint_goal[1] = 0.282
    joint_goal[2] = 0.721
    joint_goal[3] = -0.963
    
    gripper_goal[0]= 0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose5():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  -0.003
    joint_goal[1] = -1.011
    joint_goal[2] =  0.075
    joint_goal[3] = 1.017
    
    gripper_goal[0]= 0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose6():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.472
    joint_goal[1] = 0.322
    joint_goal[2] =  0.667
    joint_goal[3] = -0.971
    
    gripper_goal[0]= 0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose7():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.472
    joint_goal[1] = 0.322
    joint_goal[2] =  0.667
    joint_goal[3] = -0.971
    
    gripper_goal[0]= -0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose8():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.482
    joint_goal[1] = 0.094
    joint_goal[2] =  1.115
    joint_goal[3] = -1.184
    
    gripper_goal[0]= -0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose9():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.066
    joint_goal[1] = -1.005
    joint_goal[2] =  0.069
    joint_goal[3] = 1.017
    
    gripper_goal[0]= -0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0

def pose10():

    joint_goal = group.get_current_joint_values()
    gripper_goal = group_gripper.get_current_joint_values()
    joint_goal[0] =  0.060
    joint_goal[1] =  -0.064
    joint_goal[2] =  0.528
    joint_goal[3] = 1.025
    
    gripper_goal[0]= -0.004

    group.go(joint_goal, wait=True)
    group.stop()
    group_gripper.go(gripper_goal, wait=True)
    group_gripper.stop()

    return 0




def open_gripper():
    joint_goal = group_gripper.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  0.010
   
    group_gripper.go(joint_goal, wait=True)
    group_gripper.stop()
    return 0


def close_gripper():
    joint_goal = group_gripper.get_current_joint_values()
    print(joint_goal )
    joint_goal[0] =  -0.0025
   
    group_gripper.go(joint_goal, wait=True)
    group_gripper.stop()
    return 0



  
# Joint Space end


# Main Execution
if __name__ == '__main__':
    
    print ("============ Reference frame: %s" % group.get_planning_frame())
    print ("============ Reference frame EE: %s" % group.get_end_effector_link())
    print ("============ Robot Groups:")
    print (robot.get_group_names())



    go_to_home()
    rospy.sleep(3)
    pose2()
    rospy.sleep(3)
    pose3()
    rospy.sleep(3)
    pose4()
    rospy.sleep(3)
    pose5()
    rospy.sleep(3)
    pose6()
    rospy.sleep(3)
    pose7()
    rospy.sleep(3)
    pose8()
    rospy.sleep(3)
    pose9()
    rospy.sleep(3)
    pose10()
    rospy.sleep(3)
   


    

    print(group.get_current_pose().pose.orientation)
    print(group.get_current_pose().pose.position)
    print("End effector position:")
    print(group.get_current_pose(group.get_end_effector_link()).pose.position)
    print("End effector orientation:")
    print(group.get_current_pose(group.get_end_effector_link()).pose.orientation)
    
   
     

