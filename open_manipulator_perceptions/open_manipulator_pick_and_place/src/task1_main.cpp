
// Gavin Kenny
// Final Year Project 2024
// Task 1: Both arm passing object demo using cameras on both arms and AR tags
/* Notes
   1. In simulation the grippers open and close in the opposite way, then with the hardware, just ignore it.
   2. when using simulation for testing, comment out the trajectoryPlan function calls. Since
   you will not be using the ar tags.
   
*/

#include <ros/ros.h>
#include "robot_poses.h"


uint8_t robot_state_ = 0;
uint8_t ar_id_ = 0;

std::vector<ArMarker> ar_marker_pose;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Task 1: Passing_object_demo");
  ros::NodeHandle nh;
  ros::Subscriber ar_pose_marker_sub_;
  ros::AsyncSpinner spinner(1); spinner.start();
  RobotPoses robotPoses;
 
  ar_pose_marker_sub_ = nh.subscribe("/ar_pose_marker", 10, &RobotPoses::arPoseMarkerCallback,&robotPoses);

  

  while(ros::ok()){

    switch(robot_state_){
      case 0: // initial position
        robotPoses.initial_pose_dual_up("dual_arms");
        robotPoses.open_both_grippers("dual_grippers");
        ros::Duration(1.0).sleep();  
        robot_state_++;
        break;
      case 1: // wait
        ros::Duration(1.0).sleep();
        robotPoses.printText();
        robot_state_++;
        break;
      case 2: // marker detected, go to object
      
        //if (ar_marker_pose.size()) 
       // {
        //  ROS_INFO("AR marker detected.");
        //  robotPoses.trajectoryPlan("right_arm",ar_id_,0.105);
          robot_state_++;
       // }else{
       //   ROS_WARN("No AR tag detected!");
        //  robot_state_ = 1;
        //}
       
        break;
      case 3: // close gripper
        robotPoses.close_gripper("right_gripper");
        robot_state_++;
        break;
      case 4: // go home
        robotPoses.home_pose("right_arm");
        robot_state_++;
        break;
      case 5: // hold up
        robotPoses.hold_up_pose("right_arm");
        robot_state_++;
        break;
      case 6: // left ready
        robotPoses.left_ready_pose("left_arm");
        robot_state_++;
        break;
      case 7: // wait
        ros::Duration(1.0).sleep();
        robotPoses.printText();
        
        robot_state_++;
        break;
      case 8: // marker detect, move in
      
       // if (ar_marker_pose.size()) // if AR marker detected
       // {
       //   ROS_INFO("AR marker detected.");     
       //   robotPoses.trajectoryPlan("left_arm",1,0.22); 
          robot_state_++;  
       // }
       // else
       // {
        //  ROS_WARN("No AR tag detected!");
        //  robot_state_=7;
       // }
        
        break;
      case 9: // close left gripper
        robotPoses.close_gripper("left_gripper");
        robot_state_++;
        break;
      case 10: // wait
        ros::Duration(1.0).sleep();
        robot_state_++;
        break;
      case 11: // open right gripper
        robotPoses.open_gripper("right_gripper");
        robot_state_++;
        break;
      case 12: // left ready

        robotPoses.left_ready_pose("left_arm");
        robot_state_++;
        break;
      case 13: // go initial pose, open right gripper
        robotPoses.initial_pose_dual_up("right_arm");
        robotPoses.open_gripper("right_gripper");
        robot_state_++;
        break;
      case 14: // left place down
        robotPoses.place_pose("left_arm");
        robot_state_++;
        break;
      case 15: // open left gripper
        robotPoses.open_gripper("left_gripper");
        robot_state_++;
        break;
      case 16: // go back
        robotPoses.initial_pose_dual_up("left_arm");
        robotPoses.open_gripper("left_gripper");
      default:
        ROS_WARN("Unexpected robot state.");
        break;

    }

    
  }

    ros::shutdown();
    return 0;
}