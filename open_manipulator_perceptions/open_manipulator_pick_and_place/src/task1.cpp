
/*****************************************************************************
// Gavin Kenny
// Final Year Project 2024
// Task 1: Both arm passing object demo using cameras on both arms and AR tags
  *****************************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/JointState.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/thread.hpp>
#include <unistd.h>


typedef struct _ArMarker
{
  uint32_t id;
  double position[3];
} ArMarker;

std::vector<ArMarker> ar_marker_pose;
ros::Subscriber open_manipulator_states_sub_;
ros::Subscriber open_manipulator_joint_states_sub_;
ros::Subscriber ar_pose_marker_sub_;


uint32_t pick_ar_id_ = 0;
uint8_t demo_count_ = 0;

void go_to_home(moveit::planning_interface::MoveGroupInterface& group) {
   
  std::vector<double> joint_group_positions;
  joint_group_positions = group.getCurrentJointValues();
  joint_group_positions[0]  = 0.061;
  joint_group_positions[1] = -1.118;
  joint_group_positions[2] = 0.377;
  joint_group_positions[3] = 0.819;
  group.setJointValueTarget(joint_group_positions);
  group.move();

}

void initial_pose(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = 0.00;
   joint_group_positions[1] = -0.80;
   joint_group_positions[2] = 0.00;
   joint_group_positions[3] = 1.90;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}
void initial_pose_up(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = 0.00;
   joint_group_positions[1] = -1.361;
   joint_group_positions[2] = 0.07;
   joint_group_positions[3] = 1.832;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void initial_pose_dual(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = -0.80;
   joint_group_positions[2] = 0.00;
   joint_group_positions[3] = 1.90;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = -0.80;
   joint_group_positions[8] = 0.00;
   joint_group_positions[9] = 1.90;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void hold_up_pose(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = 1.57;
   joint_group_positions[1] = -0.715;
   joint_group_positions[2] = -0.401;
   joint_group_positions[3] = 1.117;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void left_ready_pose(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = -1.57;
   joint_group_positions[1] = -1.5;
   joint_group_positions[2] = -0.261;
   joint_group_positions[3] = 1.78;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void home_pose(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = 0.00;
   joint_group_positions[1] = -1.43;
   joint_group_positions[2] = 0.139;
   joint_group_positions[3] = 1.32;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void place_pose(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = 0.00;
   joint_group_positions[1] = -0.00;
   joint_group_positions[2] = 0.296;
   joint_group_positions[3] = 1.186;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void close_gripper(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = 0.003;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void open_gripper(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = -0.007;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}


void joint_base_min45(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0]  = -0.785;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  std::vector<ArMarker> temp_buffer;
  for (int i = 0; i < msg->markers.size(); i ++)
  {
    ArMarker temp;
    temp.id = msg->markers.at(i).id;
    temp.position[0] = msg->markers.at(i).pose.pose.position.x;
    temp.position[1] = msg->markers.at(i).pose.pose.position.y;
    temp.position[2] = msg->markers.at(i).pose.pose.position.z;
    temp_buffer.push_back(temp);
  }

  ar_marker_pose = temp_buffer;
}

void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    if (msg->markers.empty())
    {
        ROS_INFO("No AR tag detected.");
    }
    else
    {
        ROS_INFO("AR tag detected!");
        // Additional code to handle the detected tag
    }
}


void set_pose(moveit::planning_interface::MoveGroupInterface& group) {

  group.setStartStateToCurrentState();
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = group.getPlanningFrame();
  target_pose.pose.position.x = 0.17;//ar_marker_pose.at(0).position[0];
  target_pose.pose.position.y = -0.1499;///ar_marker_pose.at(0).position[1];
  target_pose.pose.position.z = 0.038;
  tf2::Quaternion quat;
 // quat.setRPY(0.00, 0.54, 0.00);
  target_pose.pose.orientation.x = 0;///quat.x();
  target_pose.pose.orientation.y = 0.0190928;///quat.y();
  target_pose.pose.orientation.z = 0.000432432;//quat.z();
  target_pose.pose.orientation.w = 1.0;//quat.w();
  //target_pose.header.frame_id = group.getPlanningFrame();
 
  group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if (success)
  {
    ROS_INFO("Planning succeeded!");
        // Execute the plan
        group.execute(my_plan);
  }
  else
  {
        ROS_ERROR("Failed to plan pose!");
  }


  ROS_INFO_STREAM("Target Pose: " << target_pose);
  ROS_INFO_STREAM("Planning result: " << success);

}

void set_waypoints(moveit::planning_interface::MoveGroupInterface& group) {

  std::vector<geometry_msgs::Pose> waypoints;
  double fraction;
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  geometry_msgs::Pose target_pose2 = start_pose;
  target_pose2.position.x += 0.2;
  waypoints.push_back(target_pose2);  
  moveit_msgs::RobotTrajectory trajectory;
  (plan,fraction) = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);   
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(1.0);

  group.execute(plan);
}

void set_waypoints2(moveit::planning_interface::MoveGroupInterface& group) {

  std::vector<geometry_msgs::Pose> waypoints;
  double fraction;
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  waypoints.push_back(start_pose);
  
  start_pose.position.x = 0.0477828;
  start_pose.position.y = -0.1499;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);

  start_pose.position.x = 0.07;
  start_pose.position.y = -0.14;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);

  start_pose.position.x = 0.15;
  start_pose.position.y = -0.1499;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
 
  moveit_msgs::RobotTrajectory trajectory;
  (plan,fraction) = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);   
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(2.0);

  group.execute(plan);
}

void set_waypoints3(moveit::planning_interface::MoveGroupInterface& group) {

  std::vector<geometry_msgs::Pose> waypoints;
  double fraction;
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  waypoints.push_back(start_pose);
  
  start_pose.position.x = 0.0477828;
  start_pose.position.y = -0.1499;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);

  start_pose.position.x = 0.07;
  start_pose.position.y = -0.14;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);

  start_pose.position.x = 0.15;
  start_pose.position.y = -0.1499;
  start_pose.position.z = 0.273912;
  waypoints.push_back(start_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
 
  moveit_msgs::RobotTrajectory trajectory;
  (plan,fraction) = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);   
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(2.0);

  group.execute(plan);
}

void trajectory_plan(moveit::planning_interface::MoveGroupInterface& group){

  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose target_pose1;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rot.setRPY(0.00, 0.0, 0.00);
  q_rot.normalize();

  for (int i = 0; i < ar_marker_pose.size(); i ++){
    if (ar_marker_pose.at(i).id == pick_ar_id_){
      target_pose1.orientation.x = q_rot.x();
      target_pose1.orientation.y = q_rot.y();//ar_marker_pose.at(i).orientation[1]; // q_rot.y();
      target_pose1.orientation.z = q_rot.z();
      target_pose1.orientation.w = q_rot.w();
      target_pose1.position.x = ar_marker_pose.at(i).position[0];//0.19;
      target_pose1.position.y = ar_marker_pose.at(i).position[1];//-0.15;
      target_pose1.position.z = 0.105;//ar_marker_pose.at(0).position[2];
      group.setPoseTarget(target_pose1);
      // Create a Cartesian path
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(target_pose1);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit_msgs::RobotTrajectory trajectory;
      double fraction;
      (plan,fraction) = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
  
      ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);  
      // Execute the Cartesian path
      group.execute(plan);
      group.move();
      ros::Duration(3.0).sleep();
      group.stop();
      group.clearPoseTargets();
      return;
    }
  }

}

void trajectory_plan_left_arm(moveit::planning_interface::MoveGroupInterface& group){

  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose target_pose1;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rot.setRPY(0.00, 0.0, 0.00);
  q_rot.normalize();
  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
    if (ar_marker_pose.at(i).id == 1)
    {
      target_pose1.orientation.x = q_rot.x();
      target_pose1.orientation.y = q_rot.y();//ar_marker_pose.at(i).orientation[1]; // q_rot.y();
      target_pose1.orientation.z = q_rot.z();
      target_pose1.orientation.w = q_rot.w();
      target_pose1.position.x = ar_marker_pose.at(i).position[0];//0.19;
      target_pose1.position.y = ar_marker_pose.at(i).position[1];//-0.15;
     
      target_pose1.position.z = 0.22;//ar_marker_pose.at(0).position[2];
      group.setPoseTarget(target_pose1);
  
      // Create a Cartesian path
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(target_pose1);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit_msgs::RobotTrajectory trajectory;
      double fraction;
      (plan,fraction) = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
  
      ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);  
      group.execute(plan);
      group.move();
      ros::Duration(3.0).sleep();
      group.stop();
      group.clearPoseTargets();
     
  
    }
 }

}


void printText()
{

  printf("Searching...!\n");
  printf("-----------------------------\n");
 // if (ar_marker_pose.size()) printf("AR marker detected.\n");
  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
    printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
           ar_marker_pose.at(i).id,
           ar_marker_pose.at(i).position[0],
           ar_marker_pose.at(i).position[1],
           ar_marker_pose.at(i).position[2]);
  }
}


void close_both_grippers(moveit::planning_interface::MoveGroupInterface& group){
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = 0.003;
   joint_group_positions[2] = 0.003;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void open_both_grippers(moveit::planning_interface::MoveGroupInterface& group){
   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = -0.007;
   joint_group_positions[2] = -0.007;
   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void initial_pose_dual_up(moveit::planning_interface::MoveGroupInterface& group) {

   std::vector<double> joint_group_positions;
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = -1.361;
   joint_group_positions[2] = 0.07;
   joint_group_positions[3] = 1.832;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = -1.361;
   joint_group_positions[8] = 0.07;
   joint_group_positions[9] = 1.832;

   group.setJointValueTarget(joint_group_positions);
   group.move();

}


int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "passing_objects_demo");
  ros::NodeHandle nh;
  ros::Subscriber ar_pose_marker_sub_;
  ros::AsyncSpinner spinner(1); spinner.start();

  ros::WallDuration(1.0).sleep();

  // Create MoveGroupInterface and PlanningSceneInterface objects
  moveit::planning_interface::MoveGroupInterface group_left("left_arm");
  moveit::planning_interface::MoveGroupInterface group_right("right_arm");
  moveit::planning_interface::MoveGroupInterface group_dual_arms("dual_arms");
  moveit::planning_interface::MoveGroupInterface left_gripper_group("left_gripper");
  moveit::planning_interface::MoveGroupInterface right_gripper_group("right_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group_dual_grippers("dual_grippers");
  // Subscribe to AR pose marker topic
  ar_pose_marker_sub_ = nh.subscribe("/ar_pose_marker", 10, arPoseMarkerCallback);

  printf("Planning frame: %s\n", group_right.getPlanningFrame().c_str());
  printf("End effector link: %s\n", group_right.getEndEffectorLink().c_str());

  // Get the joint model group for the right arm
  const robot_state::JointModelGroup* joint_model_group =
    group_right.getCurrentState()->getJointModelGroup("right_arm");

  
  ros::WallDuration(1.0).sleep();


  geometry_msgs::PoseStamped current_pose = group_right.getCurrentPose();
  ROS_INFO_STREAM("End effector pose: " << current_pose);
  group_right.setMaxVelocityScalingFactor(0.5); 
  group_left.setMaxVelocityScalingFactor(0.5); 
 
  
  while (ros::ok()) {
      group_right.clearPoseTargets();
      geometry_msgs::PoseStamped current_pose = group_right.getCurrentPose();

     // perform pick up and passing of the block with an AR Tag on it.
      switch(demo_count_){

        case 0:
          open_both_grippers(group_dual_grippers);
          initial_pose_dual_up(group_dual_arms);
          ros::Duration(1.0).sleep();
          open_both_grippers(group_dual_grippers);
          ros::Duration(1.0).sleep();
          demo_count_++;
          break;
        case 1:
          ros::Duration(1.0).sleep();
          printText();  // print ar_tag position
          demo_count_++;
          break;
        case 2:
          ros::Duration(1.0).sleep();
          if (ar_marker_pose.size()) 
          { 
            printf("AR marker detected.\n");
            //printf("Planning.................\n");
            trajectory_plan(group_right);
     
          }
          else
          {
            printf("No AR tag detected!\n");
            demo_count_=1;     
          }
          demo_count_ ++;
          break;
        case 3:
          ros::Duration(2.0).sleep();
          close_gripper(right_gripper_group);
          demo_count_ ++;
          break;
        case 4:
          ros::Duration(1.0).sleep();
          home_pose(group_right);
          demo_count_ ++;
          break; 
        case 5:
          ros::Duration(1.0).sleep();
          hold_up_pose(group_right);
          demo_count_ ++;
          break; 
        case 6:
          ros::Duration(1.0).sleep();
          left_ready_pose(group_left);
          demo_count_ ++;
          break; 
        case 7:
          ros::Duration(1.0).sleep();
          printText();  // print ar_tag position
          group_left.setMaxVelocityScalingFactor(0.1); 
          demo_count_++;
          break;
        case 8:
          ros::Duration(1.0).sleep();
          if (ar_marker_pose.size()) // if AR marker detected
          {
            printf(" AR marker detected.\n");
            // printf("Planning.................\n");
            trajectory_plan_left_arm(group_left);  
          }
          else
          {
            printf("No AR tag detected!\n");
            demo_count_=7;
              
          }
          demo_count_++;
          break;
        case 9:
          ros::Duration(1.0).sleep();
          close_gripper(left_gripper_group);
          demo_count_++;
          break; 
        case 10:
          ros::Duration(1.0).sleep();
         // group_left.setMaxVelocityScalingFactor(0.1);
         // group_right.setMaxVelocityScalingFactor(0.1);  
         // trajectory_plan_both_arm(group_left, group_right, group_dual_arms);
          demo_count_++;
          break; 
        case 11:
          ros::Duration(1.0).sleep();
          open_gripper(right_gripper_group);    
          demo_count_++;
          break; 
        case 12:
          ros::Duration(1.0).sleep();
          group_left.setMaxVelocityScalingFactor(0.3);
          group_right.setMaxVelocityScalingFactor(0.3); 
          left_ready_pose(group_left);
          demo_count_++;
          break; 
        case 13:
          ros::Duration(1.0).sleep();
          initial_pose_up(group_right);
          open_gripper(right_gripper_group);
          demo_count_++;
          break; 
        case 14:
          ros::Duration(1.0).sleep();
          place_pose(group_left);
          demo_count_++;
          break; 
        case 15:
          ros::Duration(1.0).sleep();
          open_gripper(left_gripper_group);  
          demo_count_++;
          break; 
        case 16:
          ros::Duration(1.0).sleep();
          initial_pose_up(group_left);
          ros::Duration(1.0).sleep();
          open_gripper(left_gripper_group);
          // end do nothing
     }
     ros::spinOnce();
       
  }
  ros::shutdown();
  return 0;
}


