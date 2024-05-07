// Gavin Kenny
// Final Year Project 2024
// Task 4: PID control of shared object using camera and ar tags
// Important: comment out on arm static transform publisher and uncomment off arm stp in ar_pose.launch file in open_manipulator_ar_markers package
// this selects the camera setup for on a tripod behind the robots
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>
#include <iostream>

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


#define NUM_OF_JOINT_AND_TOOL 5
#define HOME_POSE   1
#define DEMO_START  2
#define DEMO_STOP   3


typedef struct _ArMarker
{
  uint32_t id;
  double position[3];
} ArMarker;

std::vector<ArMarker> ar_marker_pose;
std::vector<sensor_msgs::Imu> imu_data;
std::vector<sensor_msgs::Imu> gyro_data;

ros::Subscriber open_manipulator_states_sub_;
ros::Subscriber open_manipulator_joint_states_sub_;
ros::Subscriber ar_pose_marker_sub_;
ros::Subscriber imu_sub;
ros::Subscriber gyro_sub;
  // Thread parameter
  
  /*****************************************************************************
  ** Variables
  *****************************************************************************/

pthread_t timer_thread_;
pthread_attr_t attr_;
bool timer_thread_state_;

uint32_t pick_ar_id_ =0;
uint8_t demo_count_=0;

 // PID Controller parameters
const double Kp = 7.00;
const double Ki = 0.1;
const double Kd = 0.01;


// Global variables
double target_position = 0.00;
double current_position = 0.0;

double error_sum = 0.0;
double prev_error = 0.0;
double min_motor_angle = -0.200;
double max_motor_angle = 0.200;

// PID Controller parameters for z-direction
const double Kp_z = 0.3;
const double Ki_z = 0.04;
const double Kd_z = 0.03;

double error_sum_z = 0.0;
double prev_error_z = 0.0;

double min_z = -0.05;
double max_z = 0.05;

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

void both_hold_up(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
 
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = 0.00;
   joint_group_positions[2] = -0.698;
   joint_group_positions[3] = 1.57;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = 0.00;
   joint_group_positions[8] = -0.698;
   joint_group_positions[9] = 1.57;

   group.setJointValueTarget(joint_group_positions);
   group.move();

}


void both_place_down(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
 
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = -0.104;
   joint_group_positions[2] = 0.837;
   joint_group_positions[3] = -0.750;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = -0.104;
   joint_group_positions[8] = 0.837;
   joint_group_positions[9] = -0.750;

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

void both_go_to_pick(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
 
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = 0.034;
   joint_group_positions[2] = 0.802;
   joint_group_positions[3] = -0.767;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = 0.034;
   joint_group_positions[8] = 0.802;
   joint_group_positions[9] = -0.767;

   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void both_go_pick_up(moveit::planning_interface::MoveGroupInterface& group) {
 
   std::vector<double> joint_group_positions;
 
   joint_group_positions = group.getCurrentJointValues();
   //left arm
   joint_group_positions[0]  = 0.00; // joint 1
   joint_group_positions[1] = -0.558;
   joint_group_positions[2] = 0.383;
   joint_group_positions[3] = 0.139;
   // 2 virtual joints 4,5 skip!
   // right arm
   joint_group_positions[6]  = 0.00; // joint 1
   joint_group_positions[7] = -0.558;
   joint_group_positions[8] = 0.383;
   joint_group_positions[9] = 0.139;

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
   joint_group_positions[0]  = 0.00;
   joint_group_positions[1] = -0.00;
   joint_group_positions[2] = 0.296;
   joint_group_positions[3] = 1.186;

   group.setJointValueTarget(joint_group_positions);
   group.move();

}

void set_joint_value_pid(moveit::planning_interface::MoveGroupInterface& group,double joint4, double joint9) {
 
   std::vector<double> joint_group_positions;
   
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = 0.00;
   joint_group_positions[1] = -0.5759;
   joint_group_positions[2] = 0.680;
   joint_group_positions[3] = joint4;
   //                   [4]
   //                   [5]
   joint_group_positions[6] = 0.00;
   joint_group_positions[7] = -0.5759;
   joint_group_positions[8] = 0.680;
   joint_group_positions[9] = joint9;

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
   // temp.orientation[1] = msg->markers.at(i).pose.pose.orientation.y;
    

    temp_buffer.push_back(temp);
  }

  ar_marker_pose = temp_buffer;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // Handle IMU sensor data here

    //  ROS_INFO("Received IMU Data:");
   // ROS_INFO("Orientation -> x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
   // ROS_INFO("Angular Velocity -> x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
   // ROS_INFO("Linear Acceleration -> x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
   // imu_data.orientation.x;

std::vector<sensor_msgs::Imu> imu;

// Handle IMU sensor data here
imu.push_back(*msg);
imu_data = imu;



}

void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg) {

///ROS_INFO("Angular Velocity -> x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

std::vector<sensor_msgs::Imu> imu;
imu.push_back(*msg);
gyro_data = imu;



}





void trajectory_plan(moveit::planning_interface::MoveGroupInterface& group){

  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose target_pose1;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rot.setRPY(0.00,0.78, 0.00);
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
 
    target_pose1.position.z = 0.05;//ar_marker_pose.at(0).position[2];
    group.setPoseTarget(target_pose1);
    // Create a Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(target_pose1);

   group.allowReplanning(true);
   group.setPlanningTime(10.0); 
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit_msgs::RobotTrajectory trajectory;
    double fraction;
   (plan,fraction) = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

   ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);  

  if (fraction == 1.0)
  {
    ROS_INFO("Planning succeeded!");
        // Execute the plan
        group.execute(plan);
        group.move();
        demo_count_=3;
  }
  else
  {
        ROS_ERROR("Failed to plan pose!");
        demo_count_=1;

  }
    
 
    
    ros::Duration(3.0).sleep();
    group.stop();
    group.clearPoseTargets();
    
   return;
  }
  }

}

void trajectory_plan_pid(moveit::planning_interface::MoveGroupInterface& group, double target_position) {

  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose target_pose1;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rot.setRPY(0.00,target_position, 0.00);
  q_rot.normalize();


  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
  if (ar_marker_pose.at(i).id == 1)
  {
    target_pose1.orientation.x = q_rot.x();
    target_pose1.orientation.y = q_rot.y();//ar_marker_pose.at(i).orientation[1]; // q_rot.y();
    target_pose1.orientation.z = q_rot.z();
    target_pose1.orientation.w = q_rot.w();
    target_pose1.position.x = 0.19;
    target_pose1.position.y =-0.15;
 
    target_pose1.position.z = 0.23;//ar_marker_pose.at(0).position[2];
    group.setPoseTarget(target_pose1);

         group.move();
 
    
  

  
    
   return;
  }
  }

}
void trajectory_plan_both_pid(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual, double target_position,double target_position2) {

  tf2::Quaternion q_rotr, q_rotl;
  geometry_msgs::Pose target_pose_r;
  geometry_msgs::Pose target_pose_l;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rotr.setRPY(-target_position,0.00, 0.00);
  q_rotr.normalize();
  q_rotl.setRPY(-target_position,0.00, 0.00);
  q_rotl.normalize();


  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
  if (ar_marker_pose.at(i).id == 1)
  {
    

    target_pose_l.position.x = 0.012;
    target_pose_l.position.y = 0.09;
    target_pose_l.position.z = 0.24 -target_position2;
    target_pose_l.orientation.x = q_rotl.x();
    target_pose_l.orientation.y = q_rotl.y();
    target_pose_l.orientation.z = q_rotl.z();
    target_pose_l.orientation.w = q_rotl.w();

    target_pose_r.position.x = 0.012;
    target_pose_r.position.y = -0.09;
    target_pose_r.position.z = (0.24 +target_position2);
    target_pose_r.orientation.x = q_rotr.x();
    target_pose_r.orientation.y = q_rotr.y();
    target_pose_r.orientation.z = q_rotr.z();
    target_pose_r.orientation.w = q_rotr.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();
  
    
   return;
  }
  }

}

void both_ready_pid( moveit::planning_interface::MoveGroupInterface& group) {

   std::vector<double> joint_group_positions;
   
   joint_group_positions = group.getCurrentJointValues();
   joint_group_positions[0] = -1.57;
   joint_group_positions[1] = -1.32;
   joint_group_positions[2] = 0.191;
   joint_group_positions[3] = 1.16;
   //                   [4]
   //                   [5]
   joint_group_positions[6] = 1.57;
   joint_group_positions[7] = -1.32;
   joint_group_positions[8] = 0.191;
   joint_group_positions[9] = 1.16;

   group.setJointValueTarget(joint_group_positions);
   group.move();
  
    
  

}

void trajectory_plan_left_arm(moveit::planning_interface::MoveGroupInterface& group){

  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose target_pose1;
    ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
  q_rot.setRPY(0.00, 0.78, 0.00);
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
  
    target_pose1.position.z = 0.05;
    group.setPoseTarget(target_pose1);
    // Create a Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(target_pose1);
//  // waypoints.push_back(target_pose2);
   group.allowReplanning(true);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction;
   (plan,fraction) = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

   ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);  

  if (fraction == 1.0)
  {
    ROS_INFO("Planning succeeded!");
        // Execute the plan
        group.execute(plan);
        group.move();
        demo_count_ =5;
  }
  else
  {
        ROS_ERROR("Failed to plan pose!");
        demo_count_=3;

  }

   // group.execute(plan);
   // group.move();
    ros::Duration(3.0).sleep();
    group.stop();
    group.clearPoseTargets();
   
   
     
   return;

   
   }
 }

}

void trajectory_plan_both_arm(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual){

    tf2::Quaternion q_orig, q_rot, q_new;
    geometry_msgs::Pose target_pose_r;
    geometry_msgs::Pose target_pose_l;
        ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
    q_rot.setRPY(0.00, 0.78, 0.00);
    q_rot.normalize();
   // group_right.getCurrentPose();
   // group_left.getCurrentPose();

    
    target_pose_r.position.x = 0.23;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.07;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();

    target_pose_l.position.x = 0.23;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.07;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();

    ros::Duration(1.0).sleep();

    target_pose_r.position.x = 0.16;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.07;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();

    target_pose_l.position.x = 0.16;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.07;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();

    ros::Duration(1.0).sleep();

        target_pose_r.position.x = 0.16;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.11;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();

    target_pose_l.position.x = 0.16;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.11;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();

       ros::Duration(1.0).sleep();

    target_pose_r.position.x = 0.22;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.11;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();

    target_pose_l.position.x = 0.22;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.11;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();

   

 

 

}
void both_place_down_pose(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual)
{
     tf2::Quaternion  q_rot;
    geometry_msgs::Pose target_pose_r;
    geometry_msgs::Pose target_pose_l;
        ///      ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
    q_rot.setRPY(0.00, 0.78, 0.00);
    q_rot.normalize();
   // group_right.getCurrentPose();
   // group_left.getCurrentPose();

    
    target_pose_r.position.x = 0.23;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.05;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();

    target_pose_l.position.x = 0.23;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.05;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();


    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
  
    group_dual.move();

   


}

void both_go_to_pose_AR(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual){

    tf2::Quaternion q_orig, q_rot, q_new;
    geometry_msgs::Pose target_pose_r;
    geometry_msgs::Pose target_pose_l;
        ///        ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
    q_rot.setRPY(0.00, 0.00, 0.00);
    q_rot.normalize();
   // group_right.getCurrentPose();
   // group_left.getCurrentPose();

   for (int i = 0; i < ar_marker_pose.size(); i ++)
   {
   if (ar_marker_pose.at(i).id == 0)
   {
    target_pose_r.position.x = ar_marker_pose.at(i).position[0];
    target_pose_r.position.y = ar_marker_pose.at(i).position[1];
    target_pose_r.position.z = 0.10;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();
    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
     group_dual.move();
   } 
    ros::Duration(1.0).sleep();
    if (ar_marker_pose.at(i).id == 1)
   {


    target_pose_l.position.x = ar_marker_pose.at(i).position[0];
    target_pose_l.position.y = ar_marker_pose.at(i).position[1];
    target_pose_l.position.z = 0.10;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
     group_dual.move();
   }
     
   }
}

   void both_go_to_pick_pose(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual){

    tf2::Quaternion q_orig, q_rot, q_new;
    geometry_msgs::Pose target_pose_r;
    geometry_msgs::Pose target_pose_l;
        ///      ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
    q_rot.setRPY(0.00, 0.00, 0.00);
    q_rot.normalize();
 

    target_pose_r.position.x = 0.25;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.10;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();
    

    target_pose_l.position.x = 0.25;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.10;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();

    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
    group_dual.move();
   
  

}

   void both_go_to_pick_up_pose(moveit::planning_interface::MoveGroupInterface& group_left, moveit::planning_interface::MoveGroupInterface& group_right, moveit::planning_interface::MoveGroupInterface& group_dual){

    tf2::Quaternion q_orig, q_rot, q_new;
    geometry_msgs::Pose target_pose_r;
    geometry_msgs::Pose target_pose_l;
        ///      ROLL, PITCH, YAW  /in RADIANS  // 1.57 radians for 90 degrees
    q_rot.setRPY(0.00, 0.00, 0.00);
    q_rot.normalize();
 

    target_pose_r.position.x = 0.21;
    target_pose_r.position.y = -0.15;
    target_pose_r.position.z = 0.19;
    target_pose_r.orientation.x = q_rot.x();
    target_pose_r.orientation.y = q_rot.y();
    target_pose_r.orientation.z = q_rot.z();
    target_pose_r.orientation.w = q_rot.w();
    

    target_pose_l.position.x = 0.21;
    target_pose_l.position.y = 0.15;
    target_pose_l.position.z = 0.19;
    target_pose_l.orientation.x = q_rot.x();
    target_pose_l.orientation.y = q_rot.y();
    target_pose_l.orientation.z = q_rot.z();
    target_pose_l.orientation.w = q_rot.w();

    group_dual.setPoseTarget(target_pose_r,group_right.getEndEffectorLink());
    group_dual.setPoseTarget(target_pose_l,group_left.getEndEffectorLink());
    group_dual.move();
   
  

}



void printText()
{
 // system("clear");
 //printf("-----------------------------\n");
 printf("AR Tag searching...\n");


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



double pid(double current_position, double target_position, double Kp, double Ki, double Kd, double min_motor_angle, double max_motor_angle){
    double error_sum ;
    double prev_error;
      // Calculate the error
    double error = target_position - current_position;

    // Calculate the PID control output
    double control_output = Kp * error + Ki * error_sum + Kd * (error - prev_error);
    

    // Update the error sum and previous error
    error_sum += error;
    prev_error = error;

    control_output = std::max(min_motor_angle, std::min(max_motor_angle, control_output));


    return control_output;




}

// Function to calculate the pitch angle from accelerometer data
double calculate_pitch(double ax, double ay, double az) {
    // Calculate the pitch angle in radians using atan2 and sqrt
    double pitch_angle_rad = std::atan2(ax, std::sqrt(ay * ay + az * az));

    // Convert the pitch angle from radians to degrees
    double pitch_angle_deg = pitch_angle_rad * (180.0 / M_PI);

    return pitch_angle_deg;
}




int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "open_manipulator_PID");
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
  moveit::planning_interface::MoveGroupInterface group_dual_grippers("dual_grippers");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Subscribe to AR pose marker topic
  ar_pose_marker_sub_ = nh.subscribe("/ar_pose_marker", 10, arPoseMarkerCallback);
   // Subscribe to IMU sensor topic
  imu_sub = nh.subscribe("/left_camera/accel/sample", 10, imuCallback);
  gyro_sub = nh.subscribe("/left_camera/gyro/sample", 10, gyroCallback);
 
  printf("Planning frame: %s\n", group_right.getPlanningFrame().c_str());
  printf("End effector link: %s\n", group_right.getEndEffectorLink().c_str());
  printf("End effector link: %s\n", group_left.getEndEffectorLink().c_str());

  // Get the joint model group for the right arm
  const robot_state::JointModelGroup* joint_model_group =
    group_right.getCurrentState()->getJointModelGroup("right_arm");

  ros::WallDuration(1.0).sleep();

  geometry_msgs::PoseStamped current_pose_right = group_right.getCurrentPose();
  geometry_msgs::PoseStamped current_pose_left = group_left.getCurrentPose();
  ROS_INFO_STREAM("Current right arm end effector pose: " << current_pose_right);
  ROS_INFO_STREAM("Current left arm end effector pose: " << current_pose_left);
  group_right.setMaxVelocityScalingFactor(0.5); 
  group_left.setMaxVelocityScalingFactor(0.5); 
  group_dual_grippers.setMaxVelocityScalingFactor(0.5);
  left_gripper_group.setMaxVelocityScalingFactor(0.5);
  right_gripper_group.setMaxVelocityScalingFactor(0.5);
  //std::vector<double> joint_group_positions;
  group_dual_arms.clearPoseTargets();


  printf("Starting......\n");
  initial_pose_dual(group_dual_arms);
  ros::Duration(1.0).sleep();
  both_ready_pid( group_dual_arms);
  ros::Duration(2.0).sleep();
  open_both_grippers(group_dual_grippers);
  ros::Duration(2.0).sleep();

 
  
  // both_go_to_pick_pose(group_left, group_right, group_dual_arms);
  
  // close_both_grippers(group_dual_grippers);
  // ros::Duration(2.0).sleep();
  // both_go_to_pick_up_pose(group_left, group_right, group_dual_arms);
  // ros::Duration(2.0).sleep();



    while(ar_marker_pose.size()== 0);

    if (ar_marker_pose.size()) 
    {
 
      for (int i = 0; i < ar_marker_pose.size(); i ++)
     {

     

      if (ar_marker_pose.at(i).id == 3)
      {
          close_both_grippers(group_dual_grippers);
      }

     }     
    }






  while(ros::ok()){
    
    printText();
    if (ar_marker_pose.size()) 
    {
     
      for (int i = 0; i < ar_marker_pose.size(); i ++)
     {
        if (ar_marker_pose.at(i).id == 1)
       {
  

        // Get the position of the AR marker
        current_position = ar_marker_pose.at(i).position[1 ];
        // Get error
        double error = target_position - current_position;
        // Calculate the PID control output
        double control_output = Kp * error + Ki * error_sum + Kd * (error - prev_error);
        double control_output_z = Kp_z * error + Ki_z * error_sum_z + Kd_z * (error - prev_error_z);
        // Update the error sum and previous error
        error_sum += error;
        prev_error = error;
        
        error_sum_z += error;
        prev_error_z = error;
        // set the control output limits
        control_output = std::max(min_motor_angle, std::min(max_motor_angle, control_output));
        control_output_z = std::max(min_z, std::min(max_z, control_output_z));
   
       // set_joint_value_pid(group_dual_arms,control_output,control_output);
       // trajectory_plan_pid(group_right, control_output);
       trajectory_plan_both_pid(group_left, group_right, group_dual_arms, control_output,control_output_z);
       

      // Print the current position and control output
      ROS_INFO("Current Position: %f", current_position);
      ROS_INFO("Control Output: %f", control_output);
      ROS_INFO("Control Outputz: %f", control_output_z);

      }
    }
    }
 
    ros::spinOnce();
       
  }


  ros::shutdown();
  return 0;
}


