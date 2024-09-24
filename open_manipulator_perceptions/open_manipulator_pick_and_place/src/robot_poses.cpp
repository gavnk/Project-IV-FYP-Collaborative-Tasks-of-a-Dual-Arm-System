// robot_poses.cpp
#include "robot_poses.h"
// Gavin Kenny
// Final Year Project 2024




// Constructor
RobotPoses::RobotPoses()
{



    // Initialize move groups and store them in the map
    move_groups_["left_arm"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_arm");
    move_groups_["right_arm"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_arm");
    move_groups_["dual_arms"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("dual_arms");
    move_groups_["left_gripper"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_gripper");
    move_groups_["right_gripper"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_gripper");
    move_groups_["dual_grippers"] = std::make_shared<moveit::planning_interface::MoveGroupInterface>("dual_grippers");

}

void RobotPoses::printText() const
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

// Function to move to a specified joint position
void RobotPoses::goToPosition(const std::string& move_group,const std::vector<double>& target_positions)
{
    std::vector<double> joint_group_positions = move_groups_[move_group]->getCurrentJointValues();



    // Set the target joint values
    for (size_t i = 0; i < joint_group_positions.size(); i++) {
        joint_group_positions[i] = target_positions[i];
    }

    // Set the joint target and move
    move_groups_[move_group]->setJointValueTarget(joint_group_positions);
    move_groups_[move_group]->move();
}

void RobotPoses::grippers_control(const std::string& move_group, const std::vector<double>& target_positions){
   std::vector<double> joint_group_positions = move_groups_[move_group]->getCurrentJointValues();

    joint_group_positions[0] = target_positions[0];
    joint_group_positions[2] = target_positions[2];
   
    move_groups_[move_group]->setJointValueTarget(joint_group_positions);
    move_groups_[move_group]->move();
    


}

// Function to perform trajectory planning
bool RobotPoses::trajectoryPlan(const std::string& move_group,
                                  int target_id,
                                  double z_height)
{
    tf2::Quaternion q_rot;
    q_rot.setRPY(0.00, 0.0, 0.00); // Default ROLL, PITCH, YAW values
    q_rot.normalize();

    geometry_msgs::Pose target_pose;
    
    // Search for the target ID in the provided ar_marker_pose list
    for (int i = 0; i < ar_marker_pose.size(); i ++)
    {
        if (ar_marker_pose.at(i).id == target_id)  
        {
            target_pose.orientation.x = q_rot.x();
            target_pose.orientation.y = q_rot.y();
            target_pose.orientation.z = q_rot.z();
            target_pose.orientation.w = q_rot.w();
            target_pose.position.x = ar_marker_pose.at(i).position[0];
            target_pose.position.y = ar_marker_pose.at(i).position[1];
            target_pose.position.z = z_height; // Set z height

            // Set the target pose for the group
            move_groups_[move_group]->setPoseTarget(target_pose);

            // Create a Cartesian path with the waypoints
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit_msgs::RobotTrajectory trajectory;
            double fraction = move_groups_[move_group]->computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

            ROS_INFO("Visualizing plan (%.2f%% achieved)", fraction * 100.0);

            // Check if a significant fraction of the path was planned successfully
            if (fraction < 0.9)
            {
                ROS_WARN("Path planning was incomplete or failed.");
                return false;
            }

            // Execute the trajectory
            //plan.trajectory_ = trajectory;
            move_groups_[move_group]->execute(plan);

            // Final move
            move_groups_[move_group]->move();
            
            move_groups_[move_group]->stop();
            ros::Duration(3.0).sleep();
            move_groups_[move_group]->clearPoseTargets();

            return true;
        }
    }

    ROS_WARN("Target ID not found in ar_marker_pose.");
    return false;
}


void RobotPoses::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
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

void RobotPoses::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
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

// Function to move to the home position
void RobotPoses::home_pose(const std::string& group_name)
{
    goToPosition(group_name, home_position_);
}


void RobotPoses::hold_up_pose(const std::string& group_name)
{
    goToPosition(group_name,hold_up_position_);
}


void RobotPoses::left_ready_pose(const std::string& group_name)
{
    goToPosition(group_name,left_ready_position_);
}

void RobotPoses::open_both_grippers(const std::string& group_name){

    grippers_control(group_name,open_both_grippers_);
}

void RobotPoses::close_both_grippers(const std::string& group_name){

    grippers_control(group_name,close_both_grippers_);
}


void RobotPoses::initial_pose_dual_up(const std::string& group_name){

    goToPosition(group_name,initial_pose_dual_up_);
}

void RobotPoses::close_gripper(const std::string& group_name){

    goToPosition(group_name,close_gripper_);
}

void RobotPoses::open_gripper(const std::string& group_name){

    goToPosition(group_name,open_gripper_);
}

void RobotPoses::initial_pose_up(const std::string& group_name){

    goToPosition(group_name,initial_pose_up_);
}

void RobotPoses::place_pose(const std::string& group_name)
{
    
    goToPosition(group_name, place_pose_);
}








