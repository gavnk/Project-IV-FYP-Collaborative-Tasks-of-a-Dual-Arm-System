// robot_poses.h
// Gavin Kenny
// Final Year Project 2024
#ifndef ROBOT_POSES_H
#define ROBOT_POSES_H


#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <vector>
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

// Ensure that ar_marker_pose is declared as an extern variable
extern std::vector<ArMarker> ar_marker_pose;

class RobotPoses
{
public:
    // Constructor
    RobotPoses();
    // class functions
    void printText() const;
    // Function to move to a specified joint position
    void goToPosition(const std::string& move_group,const std::vector<double>& target_positions);
    void grippers_control(const std::string& move_group, const std::vector<double>& target_positions);
    void home_pose(const std::string& group_name);
    void hold_up_pose(const std::string& group_name);
    void left_ready_pose(const std::string& group_name);
    void open_both_grippers(const std::string& group_name);
    void close_both_grippers(const std::string& group_name);
    void initial_pose_dual_up(const std::string& group_name);
    void close_gripper(const std::string& group_name);
    void open_gripper(const std::string& group_name);
    void initial_pose_up(const std::string& group_name);
    void place_pose(const std::string& group_name);
    void arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    
    bool trajectoryPlan(const std::string& move_group,
                    int target_id,
                    double z_height);

  


 

private:
    // Store MoveGroupInterface objects in a map for easier management
    std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups_;

    // vectors storing the angles values for the different poses in Direct Kinematics
    // edit here or add more
    const std::vector<double> home_position_ = {0.00, -1.43, 0.139, 1.32};    
    const std::vector<double> hold_up_position_ = {1.57,-0.715,-0.401,1.117};      
    const std::vector<double> left_ready_position_ = {-1.57,-1.5,-0.261,1.78};
    const std::vector<double> open_both_grippers_ = {-0.007,0,-0.007,0}; 
    const std::vector<double> close_both_grippers_ = {0.003,0,0.003,0};
    const std::vector<double> initial_pose_dual_up_ = {0.00,-1.361,0.07,1.832,0.0,0.0,0.00,-1.361,0.07,1.832};     
    const std::vector<double> close_gripper_ = {0.003};
    const std::vector<double> open_gripper_ = {-0.007};
    const std::vector<double> initial_pose_up_ = {0.00,-1.361,0.07,1.832}; 
    const std::vector<double> place_pose_ = {0.00,-0.00,0.296,1.186};     
};

#endif // ROBOT_POSES_H  