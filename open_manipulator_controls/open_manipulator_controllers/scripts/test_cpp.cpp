

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_end_effector_pose");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface group("right_arm"); // Change "right_arm" to the name of your MoveIt group
    group.setPlannerId("RRTConnectkConfigDefault"); // Optional: Set planner

    while (ros::ok()) {
        geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
        ROS_INFO_STREAM("End effector pose: " << current_pose);
        ros::Duration(1.0).sleep();
    }

    return 0;
}
