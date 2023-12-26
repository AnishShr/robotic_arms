#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pose_goal");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a MoveGroupInterface object for the arm
    moveit::planning_interface::MoveGroupInterface move_group("xarm_gripper");

    // Set the pose goal
    // geometry_msgs::PoseStamped current_pose;
    // current_pose = move_group.getCurrentPose();

    // geometry_msgs::Pose target_pose = current_pose.pose;

    // target_pose.position.z -= 0.20;

    // move_group.setPoseTarget(target_pose);

    // std::vector<double> joint_values = move_group.getCurrentJointValues();
    // joint_values[0] = (30/180)*3.14;

    move_group.setJointValueTarget("drive_joint", 0.6);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

    if (success)
    {
        ROS_INFO("[movegroup_interface_demo/pose_goal] planning OK. Proceeding ...");
    }
    else
    {
        ROS_WARN("[movegroup_interface_demo/pose_goal] planning failed. Shutting down.");
        ros::shutdown();
        return 0;
    }

    move_group.execute(my_plan);

    ros::shutdown();
    return 0;
}






