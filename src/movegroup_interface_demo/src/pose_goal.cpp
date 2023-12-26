#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pose_goal");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a MoveGroupInterface object for the arm
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

    // Set the pose goal
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose();

    geometry_msgs::Pose target_pose = current_pose.pose;

    target_pose.position.z -= 0.20;

    move_group.setPoseTarget(target_pose);

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

    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.5;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.5;
    // arm_group.setPoseTarget(target_pose);

    // // Plan the trajectory to the pose goal
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // bool success = (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //     // Execute the planned trajectory
    //     arm_group.execute(plan);
    //     ROS_INFO("Pose goal successfully executed!");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to plan pose goal!");
    // }

    return 0;
}






