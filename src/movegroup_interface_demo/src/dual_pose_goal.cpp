#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#define PI 3.14159265358979323846

int main(int argc, char** argv)
{
    ros::init(argc, argv, "double_controller_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner (1);
    spinner.start();
    
    moveit::planning_interface::MoveGroupInterface move_group("dual");
    
    // === UR5 ===
    
    geometry_msgs::PoseStamped current_pose_ur5;
    current_pose_ur5 = move_group.getCurrentPose ("ur5_tool0");

    geometry_msgs::Pose target_pose = current_pose_ur5.pose;
    target_pose.position.x += 0.3;
    target_pose. position.y -= 0.2;
    target_pose.position.z += 0.0;
    
    move_group.setPoseTarget (target_pose, "ur5_tool0");

    
    geometry_msgs::PoseStamped current_pose_panda;
    current_pose_panda = move_group.getCurrentPose ("franka_arm_link8");
    
    target_pose = current_pose_panda.pose;
    target_pose.position.x -= 0.3;
    target_pose. position.y += 0.1;
    target_pose.position.z -= 0.0;
    
    move_group.setPoseTarget (target_pose, "franka_arm_link7");
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
    
    // Success check handling
    if (success)
    {
    ROS_INFO("[movegroup_interface_demo/dual_pose_goal] Planning OK. Proce");
    }
    else
    {
    ROS_WARN("[movegroup_interface_demo/dual_pose_goal] Planning failed. S");
    ros::shutdown();
    return 0;
    }
    
    move_group.execute(my_plan);

    ROS_INFO("Goal 1 Reached. Plan Executed");

    ros::Duration(5).sleep();

    std::vector<double> joint_values = move_group.getCurrentJointValues();
    
    // // panda
    // joint_values[0] = -42 * (PI/180);
    // joint_values[1] = 27 * (PI/180);
    // joint_values[2] = -38 * (PI/180);
    // joint_values[3] = -97 * (PI/180);
    // joint_values[4] = -25 * (PI/180);
    // joint_values[5] = 98 * (PI/180);
    // joint_values[6] = -50 * (PI/180);
    
    // // UR5
    // joint_values[7] = 64 * (PI/180);
    // joint_values[8] = -36 * (PI/180);
    // joint_values[9] = 43 * (PI/180);
    // joint_values[10] = -25 * (PI/180);
    // joint_values[11] = -50 * (PI/180);
    // joint_values[12] = -66 * (PI/180);

    // panda
    joint_values[0] = -44 * (PI/180);
    joint_values[1] = 45 * (PI/180);
    joint_values[2] = -65 * (PI/180);
    joint_values[3] = -123 * (PI/180);
    joint_values[4] = -10 * (PI/180);
    joint_values[5] = 144 * (PI/180);
    joint_values[6] = -29 * (PI/180);
    
    // UR5
    joint_values[7] = 77 * (PI/180);
    joint_values[8] = 2 * (PI/180);
    joint_values[9] = -54 * (PI/180);
    joint_values[10] = 209 * (PI/180);
    joint_values[11] = 37 * (PI/180);
    joint_values[12] = -239 * (PI/180);

    move_group.setJointValueTarget(joint_values);
	success = move_group.plan(my_plan);
	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/dual_pose_goal] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/dual_pose_goal] planning failed.");
		ros::shutdown();
		return 0;
	}

    move_group.execute(my_plan);

    ROS_INFO("Goal 2 Reached. Plan Executed");

    ros::shutdown();
    return 0;
}