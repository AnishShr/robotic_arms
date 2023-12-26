#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pick_and_place_xarm7");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner (1);
	spinner.start();
	
	moveit::planning_interface::MoveGroupInterface move_group("xarm7");


	geometry_msgs::PoseStamped current_pose; 
	current_pose = move_group.getCurrentPose();

	geometry_msgs::Pose target_pose = current_pose.pose;
	target_pose.position.x += 0.20;
    target_pose.position.z += 0.30;

	move_group.setPoseTarget(target_pose);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::MoveItErrorCode success = move_group.plan(my_plan);

	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pose_goal] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	move_group.execute(my_plan);
	
	std::vector<double> joint_values = move_group.getCurrentJointValues();
	joint_values[0] += 1.60;
	
	move_group.setJointValueTarget(joint_values);
	success = move_group.plan(my_plan);
	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pose_goal] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	
	move_group.execute(my_plan);
	
	current_pose = move_group.getCurrentPose();

	target_pose = current_pose.pose;
	target_pose.position.z -= 0.15;

	move_group.setPoseTarget(target_pose);
	
	success = move_group.plan(my_plan);

	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	move_group.execute(my_plan);

	
	moveit::planning_interface::MoveGroupInterface grip_group("xarm_gripper");
	
    moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
    grip_group.setJointValueTarget("drive_joint", 0.6);

    moveit::planning_interface::MoveItErrorCode grip_success;
	grip_success = grip_group.plan(grip_plan);

    if (grip_success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}

	grip_group.execute(grip_plan);

	current_pose = move_group.getCurrentPose();

	target_pose = current_pose.pose;
	target_pose.position.z += 0.15;

	move_group.setPoseTarget(target_pose);
	
	success = move_group.plan(my_plan);

	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	move_group.execute(my_plan);
	
	joint_values = move_group.getCurrentJointValues();

	joint_values[0] -= 3.00;

	
	move_group.setJointValueTarget(joint_values);
	success = move_group.plan(my_plan);
	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	
	move_group.execute(my_plan);
	
	current_pose = move_group.getCurrentPose();

	target_pose = current_pose.pose;
	target_pose.position.z -= 0.15;

	move_group.setPoseTarget(target_pose);
	
	success = move_group.plan(my_plan);

	if (success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}
	
	move_group.execute(my_plan);

	grip_group.setNamedTarget("open");
	grip_success = grip_group.plan(grip_plan);

    if (grip_success)
	{
		ROS_INFO("[movegroup_interface_demo/pick_and_place_xarm7] planning OK. Proceeding ...");
	}
	else
	{
		ROS_WARN("[movegroup_interface_demo/pick_and_place_xarm7] planning failed.");
		ros::shutdown();
		return 0;
	}

	grip_group.execute(grip_plan);

	ros::shutdown();
	return 0;

}

