#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <std_srvs/SetBool.h>

bool performMotionPlanCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  // Start a spinner for the callback's MoveIt operations
  ros::AsyncSpinner callback_spinner(1); 
  callback_spinner.start();
  
  // Load the robot model from the parameter server
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  
  ROS_INFO("Using robot model: %s", kinematic_model->getModelFrame().c_str());
  
  // Create a RobotState and JointModelGroup reference for the arm
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* arm_joint_model_group = kinematic_model->getJointModelGroup("arm");

  // Initialize a MoveGroupInterface for controlling the arm
  moveit::planning_interface::MoveGroupInterface arm_motion_group("arm");

  // Retrieve the joint names for debugging
  const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();

  // First target pose
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 0.70;
  target_pose.orientation.x = -0.00;
  target_pose.orientation.y =  0.00;
  target_pose.orientation.z = -0.71;
  target_pose.position.x =  0.50;
  target_pose.position.y =  0.00;
  target_pose.position.z =  1.15;
  
  arm_motion_group.setStartStateToCurrentState();
  arm_motion_group.setApproximateJointValueTarget(target_pose, "arm_link_04");
  
  // Set IK timeout and try solving for IK
  double timeout = 0.1;
  bool ik_solution_found = kinematic_state->setFromIK(arm_joint_model_group, target_pose, timeout);

  std::vector<double> arm_joint_values;
  if (ik_solution_found)
  {
    kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("IK Solution Joint %s: %f", joint_names[i].c_str(), arm_joint_values[i]);
    }
  }
  else
  {
    ROS_WARN("No IK solution found for the first pose.");
  }
  
  // Set the found joint values as the target and update tolerances
  arm_motion_group.setJointValueTarget(arm_joint_values);
  arm_motion_group.setStartStateToCurrentState();
  arm_motion_group.setGoalOrientationTolerance(0.01);
  arm_motion_group.setGoalPositionTolerance(0.01);

  // Plan and execute the first motion
  moveit::planning_interface::MoveGroupInterface::Plan execution_plan;
  arm_motion_group.plan(execution_plan); 
  arm_motion_group.execute(execution_plan);
  
  ROS_INFO("Moved to Position 1");
  ros::Duration(5.0).sleep();
  
  // Second target pose
  target_pose.orientation.w = 0.00;
  target_pose.orientation.x = -0.00;
  target_pose.orientation.y =  0.00;
  target_pose.orientation.z =  1.00;
  target_pose.position.x =  0.50;
  target_pose.position.y =  0.50;
  target_pose.position.z =  1.00;
  
  arm_motion_group.setStartStateToCurrentState();
  arm_motion_group.setApproximateJointValueTarget(target_pose, "arm_link_04");
  ik_solution_found = kinematic_state->setFromIK(arm_joint_model_group, target_pose, timeout);

  if (ik_solution_found)
  {
    kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("IK Solution Joint %s: %f", joint_names[i].c_str(), arm_joint_values[i]);
    }
  }
  else
  {
    ROS_WARN("No IK solution found for the second pose.");
  }

  arm_motion_group.setJointValueTarget(arm_joint_values);
  arm_motion_group.setStartStateToCurrentState();
  arm_motion_group.setGoalOrientationTolerance(0.01);
  arm_motion_group.setGoalPositionTolerance(0.01);

  // Plan and execute the second motion
  arm_motion_group.plan(execution_plan); 
  arm_motion_group.execute(execution_plan);
  
  ROS_INFO("Moved to Position 2");
  ros::Duration(5.0).sleep();
  
  resp.message = "ok";
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_manager");
  ros::NodeHandle nh;
  
  // Advertise the service with the new name
  ros::ServiceServer service = nh.advertiseService("execute_arm_path", performMotionPlanCallback);

  // Use an AsyncSpinner in main to handle callbacks
  ros::AsyncSpinner main_spinner(1);
  main_spinner.start();

  // Keep the node running until shutdown
  ros::waitForShutdown();
  return 0;
}
