#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>

// Global variable indicating the state of the arm command:
// 0 = no command, 1 = move to high pose, 2 = move to low pose
int arm_command = 0;

bool triggerMotionCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  if (req.data)
  {
    arm_command = 1;
    resp.message = "moved_to_high_pose";
  }
  else
  {
    arm_command = 2;
    resp.message = "moved_to_low_pose";
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_arm_motion_manager");
  ros::NodeHandle nh;

  // Advertise the service for adjusting arm position
  ros::ServiceServer service = nh.advertiseService("adjust_arm_position", triggerMotionCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load the robot model from the parameter server
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  
  ROS_INFO("Robot model frame: %s", kinematic_model->getModelFrame().c_str());

  // Set up the robot state
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  
  // Get the joint model group for the arm
  const moveit::core::JointModelGroup* arm_joint_model_group = kinematic_model->getJointModelGroup("arm");

  // Initialize MoveGroup for the arm
  moveit::planning_interface::MoveGroupInterface arm_move_group("arm");

  // Retrieve joint names for reference
  const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();

  // Set tolerances
  arm_move_group.setStartStateToCurrentState();
  arm_move_group.setGoalOrientationTolerance(0.01);
  arm_move_group.setGoalPositionTolerance(0.01);

  // Main loop: if arm_command changes, move accordingly
  while (ros::ok())
  {
    if (arm_command == 1)
    {
      // Move to the "upper_stance" named target
      arm_move_group.setNamedTarget("upper_stance");
      arm_move_group.move();
      arm_command = 0;
    }
    else if (arm_command == 2)
    {
      // Move to the "lower_stance" named target
      arm_move_group.setNamedTarget("lower_stance");
      arm_move_group.move();
      arm_command = 0;
    }
  }

  return 0;
}
