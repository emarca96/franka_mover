#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto const node = std::make_shared<rclcpp::Node>(
      "move_program",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // ROS logger
  auto const logger = rclcpp::get_logger("move_program");

  // Create MoveGroupInterface for the "panda_arm"
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "fr3_arm");

  // Define the goal pose
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -M_PI / 2); // Set the rotation in radians

  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.orientation = msg_quat;
  goal_pose.position.x = 0.3;
  goal_pose.position.y = -0.3;
  goal_pose.position.z = 0.4;

  // Set the goal pose
  move_group_interface.setPoseTarget(goal_pose);

  // Create a plan to move the robot to the goal pose
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool outcome = static_cast<bool>(move_group_interface.plan(plan1));

  // Execute the plan
  if (outcome)
  {
    move_group_interface.execute(plan1);
    RCLCPP_INFO(logger, "Movement executed successfully!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Cannot perform the movement!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
