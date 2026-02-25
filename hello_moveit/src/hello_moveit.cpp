

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;


  MoveGroupInterface move_group_interface(node, "ur_manipulator");


  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.2);

  geometry_msgs::msg::Pose target_pose;


  // 180 degree rotation about the z axis, effector should face down
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  target_pose.position.x = 0.55;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.3;

  
  move_group_interface.setStartStateToCurrentState();

 
  move_group_interface.setGoalOrientationTolerance(0.05);

  move_group_interface.setPoseTarget(target_pose);

  RCLCPP_INFO(logger, "Planning frame: %s",
            move_group_interface.getPlanningFrame().c_str());

  // Plan
  MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  // Execute
  if (success) {
    RCLCPP_INFO(logger, "Planning successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
