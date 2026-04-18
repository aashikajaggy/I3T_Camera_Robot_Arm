
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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

  // cache that stores all of the transforms in the system
  tf2_ros::Buffer tf_buffer(node->get_clock());
  // continuously listens to all of the transforms 
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // wait a moment for transforms to arrive before actually using them
  rclcpp::sleep_for(std::chrono::seconds(1));

  geometry_msgs::msg::TransformStamped marker_tf;

  try {
    marker_tf = tf_buffer.lookupTransform(
      "base_link",   
      "marker",   
      tf2::TimePointZero,
      tf2::durationFromSec(1.0)
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger, "TF lookup failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }


  geometry_msgs::msg::Pose target_pose;


  // 180 degree rotation about the z axis, effector should face down
  target_pose.position.x = marker_tf.transform.translation.x;
  target_pose.position.y = marker_tf.transform.translation.y;
  target_pose.position.z = marker_tf.transform.translation.z;

  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  
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
