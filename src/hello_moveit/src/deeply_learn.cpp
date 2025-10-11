// #include "geometry_msgs/msg/pose.hpp"
// #include <Eigen/src/Geometry/Transform.h>
// #include <Eigen/src/Geometry/Translation.h>
#include <algorithm>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <utility>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("deeply_learn", node_options);

  // 关键部分，如果不执行这块的线程，无法连接到cilent
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  //
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  bool is_success;
  const moveit::core::JointModelGroup *joint_module_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit_visual_tools::MoveItVisualTools visual_tools(
      move_group_node, "panda_link0", "move_group_tutorial",
      move_group.getRobotModel());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  visual_tools.publishText(Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)),
                           "First", rviz_visual_tools::WHITE,
                           rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.4;
  target_pose1.position.z = 0.8;
  move_group.setPoseTarget(target_pose1);
  is_success =
      move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  RCLCPP_INFO(LOGGER, "%s", is_success ? "ok" : "failed");
  if (is_success) {
    // move_group.execute(my_plan);
  }

  visual_tools.prompt("Please press Next");

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2);

  std::vector<double> joint_positions;
  current_state->copyJointGroupPositions(joint_module_group, joint_positions);
  for (double tmp : joint_positions) {
    RCLCPP_INFO(LOGGER, "%f", tmp);
  }

  joint_positions[0] = -1.0; // radians
  bool within_bounds = move_group.setJointValueTarget(joint_positions);

  move_group.setMaxAccelerationScalingFactor(1);
  move_group.setMaxVelocityScalingFactor(1);

  is_success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // move_group.execute(my_plan);
  visual_tools.prompt("Next");
  moveit_msgs::msg::OrientationConstraint ocm;
  // 要求机器人在从起点到终点的整个运动过程中，末端执行器（panda_link7）相对于基座（panda_link0）的朝向必须：

  // 保持接近无旋转状态（w=1.0的四元数）

  // 在X、Y、Z三个旋转轴上的偏差不超过0.1弧度

  // 在整个路径上都满足这个约束，而不仅仅在起点和终点
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  //
  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_module_group, start_pose2);
  move_group.setStartState(start_state);
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(10.0);
  is_success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s",
              is_success ? "" : "FAILED");

  if (is_success) {
    // move_group.execute(my_plan);
    RCLCPP_INFO(LOGGER, "yeah");
  }
  visual_tools.prompt("Close");
  move_group.clearPathConstraints();
  return 0;
}