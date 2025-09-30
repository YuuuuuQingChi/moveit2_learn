#include "geometry_msgs/msg/pose.hpp"
#include <algorithm>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

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
      move_group_node, PLANNING_GROUP, "move_group_tutorial",
      move_group.getRobotModel());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // RCLCPP_INFO(LOGGER, "Planning_Frame %s",
  //             move_group.getPlanningFrame().c_str());
  // auto joint_names = move_group.getJointNames();
  // RCLCPP_INFO(LOGGER, "%s", joint_names[0].c_str());

  // geometry_msgs::msg::Pose target_pose1;
  // target_pose1.orientation.x = -0.28;
  // target_pose1.position.y = -0.2;
  // target_pose1.position.z = 0.5;
  // move_group.setPoseTarget(target_pose1);

  
  // bool is_success =
  //     move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  // RCLCPP_INFO(LOGGER, "%s", is_success ? "ok" : "failed");

  // 获得当前状态
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2);

  std::vector<double> joint_positions;
  current_state->copyJointGroupPositions(joint_module_group, joint_positions);
  for (double tmp : joint_positions) {
    RCLCPP_INFO(LOGGER, "%f", tmp);
  }

  joint_positions[0] = -1.2; // radians
  bool within_bounds = move_group.setJointValueTarget(joint_positions);

  move_group.setMaxAccelerationScalingFactor(0.6);
  move_group.setMaxVelocityScalingFactor(0.6);

  is_success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}