#include "moveit/planning_interface/planning_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "test",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  auto const logger = rclcpp::get_logger("hello_veit");

  auto Planner =
      moveit::planning_interface::MoveGroupInterface(node, "manipulator");

  auto target = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.1;
    msg.position.y = 0.9;
    msg.position.z = 0.6;
    msg.orientation.y = 0.0;
    msg.orientation.x = 3.1415926/2.0;
    return msg;
  }();
  Planner.setPoseTarget(target);
  // 创建collision,要在设置目标姿势和创建计划之间。切记这个顺序

  auto const collision_object = [frame_id = Planner.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    // 盒子的形状 单位m
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();
  moveit::planning_interface::PlanningSceneInterface plan_scene_interface;
  plan_scene_interface.applyCollisionObject(collision_object);

  auto const [success, plan] = [&Planner] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto state = static_cast<bool>(Planner.plan(msg));
    return std::make_pair(state, msg);
  }();

  if (success) {
    Planner.execute(plan);
  }
  rclcpp::shutdown();
  return 0;
}