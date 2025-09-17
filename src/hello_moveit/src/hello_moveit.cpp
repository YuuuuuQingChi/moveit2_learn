#include "geometry_msgs/msg/pose.hpp"
#include "memory"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  auto const logger = rclcpp::get_logger("1212");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node,"manipulator");
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;

    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setRPYTarget(0.0,1.0,0.0);

  auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
 


}();
  rclcpp::shutdown();
  return 0;
}
