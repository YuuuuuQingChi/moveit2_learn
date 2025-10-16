#include <algorithm>
#include <cmath>
#include <moveit/kinematics_base/kinematics_base.hpp>
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
#include <thread>
#include <atomic>

const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

std::atomic<bool> keep_printing{true};

void printJointStates(moveit::planning_interface::MoveGroupInterface& move_group, 
                     const std::string& planning_group) {
    const rclcpp::Logger log = rclcpp::get_logger("joint_theta");
    
    while (keep_printing) {
        try {
            
            auto current_state = move_group.getCurrentState(1.0);
            if (current_state) {
                auto joint_module_group = current_state->getJointModelGroup(planning_group);
                std::vector<double> joint_thetas;
                current_state->copyJointGroupPositions(joint_module_group, joint_thetas);
                
            
                RCLCPP_INFO_THROTTLE(log, *move_group.getNode()->get_clock(), 1000, 
                    "Joint angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                    joint_thetas[0], joint_thetas[1], joint_thetas[2], 
                    joint_thetas[3], joint_thetas[4], joint_thetas[5], joint_thetas[6]);
            } else {
                RCLCPP_WARN_THROTTLE(log, *move_group.getNode()->get_clock(), 2000, 
                    "Failed to get current robot state");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(log, *move_group.getNode()->get_clock(), 5000, 
                "Exception in joint state printing: %s", e.what());
        }
        
      
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("deeply_learn", node_options);

  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  
  RCLCPP_INFO(LOGGER, "Waiting for MoveIt connection...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  
  if (!move_group.getCurrentState()) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to robot state monitor");
    keep_printing = false;
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return -1;
  }

  const moveit::core::JointModelGroup *joint_module_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit_visual_tools::MoveItVisualTools visual_tools(
      move_group_node, "panda_link0", "move_group_tutorial",
      move_group.getRobotModel());
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool is_success;

  
  std::thread printing_thread(printJointStates, std::ref(move_group), PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Starting motion planning...");

  
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  visual_tools.publishText(Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)),
                           "First", rviz_visual_tools::WHITE,
                           rviz_visual_tools::XLARGE);
  visual_tools.trigger();

 
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;  
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.4;
  target_pose1.position.z = 0.8;
  
  move_group.setPoseTarget(target_pose1);
  RCLCPP_INFO(LOGGER, "Planning to first target...");
  
  is_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "First plan %s", is_success ? "SUCCESS" : "FAILED");
  
  if (is_success) {
    move_group.execute(my_plan);
  }

  visual_tools.prompt("Please press Next");

  auto robot_state = move_group.getCurrentState(1);
  std::vector<double> test_joint_theta;
  robot_state->copyJointGroupPositions(joint_module_group,test_joint_theta);\

  test_joint_theta[0] = 1.0;
  test_joint_theta[2] = 1.0;

  move_group.setJointValueTarget(test_joint_theta);
  move_group.setMaxAccelerationScalingFactor(1);
  move_group.setMaxVelocityScalingFactor(1);

  is_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Second plan %s", is_success ? "SUCCESS" : "FAILED");
  if(is_success){
    move_group.execute(my_plan);
  }  
  

  visual_tools.prompt("Next");

  visual_tools.prompt("Close");


  RCLCPP_INFO(LOGGER, "Shutting down...");
  keep_printing = false;
  if (printing_thread.joinable()) {
    printing_thread.join();
  }
  
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  
  rclcpp::shutdown();
  return 0;
}