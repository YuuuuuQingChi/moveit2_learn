cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

# 编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(graph_msgs REQUIRED)


# 可执行文件
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(plan src/Obstacle_avoidance_planning.cpp)
add_executable(deep src/deeply_learn.cpp)

# 通用设置函数
function(setup_target target)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(${target}
    moveit_ros_planning_interface
    rclcpp
    moveit_visual_tools
    graph_msgs
    )
endfunction()

# 应用设置
setup_target(hello_moveit)
setup_target(plan)
setup_target(deep)

# 安装
install(TARGETS hello_moveit plan deep
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()