# CMake generated Testfile for 
# Source directory: /home/yuqingchi/Code/ws_moveit/src/moveit2/moveit_core/utils/test
# Build directory: /home/yuqingchi/Code/ws_moveit/build/moveit_core/utils/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test-node_logging]=] "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/yuqingchi/Code/ws_moveit/build/moveit_core/test_results/moveit_core/test-node_logging.xunit.xml" "--package-name" "moveit_core" "--output-file" "/home/yuqingchi/Code/ws_moveit/build/moveit_core/launch_test/test-node_logging.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/yuqingchi/Code/ws_moveit/src/moveit2/moveit_core/utils/test/rosout_publish_test.py" "dut:=logger_dut" "--junit-xml=/home/yuqingchi/Code/ws_moveit/build/moveit_core/test_results/moveit_core/test-node_logging.xunit.xml" "--package-name=moveit_core")
set_tests_properties([=[test-node_logging]=] PROPERTIES  LABELS "launch_test" TIMEOUT "60" WORKING_DIRECTORY "/home/yuqingchi/Code/ws_moveit/build/moveit_core/utils/test" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/yuqingchi/Code/ws_moveit/src/moveit2/moveit_core/utils/test/CMakeLists.txt;13;add_launch_test;/home/yuqingchi/Code/ws_moveit/src/moveit2/moveit_core/utils/test/CMakeLists.txt;0;")
