cmake_minimum_required(VERSION 3.8)
project(rebotarm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(planning_demo src/planning_demo.cpp)
ament_target_dependencies(planning_demo control_msgs moveit_ros_planning_interface rclcpp rclcpp_action)

install(TARGETS planning_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py)
endif()

ament_package()
