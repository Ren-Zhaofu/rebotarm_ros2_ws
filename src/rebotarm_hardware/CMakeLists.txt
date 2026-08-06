cmake_minimum_required(VERSION 3.8)
project(rebotarm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rebotarm_dm_motor_sdk REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rebotarm_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rebotarm_dm_motor_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface rebotarm_hardware_plugins.xml
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ros2_control/ DESTINATION share/${PROJECT_NAME}/ros2_control)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  ament_add_gtest(test_rebotarm_system test/test_rebotarm_system.cpp)
  target_link_libraries(test_rebotarm_system ${PROJECT_NAME})
  ament_target_dependencies(test_rebotarm_system
    hardware_interface
    pluginlib
    rclcpp
  )

  add_test(
    NAME test_dm_ros2_control_xacro
    COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/rebotarm_dm.test.urdf.xacro
  )
  add_test(
    NAME test_rs_ros2_control_xacro
    COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/rebotarm_rs.test.urdf.xacro
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rebotarm_dm_motor_sdk
)
ament_package()
