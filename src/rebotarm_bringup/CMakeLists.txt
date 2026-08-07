cmake_minimum_required(VERSION 3.8)
project(rebotarm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup_config test/test_bringup_config.py)
endif()

ament_package()
