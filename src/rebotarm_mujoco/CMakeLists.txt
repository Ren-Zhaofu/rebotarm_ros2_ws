cmake_minimum_required(VERSION 3.8)
project(rebotarm_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch mjcf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mujoco_config test/test_mujoco_config.py)
endif()

ament_package()
