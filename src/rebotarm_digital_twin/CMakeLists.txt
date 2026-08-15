cmake_minimum_required(VERSION 3.8)
project(rebotarm_digital_twin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(rebotarm_digital_twin)

install(PROGRAMS
  rebotarm_digital_twin/state_mirror.py
  rebotarm_digital_twin/target_limiter.py
  rebotarm_digital_twin/target_to_trajectory.py
  rebotarm_digital_twin/mode_arbiter.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_twin_config test/test_twin_config.py)
endif()

ament_package()
