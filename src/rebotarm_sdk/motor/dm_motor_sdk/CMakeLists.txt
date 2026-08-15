cmake_minimum_required(VERSION 3.8)
project(rebotarm_dm_motor_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED src/dm_motor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dm_motor_read_parameters tools/dm_motor_read_parameters.cpp)
target_link_libraries(dm_motor_read_parameters ${PROJECT_NAME})

add_executable(dm_motor_read_state tools/dm_motor_read_state.cpp)
target_link_libraries(dm_motor_read_state ${PROJECT_NAME})

add_executable(dm_motor_set_zero tools/dm_motor_set_zero.cpp)
target_link_libraries(dm_motor_set_zero ${PROJECT_NAME})

add_executable(dm_motor_clear_fault tools/dm_motor_clear_fault.cpp)
target_link_libraries(dm_motor_clear_fault ${PROJECT_NAME})

add_executable(dm_motor_minimum_jerk_home tools/dm_motor_minimum_jerk_home.cpp)
target_link_libraries(dm_motor_minimum_jerk_home ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS dm_motor_read_parameters DESTINATION lib/${PROJECT_NAME})
install(TARGETS dm_motor_read_state DESTINATION lib/${PROJECT_NAME})
install(TARGETS dm_motor_set_zero DESTINATION lib/${PROJECT_NAME})
install(TARGETS dm_motor_clear_fault DESTINATION lib/${PROJECT_NAME})
install(TARGETS dm_motor_minimum_jerk_home DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md PROTOCOL.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dm_motor test/test_dm_motor.cpp)
  target_link_libraries(test_dm_motor ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
