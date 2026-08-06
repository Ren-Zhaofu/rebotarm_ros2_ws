cmake_minimum_required(VERSION 3.8)
project(rs_motor_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED src/rs_motor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(rs_motor_read_state tools/rs_motor_read_state.cpp)
target_link_libraries(rs_motor_read_state ${PROJECT_NAME})
add_executable(rs_motor_set_zero tools/rs_motor_set_zero.cpp)
target_link_libraries(rs_motor_set_zero ${PROJECT_NAME})
add_executable(rs_motor_home tools/rs_motor_home.cpp)
target_link_libraries(rs_motor_home ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rs_motor_read_state DESTINATION lib/${PROJECT_NAME})
install(TARGETS rs_motor_set_zero DESTINATION lib/${PROJECT_NAME})
install(TARGETS rs_motor_home DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rs_motor test/test_rs_motor.cpp)
  target_link_libraries(test_rs_motor ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
