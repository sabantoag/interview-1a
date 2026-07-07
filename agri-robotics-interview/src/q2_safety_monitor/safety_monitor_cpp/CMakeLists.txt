cmake_minimum_required(VERSION 3.8)
project(safety_monitor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

# --- library with the safety types/classes candidates implement ---
add_library(safety_types src/safety_types.cpp)
target_include_directories(safety_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# --- given node: publishes synthetic telemetry + fault injection ---
add_executable(telemetry_publisher_node src/telemetry_publisher_node.cpp)
ament_target_dependencies(telemetry_publisher_node rclcpp std_msgs geometry_msgs)

# --- candidate node: evaluates telemetry against safety_types ---
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
target_link_libraries(safety_monitor_node safety_types)
ament_target_dependencies(
  safety_monitor_node rclcpp std_msgs geometry_msgs diagnostic_msgs)

install(TARGETS safety_types DESTINATION lib/${PROJECT_NAME})
install(TARGETS telemetry_publisher_node safety_monitor_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# --- plain assert-based unit test for ViolationTracker (no gtest needed) ---
add_executable(test_safety_types test/test_safety_types.cpp)
target_link_libraries(test_safety_types safety_types)
install(TARGETS test_safety_types DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
