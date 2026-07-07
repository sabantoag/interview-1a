cmake_minimum_required(VERSION 3.8)
project(geometry_overlap_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Boost.Geometry and Boost.Geometry.Index are header-only -- no libraries to
# link, just headers. On Ubuntu: `sudo apt install libboost-dev`.
find_package(Boost REQUIRED)

include_directories(include)

# --- library with the geometry algorithms candidates implement ---
add_library(geometry_types src/geometry_types.cpp)
target_include_directories(geometry_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(geometry_types PUBLIC Boost::boost)
ament_target_dependencies(geometry_types geometry_msgs)

# --- given node: publishes mock field/tractor geometry ---
add_executable(geometry_provider_node src/geometry_provider_node.cpp)
ament_target_dependencies(geometry_provider_node rclcpp std_msgs geometry_msgs)

# --- candidate node: subscribes + calls into geometry_types ---
add_executable(overlap_checker_node src/overlap_checker_node.cpp)
target_link_libraries(overlap_checker_node geometry_types)
ament_target_dependencies(overlap_checker_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  geometry_types
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  geometry_provider_node
  overlap_checker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- plain assert-based unit test for the algorithms (no gtest needed) ---
add_executable(test_geometry_types test/test_geometry_types.cpp)
target_link_libraries(test_geometry_types geometry_types)
ament_target_dependencies(test_geometry_types geometry_msgs)
install(TARGETS test_geometry_types DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
